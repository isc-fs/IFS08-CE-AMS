// SPDX-License-Identifier: proprietary
//
// Pure-logic tests for ams::logfs::Server -- the LOGFS handlers (#406 / #439)
// driven against a fake backend, so no card, no FatFs, no RTOS.
//
// The cases that matter operationally are the ugly ones: a stale handle after
// a session drop, a card yanked mid-pull, a host asking for more than we will
// serve, and a listing that spans several replies. A happy-path-only suite
// would pass while the pit tool hangs.

#include "logfs_server.hpp"

#include "unity.h"

#include <cstdint>
#include <cstdio>

namespace {

using namespace ams;

// ---------------------------------------------------------------------------
// Fake backend: a handful of in-memory "files".
// ---------------------------------------------------------------------------
class FakeFs {
public:
    static constexpr std::uint16_t kFileCount = 5u;

    bool present   = true;
    bool fail_seek = false;
    bool fail_read = false;
    bool fail_crc  = false;
    bool sidecar_missing = false;
    int  opens     = 0;
    int  closes    = 0;

    [[nodiscard]] bool card_present() const noexcept { return present; }

    bool list_begin(std::uint16_t cursor) noexcept {
        if (fail_seek) return false;
        cursor_ = cursor;
        return true;
    }

    bool list_next(logfs::Entry& out) noexcept {
        if (cursor_ >= kFileCount) return false;
        out.index = cursor_;
        out.size  = size_of(cursor_);
        out.mtime = 0x5A000000u + cursor_;
        std::snprintf(out.name, sizeof out.name, "LOG%04u.CSV", cursor_);
        ++cursor_;
        return true;
    }

    bool open(std::uint16_t index, std::uint32_t& size_out,
              std::uint32_t& crc_out) noexcept {
        if (index >= kFileCount) return false;
        ++opens;
        size_out = size_of(index);
        crc_out  = sidecar_missing ? 0u : (0xC0FFEE00u + index);
        return true;
    }

    int read(std::uint16_t index, std::uint32_t off, std::uint8_t* out,
             std::uint16_t len) noexcept {
        if (fail_read) return -1;
        const std::uint32_t sz = size_of(index);
        if (off >= sz) return 0;                       // EOF -> short read
        std::uint32_t n = sz - off;
        if (n > len) n = len;
        for (std::uint32_t i = 0; i < n; ++i) {
            out[i] = static_cast<std::uint8_t>((off + i) & 0xFFu);   // known pattern
        }
        return static_cast<int>(n);
    }

    bool crc32(std::uint16_t index, std::uint32_t& crc_out) noexcept {
        if (fail_crc) return false;
        crc_out = 0xC0FFEE00u + index;
        return true;
    }

    void close(std::uint16_t) noexcept { ++closes; }

    static std::uint32_t size_of(std::uint16_t i) noexcept {
        return 100u * (static_cast<std::uint32_t>(i) + 1u);   // 100,200,300,400,500
    }

private:
    std::uint16_t cursor_ = 0;
};

// Build a request in place. Returns a Request viewing `buf`.
struct Req {
    std::uint8_t  buf[32] = {};
    std::uint16_t len     = 0;
    diag::Request req{};

    Req(std::uint8_t opcode, const std::uint8_t* args, std::uint16_t alen) {
        buf[0] = static_cast<std::uint8_t>(diag::MsgType::AppCtrl);
        buf[1] = opcode;
        for (std::uint16_t i = 0; i < alen; ++i) buf[2 + i] = args[i];
        len = static_cast<std::uint16_t>(2 + alen);
        (void)diag::parse_request(buf, len, req);
    }
};

std::uint8_t g_out[isotp::MaxMsg];

// OPEN file `index`, return its handle (0 if the open was rejected).
template <class S>
std::uint16_t do_open(S& srv, std::uint16_t index) {
    std::uint8_t a[2];
    diag::put_u16(a, index);
    Req r(diag::OpLogfsOpen, a, 2);
    const std::uint16_t n = srv.handle(r.req, g_out, sizeof g_out);
    if (n != 12u || g_out[0] != static_cast<std::uint8_t>(diag::MsgType::Ack)) return 0;
    return diag::get_u16(g_out + 2);
}

}  // namespace

// --- LIST -------------------------------------------------------------------

extern "C" void test_logfs_list_returns_all_entries(void) {
    FakeFs fs;
    logfs::Server<FakeFs> srv(fs);
    std::uint8_t a[2];
    diag::put_u16(a, 0u);
    Req r(diag::OpLogfsList, a, 2);

    const std::uint16_t n = srv.handle(r.req, g_out, sizeof g_out);
    TEST_ASSERT_EQUAL_HEX8(0x01, g_out[0]);
    TEST_ASSERT_EQUAL_HEX8(diag::OpLogfsList, g_out[1]);
    TEST_ASSERT_EQUAL_HEX16(logfs::CursorEnd, diag::get_u16(g_out + 2));  // all fit
    TEST_ASSERT_EQUAL_UINT8(FakeFs::kFileCount, g_out[4]);
    TEST_ASSERT_EQUAL_UINT16(5u + FakeFs::kFileCount * diag::LogfsEntryLen, n);
}

// The host parses by 22-byte stride, so the layout of entry N is wire.
extern "C" void test_logfs_list_entry_layout(void) {
    FakeFs fs;
    logfs::Server<FakeFs> srv(fs);
    std::uint8_t a[2];
    diag::put_u16(a, 0u);
    Req r(diag::OpLogfsList, a, 2);
    (void)srv.handle(r.req, g_out, sizeof g_out);

    const std::uint8_t* e1 = g_out + 5 + diag::LogfsEntryLen;   // second entry
    TEST_ASSERT_EQUAL_UINT16(1u, diag::get_u16(e1));
    TEST_ASSERT_EQUAL_UINT32(200u, diag::get_u32(e1 + 2));
    TEST_ASSERT_EQUAL_UINT32(0x5A000001u, diag::get_u32(e1 + 6));
    TEST_ASSERT_EQUAL_STRING("LOG0001.CSV", reinterpret_cast<const char*>(e1 + 10));
}

// A cursor past the end is legal and simply lists nothing -- the host's
// termination check must not depend on us erroring.
extern "C" void test_logfs_list_cursor_past_end_is_empty(void) {
    FakeFs fs;
    logfs::Server<FakeFs> srv(fs);
    std::uint8_t a[2];
    diag::put_u16(a, 99u);
    Req r(diag::OpLogfsList, a, 2);
    const std::uint16_t n = srv.handle(r.req, g_out, sizeof g_out);
    TEST_ASSERT_EQUAL_UINT16(5u, n);
    TEST_ASSERT_EQUAL_UINT8(0u, g_out[4]);
    TEST_ASSERT_EQUAL_HEX16(logfs::CursorEnd, diag::get_u16(g_out + 2));
}

// With a small buffer the listing must paginate rather than truncate silently.
extern "C" void test_logfs_list_paginates_when_reply_is_full(void) {
    FakeFs fs;
    logfs::Server<FakeFs> srv(fs);
    std::uint8_t small[5 + 2 * diag::LogfsEntryLen];

    std::uint8_t a[2];
    diag::put_u16(a, 0u);
    Req r0(diag::OpLogfsList, a, 2);
    (void)srv.handle(r0.req, small, sizeof small);
    TEST_ASSERT_EQUAL_UINT8(2u, small[4]);
    const std::uint16_t next = diag::get_u16(small + 2);
    TEST_ASSERT_EQUAL_UINT16(2u, next);          // more to come

    diag::put_u16(a, next);
    Req r1(diag::OpLogfsList, a, 2);
    (void)srv.handle(r1.req, small, sizeof small);
    TEST_ASSERT_EQUAL_UINT8(2u, small[4]);
    TEST_ASSERT_EQUAL_UINT16(4u, diag::get_u16(small + 2));

    diag::put_u16(a, 4u);
    Req r2(diag::OpLogfsList, a, 2);
    (void)srv.handle(r2.req, small, sizeof small);
    TEST_ASSERT_EQUAL_UINT8(1u, small[4]);       // last one
    TEST_ASSERT_EQUAL_HEX16(logfs::CursorEnd, diag::get_u16(small + 2));
}

extern "C" void test_logfs_list_fs_error_nacks(void) {
    FakeFs fs;
    fs.fail_seek = true;
    logfs::Server<FakeFs> srv(fs);
    std::uint8_t a[2];
    diag::put_u16(a, 0u);
    Req r(diag::OpLogfsList, a, 2);
    (void)srv.handle(r.req, g_out, sizeof g_out);
    TEST_ASSERT_EQUAL_HEX8(0x02, g_out[0]);
    TEST_ASSERT_EQUAL_HEX8(diag::NackFsError, g_out[2]);
}

// --- OPEN / CLOSE -----------------------------------------------------------

extern "C" void test_logfs_open_returns_handle_and_size(void) {
    FakeFs fs;
    logfs::Server<FakeFs> srv(fs);
    const std::uint16_t h = do_open(srv, 2u);
    TEST_ASSERT_NOT_EQUAL(logfs::NoHandle, h);
    TEST_ASSERT_EQUAL_UINT32(300u, diag::get_u32(g_out + 4));
    TEST_ASSERT_EQUAL_HEX32(0xC0FFEE02u, diag::get_u32(g_out + 8));  // sealed CRC
    TEST_ASSERT_TRUE(srv.has_open_file());
}

extern "C" void test_logfs_open_missing_file_nacks(void) {
    FakeFs fs;
    logfs::Server<FakeFs> srv(fs);
    std::uint8_t a[2];
    diag::put_u16(a, 99u);
    Req r(diag::OpLogfsOpen, a, 2);
    (void)srv.handle(r.req, g_out, sizeof g_out);
    TEST_ASSERT_EQUAL_HEX8(0x02, g_out[0]);
    TEST_ASSERT_EQUAL_HEX8(diag::NackFileNotFound, g_out[2]);
    TEST_ASSERT_FALSE(srv.has_open_file());
}

// A client that lost its ACK retries OPEN; that must not leak the first file.
extern "C" void test_logfs_reopen_supersedes_and_closes_previous(void) {
    FakeFs fs;
    logfs::Server<FakeFs> srv(fs);
    const std::uint16_t h1 = do_open(srv, 0u);
    const std::uint16_t h2 = do_open(srv, 1u);
    TEST_ASSERT_EQUAL_INT(1, fs.closes);            // the first one was closed
    TEST_ASSERT_NOT_EQUAL(h1, h2);                  // and its handle retired
}

extern "C" void test_logfs_close_releases_handle(void) {
    FakeFs fs;
    logfs::Server<FakeFs> srv(fs);
    const std::uint16_t h = do_open(srv, 0u);
    std::uint8_t a[2];
    diag::put_u16(a, h);
    Req r(diag::OpLogfsClose, a, 2);
    const std::uint16_t n = srv.handle(r.req, g_out, sizeof g_out);
    TEST_ASSERT_EQUAL_UINT16(2u, n);
    TEST_ASSERT_EQUAL_HEX8(0x01, g_out[0]);
    TEST_ASSERT_FALSE(srv.has_open_file());
    TEST_ASSERT_EQUAL_INT(1, fs.closes);
}

// release() is what the session timeout calls -- an operator who unplugs
// mid-pull must not pin a file handle forever.
extern "C" void test_logfs_release_closes_open_file(void) {
    FakeFs fs;
    logfs::Server<FakeFs> srv(fs);
    (void)do_open(srv, 0u);
    srv.release();
    TEST_ASSERT_EQUAL_INT(1, fs.closes);
    TEST_ASSERT_FALSE(srv.has_open_file());
    srv.release();                                   // idempotent
    TEST_ASSERT_EQUAL_INT(1, fs.closes);
}

// --- READ -------------------------------------------------------------------

extern "C" void test_logfs_read_returns_requested_bytes(void) {
    FakeFs fs;
    logfs::Server<FakeFs> srv(fs);
    const std::uint16_t h = do_open(srv, 4u);        // 500 bytes

    std::uint8_t a[8];
    diag::put_u16(a, h);
    diag::put_u32(a + 2, 10u);
    diag::put_u16(a + 6, 64u);
    Req r(diag::OpLogfsRead, a, 8);
    const std::uint16_t n = srv.handle(r.req, g_out, sizeof g_out);

    TEST_ASSERT_EQUAL_UINT16(2u + 64u, n);
    TEST_ASSERT_EQUAL_HEX8(0x01, g_out[0]);
    TEST_ASSERT_EQUAL_HEX8(diag::OpLogfsRead, g_out[1]);
    TEST_ASSERT_EQUAL_HEX8(10u, g_out[2]);           // pattern == offset
    TEST_ASSERT_EQUAL_HEX8(73u, g_out[2 + 63]);
}

// Short read == EOF. The host relies on this to stop, so it must be an ACK.
extern "C" void test_logfs_read_past_eof_is_short_ack_not_error(void) {
    FakeFs fs;
    logfs::Server<FakeFs> srv(fs);
    const std::uint16_t h = do_open(srv, 0u);        // 100 bytes

    std::uint8_t a[8];
    diag::put_u16(a, h);
    diag::put_u32(a + 2, 80u);
    diag::put_u16(a + 6, 512u);
    Req r(diag::OpLogfsRead, a, 8);
    TEST_ASSERT_EQUAL_UINT16(2u + 20u, srv.handle(r.req, g_out, sizeof g_out));
    TEST_ASSERT_EQUAL_HEX8(0x01, g_out[0]);

    diag::put_u32(a + 2, 100u);                      // exactly at EOF
    Req r2(diag::OpLogfsRead, a, 8);
    TEST_ASSERT_EQUAL_UINT16(2u, srv.handle(r2.req, g_out, sizeof g_out));
    TEST_ASSERT_EQUAL_HEX8(0x01, g_out[0]);
}

// An over-large request is clamped, not rejected -- a client guessing 4096
// still makes progress instead of hard-failing.
extern "C" void test_logfs_read_clamps_oversized_request(void) {
    FakeFs fs;
    logfs::Server<FakeFs> srv(fs);
    const std::uint16_t h = do_open(srv, 4u);        // 500 bytes

    std::uint8_t a[8];
    diag::put_u16(a, h);
    diag::put_u32(a + 2, 0u);
    diag::put_u16(a + 6, 4096u);
    Req r(diag::OpLogfsRead, a, 8);
    const std::uint16_t n = srv.handle(r.req, g_out, sizeof g_out);
    TEST_ASSERT_EQUAL_UINT16(2u + 500u, n);          // clamped to 512, EOF at 500
}

// The failure this prevents: a handle from a previous session silently reading
// whatever file happens to be open now.
extern "C" void test_logfs_read_with_stale_handle_nacks(void) {
    FakeFs fs;
    logfs::Server<FakeFs> srv(fs);
    const std::uint16_t h = do_open(srv, 0u);
    srv.release();                                   // session dropped
    (void)do_open(srv, 1u);                          // someone else opened another

    std::uint8_t a[8];
    diag::put_u16(a, h);                             // the OLD handle
    diag::put_u32(a + 2, 0u);
    diag::put_u16(a + 6, 16u);
    Req r(diag::OpLogfsRead, a, 8);
    (void)srv.handle(r.req, g_out, sizeof g_out);
    TEST_ASSERT_EQUAL_HEX8(0x02, g_out[0]);
    TEST_ASSERT_EQUAL_HEX8(diag::NackBadHandle, g_out[2]);
}

extern "C" void test_logfs_read_without_open_nacks(void) {
    FakeFs fs;
    logfs::Server<FakeFs> srv(fs);
    std::uint8_t a[8] = {};
    diag::put_u16(a, 1u);
    Req r(diag::OpLogfsRead, a, 8);
    (void)srv.handle(r.req, g_out, sizeof g_out);
    TEST_ASSERT_EQUAL_HEX8(diag::NackBadHandle, g_out[2]);
}

extern "C" void test_logfs_read_io_error_nacks(void) {
    FakeFs fs;
    logfs::Server<FakeFs> srv(fs);
    const std::uint16_t h = do_open(srv, 0u);
    fs.fail_read = true;

    std::uint8_t a[8];
    diag::put_u16(a, h);
    diag::put_u32(a + 2, 0u);
    diag::put_u16(a + 6, 16u);
    Req r(diag::OpLogfsRead, a, 8);
    (void)srv.handle(r.req, g_out, sizeof g_out);
    TEST_ASSERT_EQUAL_HEX8(0x02, g_out[0]);
    TEST_ASSERT_EQUAL_HEX8(diag::NackReadError, g_out[2]);
}

extern "C" void test_logfs_read_truncated_args_nacks(void) {
    FakeFs fs;
    logfs::Server<FakeFs> srv(fs);
    const std::uint16_t h = do_open(srv, 0u);
    std::uint8_t a[4];
    diag::put_u16(a, h);
    Req r(diag::OpLogfsRead, a, 4);                  // missing offset/len
    (void)srv.handle(r.req, g_out, sizeof g_out);
    TEST_ASSERT_EQUAL_HEX8(0x02, g_out[0]);
    TEST_ASSERT_EQUAL_HEX8(diag::NackOutOfBounds, g_out[2]);
}

// --- CRC --------------------------------------------------------------------

extern "C" void test_logfs_crc_returns_value(void) {
    FakeFs fs;
    logfs::Server<FakeFs> srv(fs);
    const std::uint16_t h = do_open(srv, 3u);
    std::uint8_t a[2];
    diag::put_u16(a, h);
    Req r(diag::OpLogfsCrc, a, 2);
    const std::uint16_t n = srv.handle(r.req, g_out, sizeof g_out);
    TEST_ASSERT_EQUAL_UINT16(6u, n);
    TEST_ASSERT_EQUAL_HEX32(0xC0FFEE03u, diag::get_u32(g_out + 2));
}

extern "C" void test_logfs_crc_error_nacks(void) {
    FakeFs fs;
    logfs::Server<FakeFs> srv(fs);
    const std::uint16_t h = do_open(srv, 0u);
    fs.fail_crc = true;
    std::uint8_t a[2];
    diag::put_u16(a, h);
    Req r(diag::OpLogfsCrc, a, 2);
    (void)srv.handle(r.req, g_out, sizeof g_out);
    TEST_ASSERT_EQUAL_HEX8(diag::NackReadError, g_out[2]);
}

// --- card / dispatch --------------------------------------------------------

// "No card" must be its own answer so an operator reseats the card instead of
// wondering where the logs went.
extern "C" void test_logfs_no_card_nacks_every_opcode(void) {
    FakeFs fs;
    fs.present = false;
    logfs::Server<FakeFs> srv(fs);
    const std::uint8_t ops[] = {diag::OpLogfsList, diag::OpLogfsOpen, diag::OpLogfsRead,
                                diag::OpLogfsCrc,  diag::OpLogfsClose};
    for (std::uint8_t op : ops) {
        std::uint8_t a[8] = {};
        Req r(op, a, 8);
        (void)srv.handle(r.req, g_out, sizeof g_out);
        TEST_ASSERT_EQUAL_HEX8(0x02, g_out[0]);
        TEST_ASSERT_EQUAL_HEX8(diag::NackNoSdCard, g_out[2]);
    }
}

extern "C" void test_logfs_owns_only_its_opcodes(void) {
    TEST_ASSERT_TRUE(logfs::Server<FakeFs>::owns(diag::OpLogfsList));
    TEST_ASSERT_TRUE(logfs::Server<FakeFs>::owns(diag::OpLogfsClose));
    TEST_ASSERT_FALSE(logfs::Server<FakeFs>::owns(diag::OpConnect));
    TEST_ASSERT_FALSE(logfs::Server<FakeFs>::owns(0x26u));   // DELETE stays unowned
}

// v1 is read-only: a DELETE attempt must be refused, not ignored.
extern "C" void test_logfs_unknown_opcode_nacks_unsupported(void) {
    FakeFs fs;
    logfs::Server<FakeFs> srv(fs);
    std::uint8_t a[2] = {};
    Req r(0x26u, a, 2);
    (void)srv.handle(r.req, g_out, sizeof g_out);
    TEST_ASSERT_EQUAL_HEX8(0x02, g_out[0]);
    TEST_ASSERT_EQUAL_HEX8(diag::NackUnsupported, g_out[2]);
}

// A whole-file pull, the way the client actually drives it.
extern "C" void test_logfs_full_file_pull_sequence(void) {
    FakeFs fs;
    logfs::Server<FakeFs> srv(fs);
    const std::uint16_t h = do_open(srv, 4u);        // 500 bytes
    TEST_ASSERT_NOT_EQUAL(logfs::NoHandle, h);

    std::uint32_t off   = 0;
    int           reads = 0;
    for (;;) {
        std::uint8_t a[8];
        diag::put_u16(a, h);
        diag::put_u32(a + 2, off);
        diag::put_u16(a + 6, 128u);
        Req r(diag::OpLogfsRead, a, 8);
        const std::uint16_t n = srv.handle(r.req, g_out, sizeof g_out);
        TEST_ASSERT_EQUAL_HEX8(0x01, g_out[0]);
        const std::uint16_t got = static_cast<std::uint16_t>(n - 2u);
        for (std::uint16_t i = 0; i < got; ++i) {
            TEST_ASSERT_EQUAL_HEX8((off + i) & 0xFFu, g_out[2 + i]);
        }
        off += got;
        if (++reads > 10) TEST_FAIL_MESSAGE("read loop did not terminate");
        if (got < 128u) break;                       // short read = EOF
    }
    TEST_ASSERT_EQUAL_UINT32(500u, off);
    TEST_ASSERT_EQUAL_INT(4, reads);                 // 128*3 + 116
}

// --- OPEN carries the sealed CRC (#452 wire contract) -----------------------

// The host reads [handle:u16][size:u32][crc32:u32] by fixed offset, so the
// reply length and field order are wire.
extern "C" void test_logfs_open_reply_is_ten_bytes(void) {
    FakeFs fs;
    logfs::Server<FakeFs> srv(fs);
    std::uint8_t a[2];
    diag::put_u16(a, 1u);
    Req r(diag::OpLogfsOpen, a, 2);
    const std::uint16_t n = srv.handle(r.req, g_out, sizeof g_out);

    TEST_ASSERT_EQUAL_UINT16(2u + 10u, n);
    TEST_ASSERT_EQUAL_HEX8(0x01, g_out[0]);
    TEST_ASSERT_EQUAL_UINT32(200u, diag::get_u32(g_out + 4));
    TEST_ASSERT_EQUAL_HEX32(0xC0FFEE01u, diag::get_u32(g_out + 8));
}

// crc32 == 0 is the agreed "not available, use LOGFS_CRC" marker -- a log
// written before sidecars existed. OPEN must still succeed and must NOT stall
// computing one.
extern "C" void test_logfs_open_reports_zero_crc_when_no_sidecar(void) {
    FakeFs fs;
    fs.sidecar_missing = true;
    logfs::Server<FakeFs> srv(fs);
    std::uint8_t a[2];
    diag::put_u16(a, 0u);
    Req r(diag::OpLogfsOpen, a, 2);
    const std::uint16_t n = srv.handle(r.req, g_out, sizeof g_out);

    TEST_ASSERT_EQUAL_UINT16(2u + 10u, n);
    TEST_ASSERT_EQUAL_HEX8(0x01, g_out[0]);                          // still an ACK
    TEST_ASSERT_EQUAL_UINT32(100u, diag::get_u32(g_out + 4));        // size still valid
    TEST_ASSERT_EQUAL_HEX32(0u, diag::get_u32(g_out + 8));           // "unknown"
}
