// SPDX-License-Identifier: proprietary
//
// LOGFS command handlers (#406 / #439) -- LIST / OPEN / READ / CRC / CLOSE.
//
// Read-only by design: v1 serves sealed LOGnnnn.CSV files and cannot delete or
// modify anything on the card. An extraction tool that can also erase evidence
// is the wrong tool to hand a pit crew.
//
// FILESYSTEM OWNERSHIP -- why a Backend template parameter
// --------------------------------------------------------
// FatFs IS reentrant in this build (ffconf.h `_FS_REENTRANT 1`, `_SYNC_t
// osSemaphoreId_t`, with option/syscall.c compiled in), so a separate diag
// thread calling f_read would be memory-safe -- FatFs serialises per volume.
// The server still runs ON the logger thread, for three reasons that are
// about scheduling and headroom rather than data races:
//
//  1. `_SYNC_t` is an osSemaphoreId_t, i.e. a plain counting semaphore with NO
//     priority inheritance. Contending for the volume across threads invites
//     unbounded priority inversion; keeping one owner sidesteps it.
//  2. `_FS_LOCK 2` caps simultaneously-open files at two. The logger holds the
//     active .TMP, so a diag read is the second and last slot -- zero headroom
//     for a concurrent third open.
//  3. A diag thread costs another stack plus a ~550-byte FIL, and SdLoggerTask
//     already owns the volume (it defines hsd1, mounts, writes, rotates).
//
// So: one thread touches FatFs, matching the lock-free single-writer services.
// The template keeps that zero-cost and lets the host tests drive a fake
// backend with no card, no HAL and no RTOS.
//
// WIRE FORMAT (little-endian; msg_type APP_CTRL 0x06 -- see diag_proto.hpp)
//
//   LIST  args [cursor:u16]                -> ack [next_cursor:u16][count:u8][entry × count]
//         entry = {index:u16, size:u32, mtime:u32, name[12]}  (22 B, stride-parsed)
//         next_cursor == CursorEnd (0xFFFF) means the listing is complete.
//   OPEN  args [index:u16]                 -> ack [handle:u16][size:u32][crc32:u32]
//         crc32 == 0 means "not available -- use LOGFS_CRC", not a zero CRC.
//   READ  args [handle:u16][offset:u32][len:u16]
//                                          -> ack [data × n], n <= min(len, 512)
//         A SHORT read means end-of-file. That is the host's normal
//         termination signal, so a truncated reply is not an error.
//   CRC   args [handle:u16]                -> ack [crc32:u32]  (whole file)
//   CLOSE args [handle:u16]                -> ack []
//   FINAL args []                          -> ack [index:u16]  (0x27)
//         Seals the ACTIVE log and returns its index, so the run that just
//         happened is immediately listable/openable. NACKs FILE_NOT_FOUND when
//         there is nothing to seal.

#pragma once

#include "diag_proto.hpp"

#include <cstdint>

namespace ams::logfs {

// Sentinel returned in next_cursor when there are no more entries.
inline constexpr std::uint16_t CursorEnd = 0xFFFFu;

// A handle is never 0, so 0 is a safe "nothing open" marker and a host bug
// that sends a zeroed handle gets a clean BAD_HANDLE instead of hitting file 0.
inline constexpr std::uint16_t NoHandle = 0u;

// One directory entry, as the backend reports it.
struct Entry {
    std::uint16_t index = 0;      // LOGnnnn -> nnnn
    std::uint32_t size  = 0;      // bytes
    std::uint32_t mtime = 0;      // FAT timestamp, 0 if the RTC was unset
    char          name[diag::LogfsNameLen + 1] = {};  // NUL-terminated 8.3
};

// What a Backend must provide. Not a base class -- documentation for the
// template contract. All calls run on the filesystem-owning thread.
//
//   bool  card_present()
//   bool  list_begin(u16 cursor)              // seek; false on FS error
//   bool  list_next(Entry& out)               // false = no more entries
//   bool  open(u16 index, u32& size_out, u32& crc_out)   // crc 0 = unknown
//   int   read(u16 index, u32 off, u8* out, u16 len)   // -1 err, else n
//   bool  crc32(u16 index, u32& crc_out)
//   void  close(u16 index)
//   bool  finalize(u16& sealed_index_out)   // false = nothing to seal
//
// list_* is an ITERATOR rather than a fill-my-array call on purpose: the
// server runs on SdLoggerTask, whose stack is 4 KiB and shared with FatFs. A
// 46-entry array plus a staged reply body would have put ~2 KiB of that on the
// stack. Streaming one Entry at a time straight into the reply buffer costs
// ~24 bytes instead.

// ---------------------------------------------------------------------------
// Server -- decodes one LOGFS request and builds the reply. Stateless except
// for the open handle.
// ---------------------------------------------------------------------------
template <class Backend>
class Server {
public:
    explicit Server(Backend& be) noexcept : be_(be) {}

    // True if `opcode` is one this server owns. Enumerated rather than a range
    // so 0x26 (DELETE, deliberately unimplemented -- v1 is read-only) keeps
    // falling through to NACK UNSUPPORTED as the range grows around it.
    [[nodiscard]] static bool owns(std::uint8_t opcode) noexcept {
        switch (opcode) {
        case diag::OpLogfsList:
        case diag::OpLogfsOpen:
        case diag::OpLogfsRead:
        case diag::OpLogfsCrc:
        case diag::OpLogfsClose:
        case diag::OpLogfsFinalize: return true;
        default:                    return false;
        }
    }

    // Handle a request. Returns the reply length written to `out` (always > 0:
    // every request gets either an ACK or a NACK). `out` must hold at least
    // isotp::MaxMsg bytes.
    [[nodiscard]] std::uint16_t handle(const diag::Request& req,
                                       std::uint8_t*        out,
                                       std::uint16_t        cap) noexcept {
        // No card is its own answer -- distinguishable from an empty card, so
        // an operator knows to reseat rather than wonder where the logs went.
        if (!be_.card_present()) return nack(out, cap, req.opcode, diag::NackNoSdCard);

        switch (req.opcode) {
            case diag::OpLogfsList:  return do_list(req, out, cap);
            case diag::OpLogfsOpen:  return do_open(req, out, cap);
            case diag::OpLogfsRead:  return do_read(req, out, cap);
            case diag::OpLogfsCrc:   return do_crc(req, out, cap);
            case diag::OpLogfsClose: return do_close(req, out, cap);
            case diag::OpLogfsFinalize: return do_finalize(req, out, cap);
            default:                 return nack(out, cap, req.opcode, diag::NackUnsupported);
        }
    }

    // Drop any open file. Called when the session ends or times out, so an
    // operator who unplugs mid-pull does not leave a handle pinned.
    void release() noexcept {
        if (open_handle_ != NoHandle) {
            be_.close(open_index_);
            open_handle_ = NoHandle;
        }
    }

    [[nodiscard]] bool has_open_file() const noexcept { return open_handle_ != NoHandle; }

private:
    static std::uint16_t nack(std::uint8_t* out, std::uint16_t cap,
                              std::uint8_t opcode, std::uint8_t code) noexcept {
        return diag::build_nack(out, cap, opcode, code);
    }

    // Map a caller-supplied handle to the open file. Rejects a stale handle
    // from before a session drop, which would otherwise silently read whatever
    // file happens to be open now.
    [[nodiscard]] bool resolve(std::uint16_t h) const noexcept {
        return open_handle_ != NoHandle && h == open_handle_;
    }

    std::uint16_t do_list(const diag::Request& req, std::uint8_t* out,
                          std::uint16_t cap) noexcept {
        if (req.arg_len < 2u) return nack(out, cap, req.opcode, diag::NackOutOfBounds);
        const std::uint16_t cursor = diag::get_u16(req.args);
        if (cursor == CursorEnd)  return nack(out, cap, req.opcode, diag::NackOutOfBounds);

        // Reply is assembled in place: [Ack][opcode][next:u16][count:u8][entries]
        const std::uint16_t hdr = 5u;
        if (cap < static_cast<std::uint16_t>(hdr + diag::LogfsEntryLen)) {
            return nack(out, cap, req.opcode, diag::NackOutOfBounds);
        }
        // How many entries this reply can hold: bounded by both the caller's
        // buffer and the ISO-TP message ceiling.
        std::uint16_t room = static_cast<std::uint16_t>((cap - hdr) / diag::LogfsEntryLen);
        if (room > diag::LogfsMaxListEntries) room = diag::LogfsMaxListEntries;

        if (!be_.list_begin(cursor)) return nack(out, cap, req.opcode, diag::NackFsError);

        std::uint8_t  count = 0;
        std::uint16_t w     = hdr;
        Entry         e;
        while (count < room && be_.list_next(e)) {
            diag::put_u16(out + w, e.index);
            diag::put_u32(out + w + 2u, e.size);
            diag::put_u32(out + w + 6u, e.mtime);
            for (std::uint8_t c = 0; c < diag::LogfsNameLen; ++c) {
                out[w + 10u + c] = static_cast<std::uint8_t>(e.name[c]);
            }
            w = static_cast<std::uint16_t>(w + diag::LogfsEntryLen);
            ++count;
        }

        // A full batch means there may be more; a short one is the end. This
        // costs one extra empty LIST when the count is an exact multiple of
        // the batch size, which is cheaper than making the backend look ahead.
        std::uint16_t next = CursorEnd;
        if (count == room) {
            const std::uint32_t advanced = static_cast<std::uint32_t>(cursor) + count;
            // Can't address past the sentinel, so treat that as the end rather
            // than wrapping the cursor back to the start of the card.
            next = (advanced >= CursorEnd) ? CursorEnd : static_cast<std::uint16_t>(advanced);
        }

        out[0] = static_cast<std::uint8_t>(diag::MsgType::Ack);
        out[1] = req.opcode;
        diag::put_u16(out + 2, next);
        out[4] = count;
        return w;
    }

    std::uint16_t do_open(const diag::Request& req, std::uint8_t* out,
                          std::uint16_t cap) noexcept {
        if (req.arg_len < 2u) return nack(out, cap, req.opcode, diag::NackOutOfBounds);
        const std::uint16_t index = diag::get_u16(req.args);

        // One file at a time. A second OPEN supersedes the first rather than
        // erroring -- a client that lost its ACK can simply retry.
        if (open_handle_ != NoHandle) be_.close(open_index_);
        open_handle_ = NoHandle;

        std::uint32_t size = 0;
        std::uint32_t crc  = 0;
        if (!be_.open(index, size, crc)) {
            return nack(out, cap, req.opcode, diag::NackFileNotFound);
        }

        open_index_  = index;
        open_handle_ = next_handle();

        // crc32 == 0 means "not available, ask LOGFS_CRC" -- NOT a literal zero
        // CRC (#452). Sealing writes a sidecar, so it is populated for anything
        // this firmware wrote; older cards have none and OPEN must not stall to
        // stream 4 MiB computing one.
        std::uint8_t body[10];
        diag::put_u16(body, open_handle_);
        diag::put_u32(body + 2, size);
        diag::put_u32(body + 6, crc);
        return diag::build_ack(out, cap, req.opcode, body, sizeof body);
    }

    std::uint16_t do_read(const diag::Request& req, std::uint8_t* out,
                          std::uint16_t cap) noexcept {
        if (req.arg_len < 8u) return nack(out, cap, req.opcode, diag::NackOutOfBounds);
        const std::uint16_t h   = diag::get_u16(req.args);
        const std::uint32_t off = diag::get_u32(req.args + 2);
        std::uint16_t       len = diag::get_u16(req.args + 6);

        if (!resolve(h)) return nack(out, cap, req.opcode, diag::NackBadHandle);
        if (len > diag::LogfsMaxRead) len = diag::LogfsMaxRead;   // clamp, don't reject
        if (len == 0u) return diag::build_ack(out, cap, req.opcode, nullptr, 0u);

        // Read straight into the reply, past the [type][opcode] header, so a
        // 512-byte read needs no second buffer.
        if (cap < static_cast<std::uint16_t>(2u + len)) {
            return nack(out, cap, req.opcode, diag::NackOutOfBounds);
        }
        const int n = be_.read(open_index_, off, out + 2, len);
        if (n < 0) return nack(out, cap, req.opcode, diag::NackReadError);

        out[0] = static_cast<std::uint8_t>(diag::MsgType::Ack);
        out[1] = req.opcode;
        return static_cast<std::uint16_t>(2 + n);
    }

    std::uint16_t do_crc(const diag::Request& req, std::uint8_t* out,
                         std::uint16_t cap) noexcept {
        if (req.arg_len < 2u) return nack(out, cap, req.opcode, diag::NackOutOfBounds);
        if (!resolve(diag::get_u16(req.args))) {
            return nack(out, cap, req.opcode, diag::NackBadHandle);
        }
        std::uint32_t crc = 0;
        if (!be_.crc32(open_index_, crc)) return nack(out, cap, req.opcode, diag::NackReadError);

        std::uint8_t body[4];
        diag::put_u32(body, crc);
        return diag::build_ack(out, cap, req.opcode, body, sizeof body);
    }

    std::uint16_t do_close(const diag::Request& req, std::uint8_t* out,
                           std::uint16_t cap) noexcept {
        if (req.arg_len < 2u) return nack(out, cap, req.opcode, diag::NackOutOfBounds);
        if (!resolve(diag::get_u16(req.args))) {
            return nack(out, cap, req.opcode, diag::NackBadHandle);
        }
        be_.close(open_index_);
        open_handle_ = NoHandle;
        return diag::build_ack(out, cap, req.opcode, nullptr, 0u);
    }

    // Seal the active log and report its index. Takes no arguments: there is
    // only ever one active file, and naming it would invite a host racing a
    // rotation. Refused when nothing has been written, so an eager operator
    // cannot litter the card with header-only files.
    std::uint16_t do_finalize(const diag::Request& req, std::uint8_t* out,
                              std::uint16_t cap) noexcept {
        // Any open read handle points into the file set we are about to change;
        // drop it rather than leave the host holding something ambiguous.
        release();

        std::uint16_t sealed = 0;
        if (!be_.finalize(sealed)) {
            // Nothing to seal: no active file, or no rows in it yet.
            return nack(out, cap, req.opcode, diag::NackFileNotFound);
        }
        std::uint8_t body[2];
        diag::put_u16(body, sealed);
        return diag::build_ack(out, cap, req.opcode, body, sizeof body);
    }

    // Monotonic, skipping 0. Wrapping is harmless: only one handle is ever
    // live, and a stale one is rejected by value comparison.
    [[nodiscard]] std::uint16_t next_handle() noexcept {
        ++handle_seq_;
        if (handle_seq_ == NoHandle) handle_seq_ = 1u;
        return handle_seq_;
    }

    Backend&      be_;
    std::uint16_t open_handle_ = NoHandle;
    std::uint16_t open_index_  = 0;
    std::uint16_t handle_seq_  = 0;
};

}  // namespace ams::logfs
