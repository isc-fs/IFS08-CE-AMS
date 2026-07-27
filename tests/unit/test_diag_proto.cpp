// SPDX-License-Identifier: proprietary
//
// Pure-logic tests for ams::diag -- the command layer above the ISO-TP
// transport (#406 / #439 LOGFS).
//
// Two things are worth pinning here, and both are about the HOST, not us:
//
//  1. The numbers. msg_type APP_CTRL = 0x06 (not 0x00 -- we deliberately keep
//     off the bootloader's opcode namespace), and the NACK codes, which have
//     already been mis-numbered once: 0x08 was proposed for BAD_HANDLE in #439
//     but 0x08 is BL_NACK_BUSY. A literal assertion is what stops that from
//     silently regressing.
//  2. Byte order. The wire is little-endian; a host-endianness-dependent
//     encoder would pass any round-trip test but break the client.

#include "diag_proto.hpp"

#include "unity.h"

#include <cstdint>

namespace {

using namespace ams;

constexpr std::uint8_t kPeer  = 2u;
constexpr std::uint8_t kOther = 3u;

}  // namespace

// --- wire constants ---------------------------------------------------------

// The whole point of the APP_CTRL decision: LOGFS must NOT ride on CMD (0x00).
extern "C" void test_diag_msgtype_app_ctrl_is_0x06(void) {
    TEST_ASSERT_EQUAL_HEX8(0x06, static_cast<std::uint8_t>(diag::MsgType::AppCtrl));
    TEST_ASSERT_EQUAL_HEX8(0x00, static_cast<std::uint8_t>(diag::MsgType::Cmd));
    TEST_ASSERT_EQUAL_HEX8(0x01, static_cast<std::uint8_t>(diag::MsgType::Ack));
    TEST_ASSERT_EQUAL_HEX8(0x02, static_cast<std::uint8_t>(diag::MsgType::Nack));
}

extern "C" void test_diag_logfs_opcodes(void) {
    TEST_ASSERT_EQUAL_HEX8(0x21, diag::OpLogfsList);
    TEST_ASSERT_EQUAL_HEX8(0x22, diag::OpLogfsOpen);
    TEST_ASSERT_EQUAL_HEX8(0x23, diag::OpLogfsRead);
    TEST_ASSERT_EQUAL_HEX8(0x24, diag::OpLogfsCrc);
    TEST_ASSERT_EQUAL_HEX8(0x25, diag::OpLogfsClose);
}

// BAD_HANDLE must NOT be 0x08 -- that is BL_NACK_BUSY. This test exists
// because that exact collision was published and had to be corrected.
extern "C" void test_diag_nack_codes_do_not_collide_with_bootloader(void) {
    TEST_ASSERT_EQUAL_HEX8(0x11, diag::NackBadHandle);
    TEST_ASSERT_EQUAL_HEX8(0x08, diag::NackBusy);
    TEST_ASSERT_NOT_EQUAL(diag::NackBusy, diag::NackBadHandle);

    TEST_ASSERT_EQUAL_HEX8(0x04, diag::NackFileNotFound);
    TEST_ASSERT_EQUAL_HEX8(0x12, diag::NackNoSdCard);
    TEST_ASSERT_EQUAL_HEX8(0x13, diag::NackFsError);
    TEST_ASSERT_EQUAL_HEX8(0x14, diag::NackReadError);
    TEST_ASSERT_EQUAL_HEX8(0xFE, diag::NackUnsupported);
}

// A LIST reply is parsed by stride on the host, so the entry size is wire.
extern "C" void test_diag_logfs_entry_is_22_bytes(void) {
    TEST_ASSERT_EQUAL_UINT8(22, diag::LogfsEntryLen);
    TEST_ASSERT_EQUAL_UINT8(12, diag::LogfsNameLen);
    TEST_ASSERT_EQUAL_UINT16(512, diag::LogfsMaxRead);
}

// Every LIST reply must fit one ISO-TP message, header included.
extern "C" void test_diag_list_batch_fits_one_isotp_message(void) {
    const std::uint32_t reply =
        2u + 4u + static_cast<std::uint32_t>(diag::LogfsMaxListEntries) * diag::LogfsEntryLen;
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(isotp::MaxMsg, reply);
    TEST_ASSERT_GREATER_THAN_UINT8(0, diag::LogfsMaxListEntries);
}

// --- little-endian helpers --------------------------------------------------

extern "C" void test_diag_put_u16_is_little_endian(void) {
    std::uint8_t b[2] = {0xFFu, 0xFFu};
    diag::put_u16(b, 0x1234u);
    TEST_ASSERT_EQUAL_HEX8(0x34, b[0]);
    TEST_ASSERT_EQUAL_HEX8(0x12, b[1]);
    TEST_ASSERT_EQUAL_HEX16(0x1234u, diag::get_u16(b));
}

extern "C" void test_diag_put_u32_is_little_endian(void) {
    std::uint8_t b[4] = {0, 0, 0, 0};
    diag::put_u32(b, 0xDEADBEEFu);
    TEST_ASSERT_EQUAL_HEX8(0xEF, b[0]);
    TEST_ASSERT_EQUAL_HEX8(0xBE, b[1]);
    TEST_ASSERT_EQUAL_HEX8(0xAD, b[2]);
    TEST_ASSERT_EQUAL_HEX8(0xDE, b[3]);
    TEST_ASSERT_EQUAL_HEX32(0xDEADBEEFu, diag::get_u32(b));
}

extern "C" void test_diag_le_helpers_handle_extremes(void) {
    std::uint8_t b[4] = {0};
    diag::put_u32(b, 0xFFFFFFFFu);
    TEST_ASSERT_EQUAL_HEX32(0xFFFFFFFFu, diag::get_u32(b));
    diag::put_u32(b, 0u);
    TEST_ASSERT_EQUAL_HEX32(0u, diag::get_u32(b));
}

// --- request parsing --------------------------------------------------------

extern "C" void test_diag_parse_request_extracts_type_opcode_args(void) {
    const std::uint8_t msg[] = {0x06u, 0x23u, 0x01u, 0x02u, 0x03u};
    diag::Request req;
    TEST_ASSERT_TRUE(diag::parse_request(msg, sizeof(msg), req));
    TEST_ASSERT_EQUAL_HEX8(0x06, static_cast<std::uint8_t>(req.type));
    TEST_ASSERT_TRUE(req.is_app_ctrl());
    TEST_ASSERT_EQUAL_HEX8(diag::OpLogfsRead, req.opcode);
    TEST_ASSERT_EQUAL_UINT16(3u, req.arg_len);
    TEST_ASSERT_EQUAL_HEX8(0x01, req.args[0]);
    TEST_ASSERT_EQUAL_HEX8(0x03, req.args[2]);
}

extern "C" void test_diag_parse_request_no_args(void) {
    const std::uint8_t msg[] = {0x06u, 0x21u};
    diag::Request req;
    TEST_ASSERT_TRUE(diag::parse_request(msg, sizeof(msg), req));
    TEST_ASSERT_EQUAL_UINT16(0u, req.arg_len);
    TEST_ASSERT_NULL(req.args);
}

extern "C" void test_diag_parse_request_rejects_runt(void) {
    const std::uint8_t msg[] = {0x06u};
    diag::Request req;
    TEST_ASSERT_FALSE(diag::parse_request(msg, 1u, req));
    TEST_ASSERT_FALSE(diag::parse_request(msg, 0u, req));
    TEST_ASSERT_FALSE(diag::parse_request(nullptr, 8u, req));
}

// A CMD-typed message must be distinguishable, so the dispatcher can refuse to
// serve LOGFS to a host still speaking the old framing.
extern "C" void test_diag_parse_request_flags_non_app_ctrl(void) {
    const std::uint8_t msg[] = {0x00u, 0x21u};
    diag::Request req;
    TEST_ASSERT_TRUE(diag::parse_request(msg, sizeof(msg), req));
    TEST_ASSERT_FALSE(req.is_app_ctrl());
}

// --- response building ------------------------------------------------------

extern "C" void test_diag_build_ack_frames_type_and_opcode(void) {
    std::uint8_t out[16] = {0};
    const std::uint8_t payload[] = {0xAAu, 0xBBu};
    const std::uint16_t n = diag::build_ack(out, sizeof(out), diag::OpLogfsOpen, payload, 2u);
    TEST_ASSERT_EQUAL_UINT16(4u, n);
    TEST_ASSERT_EQUAL_HEX8(0x01, out[0]);              // Ack
    TEST_ASSERT_EQUAL_HEX8(diag::OpLogfsOpen, out[1]); // echoes the opcode
    TEST_ASSERT_EQUAL_HEX8(0xAA, out[2]);
    TEST_ASSERT_EQUAL_HEX8(0xBB, out[3]);
}

extern "C" void test_diag_build_ack_empty_payload(void) {
    std::uint8_t out[4] = {0};
    const std::uint16_t n = diag::build_ack(out, sizeof(out), diag::OpLogfsClose, nullptr, 0u);
    TEST_ASSERT_EQUAL_UINT16(2u, n);
    TEST_ASSERT_EQUAL_HEX8(0x01, out[0]);
    TEST_ASSERT_EQUAL_HEX8(diag::OpLogfsClose, out[1]);
}

extern "C" void test_diag_build_ack_refuses_overflow(void) {
    std::uint8_t out[4] = {0};
    const std::uint8_t payload[8] = {0};
    TEST_ASSERT_EQUAL_UINT16(0u, diag::build_ack(out, sizeof(out), 0x21u, payload, 8u));
}

extern "C" void test_diag_build_nack_is_three_bytes(void) {
    std::uint8_t out[8] = {0};
    const std::uint16_t n =
        diag::build_nack(out, sizeof(out), diag::OpLogfsRead, diag::NackBadHandle);
    TEST_ASSERT_EQUAL_UINT16(3u, n);
    TEST_ASSERT_EQUAL_HEX8(0x02, out[0]);
    TEST_ASSERT_EQUAL_HEX8(diag::OpLogfsRead, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x11, out[2]);
    TEST_ASSERT_EQUAL_UINT16(0u, diag::build_nack(out, 2u, 0x21u, 0x01u));
}

// --- session ----------------------------------------------------------------

extern "C" void test_diag_session_starts_closed(void) {
    diag::Session s;
    TEST_ASSERT_FALSE(s.is_open());
    TEST_ASSERT_FALSE(s.touch(kPeer, 100u));
}

extern "C" void test_diag_session_connect_disconnect(void) {
    diag::Session s;
    s.connect(kPeer, 1000u);
    TEST_ASSERT_TRUE(s.is_open());
    TEST_ASSERT_EQUAL_UINT8(kPeer, s.peer());
    TEST_ASSERT_TRUE(s.touch(kPeer, 1100u));
    s.disconnect();
    TEST_ASSERT_FALSE(s.is_open());
}

// A second host must not be able to drive a session it did not open.
extern "C" void test_diag_session_rejects_other_peer(void) {
    diag::Session s;
    s.connect(kPeer, 1000u);
    TEST_ASSERT_FALSE(s.touch(kOther, 1100u));
    TEST_ASSERT_TRUE(s.is_open());  // and it does not disturb the real one
    TEST_ASSERT_TRUE(s.touch(kPeer, 1200u));
}

extern "C" void test_diag_session_expires_when_host_walks_away(void) {
    diag::Session s;
    s.connect(kPeer, 1000u);
    TEST_ASSERT_FALSE(s.tick(1000u + diag::Session::IdleTimeoutMs - 1u));
    TEST_ASSERT_TRUE(s.is_open());
    TEST_ASSERT_TRUE(s.tick(1000u + diag::Session::IdleTimeoutMs));
    TEST_ASSERT_FALSE(s.is_open());
    TEST_ASSERT_FALSE(s.tick(9999999u));  // expiry reported once, not repeatedly
}

// Traffic keeps a long transfer alive well past the idle timeout.
extern "C" void test_diag_session_traffic_refreshes_idle_timer(void) {
    diag::Session s;
    s.connect(kPeer, 0u);
    for (std::uint32_t t = 5000u; t <= 100000u; t += 5000u) {
        TEST_ASSERT_TRUE(s.touch(kPeer, t));
        TEST_ASSERT_FALSE(s.tick(t));
    }
    TEST_ASSERT_TRUE(s.is_open());
}
