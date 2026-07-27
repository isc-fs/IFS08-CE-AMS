// SPDX-License-Identifier: proprietary
//
// Pure-logic tests for ams::isotp -- the application-side ISO-TP transport
// that carries the #406 LOGFS command protocol. No HAL, no RTOS.
//
// The wire format is the bootloader's (stm32-can-bootloader bl_isotp.h), so
// these tests pin the PCI encoding explicitly rather than only round-tripping
// our own code against itself -- a self-consistent but wrong encoding would
// still break the host, which is the failure mode that actually matters.

#include "isotp.hpp"

#include "unity.h"

#include <cstdint>

namespace {

using namespace ams;

constexpr std::uint8_t kPeer = 2u;

// Push one full message through Segmenter -> Reassembler. Returns true iff it
// reassembled byte-identically.
bool round_trip(const std::uint8_t* msg, std::uint16_t len) {
    isotp::Segmenter seg;
    isotp::Reassembler rx;
    if (!seg.begin(msg, len)) return false;

    std::uint8_t frame[isotp::FrameLen];
    isotp::RxStatus st = isotp::RxStatus::Ok;
    while (seg.next(frame)) {
        st = rx.feed(frame, isotp::FrameLen, kPeer, 1000u);
        if (st == isotp::RxStatus::MsgComplete) break;
        if (st != isotp::RxStatus::Ok) return false;
    }
    if (st != isotp::RxStatus::MsgComplete) return false;
    if (rx.size() != len) return false;
    for (std::uint16_t i = 0; i < len; ++i) {
        if (rx.data()[i] != msg[i]) return false;
    }
    return true;
}

void fill_pattern(std::uint8_t* buf, std::uint16_t len) {
    for (std::uint16_t i = 0; i < len; ++i) {
        buf[i] = static_cast<std::uint8_t>((i * 31u + 7u) & 0xFFu);
    }
}

}  // namespace

// setUp/tearDown are defined once for the whole binary in test_bms_service.cpp.

// ---------------------------------------------------------------------------
// Round trips across the interesting size boundaries.
// ---------------------------------------------------------------------------
extern "C" void test_isotp_round_trip_single_frame(void) {
    std::uint8_t msg[7];
    for (std::uint16_t n = 1; n <= isotp::SfMaxLen; ++n) {
        fill_pattern(msg, n);
        TEST_ASSERT_TRUE_MESSAGE(round_trip(msg, n), "SF round trip failed");
    }
}

extern "C" void test_isotp_round_trip_first_multiframe(void) {
    // 8 bytes is the smallest message that cannot be a single frame.
    std::uint8_t msg[8];
    fill_pattern(msg, sizeof msg);
    TEST_ASSERT_TRUE(round_trip(msg, sizeof msg));
}

// >15 consecutive frames, so the CF sequence counter wraps 15 -> 0.
extern "C" void test_isotp_round_trip_sequence_wrap(void) {
    std::uint8_t msg[200];
    fill_pattern(msg, sizeof msg);
    TEST_ASSERT_TRUE(round_trip(msg, sizeof msg));
}

extern "C" void test_isotp_round_trip_max_message(void) {
    static std::uint8_t msg[isotp::MaxMsg];
    fill_pattern(msg, isotp::MaxMsg);
    TEST_ASSERT_TRUE(round_trip(msg, isotp::MaxMsg));
}

// ---------------------------------------------------------------------------
// Encoding is pinned to the bootloader's wire format, not just self-consistent.
// ---------------------------------------------------------------------------
extern "C" void test_isotp_segmenter_pins_wire_encoding(void) {
    // Single frame: PCI = 0x0L, payload follows in bytes 1..L.
    const std::uint8_t sf[3] = { 0xAA, 0xBB, 0xCC };
    isotp::Segmenter seg;
    std::uint8_t f[isotp::FrameLen];
    TEST_ASSERT_TRUE(seg.begin(sf, sizeof sf));
    TEST_ASSERT_TRUE(seg.next(f));
    TEST_ASSERT_EQUAL_HEX8(0x03, f[0]);
    TEST_ASSERT_EQUAL_HEX8(0xAA, f[1]);
    TEST_ASSERT_EQUAL_HEX8(0xCC, f[3]);
    TEST_ASSERT_FALSE(seg.next(f));          // SF is the whole message

    // Multi-frame: FF carries the 12-bit length, then CFs numbered from 1.
    static std::uint8_t big[300];
    fill_pattern(big, sizeof big);
    TEST_ASSERT_TRUE(seg.begin(big, sizeof big));
    TEST_ASSERT_TRUE(seg.next(f));
    TEST_ASSERT_EQUAL_HEX8(0x11, f[0]);      // FF | high nibble of 300 (0x12C)
    TEST_ASSERT_EQUAL_HEX8(0x2C, f[1]);      // low byte of 300
    TEST_ASSERT_EQUAL_HEX8(big[0], f[2]);    // FF data starts at byte 2
    TEST_ASSERT_TRUE(seg.next(f));
    TEST_ASSERT_EQUAL_HEX8(0x21, f[0]);      // first CF is sequence 1
    TEST_ASSERT_EQUAL_HEX8(big[6], f[1]);    // continues after the FF's 6 bytes
    TEST_ASSERT_TRUE(seg.next(f));
    TEST_ASSERT_EQUAL_HEX8(0x22, f[0]);      // then 2
}

extern "C" void test_isotp_flow_control_frame_format(void) {
    std::uint8_t fc[isotp::FrameLen];
    isotp::build_fc_cts(fc);
    TEST_ASSERT_EQUAL_HEX8(0x30, fc[0]);     // FC | CTS
    TEST_ASSERT_EQUAL_HEX8(0x00, fc[1]);     // BlockSize = 0
    TEST_ASSERT_EQUAL_HEX8(0x00, fc[2]);     // STmin = 0
}

// An FF must leave the receiver owing a flow-control frame.
extern "C" void test_isotp_ff_raises_flow_control_pending(void) {
    isotp::Reassembler rx;
    std::uint8_t ff[isotp::FrameLen] = { 0x10, 0x20, 1, 2, 3, 4, 5, 6 };  // 32 bytes
    TEST_ASSERT_FALSE(rx.fc_pending());
    TEST_ASSERT_EQUAL(static_cast<int>(isotp::RxStatus::Ok),
                      static_cast<int>(rx.feed(ff, isotp::FrameLen, kPeer, 0u)));
    TEST_ASSERT_TRUE(rx.fc_pending());
    rx.clear_fc_pending();
    TEST_ASSERT_FALSE(rx.fc_pending());
}

// ---------------------------------------------------------------------------
// Error paths -- malformed input must be rejected, not silently accepted.
// ---------------------------------------------------------------------------
extern "C" void test_isotp_rejects_bad_single_frame_length(void) {
    isotp::Reassembler rx;
    std::uint8_t zero_len[isotp::FrameLen] = { 0x00, 1, 2, 3, 4, 5, 6, 7 };
    TEST_ASSERT_EQUAL(static_cast<int>(isotp::RxStatus::ErrBadPci),
                      static_cast<int>(rx.feed(zero_len, isotp::FrameLen, kPeer, 0u)));

    // SF claiming 7 bytes but the frame only carries 3.
    std::uint8_t truncated[4] = { 0x07, 1, 2, 3 };
    TEST_ASSERT_EQUAL(static_cast<int>(isotp::RxStatus::ErrBadPci),
                      static_cast<int>(rx.feed(truncated, 4u, kPeer, 0u)));
}

// A message that fits in an SF must not be sent as an FF.
extern "C" void test_isotp_rejects_first_frame_that_should_be_single(void) {
    isotp::Reassembler rx;
    std::uint8_t ff[isotp::FrameLen] = { 0x10, 0x05, 1, 2, 3, 4, 5, 6 };
    TEST_ASSERT_EQUAL(static_cast<int>(isotp::RxStatus::ErrBadPci),
                      static_cast<int>(rx.feed(ff, isotp::FrameLen, kPeer, 0u)));
}

extern "C" void test_isotp_rejects_oversized_message(void) {
    isotp::Reassembler rx;
    // 0xFFF = 4095 > MaxMsg (1024).
    std::uint8_t ff[isotp::FrameLen] = { 0x1F, 0xFF, 1, 2, 3, 4, 5, 6 };
    TEST_ASSERT_EQUAL(static_cast<int>(isotp::RxStatus::ErrOverflow),
                      static_cast<int>(rx.feed(ff, isotp::FrameLen, kPeer, 0u)));
}

extern "C" void test_isotp_rejects_consecutive_without_first(void) {
    isotp::Reassembler rx;
    std::uint8_t cf[isotp::FrameLen] = { 0x21, 1, 2, 3, 4, 5, 6, 7 };
    TEST_ASSERT_EQUAL(static_cast<int>(isotp::RxStatus::ErrNoFf),
                      static_cast<int>(rx.feed(cf, isotp::FrameLen, kPeer, 0u)));
}

extern "C" void test_isotp_rejects_out_of_order_sequence(void) {
    isotp::Reassembler rx;
    std::uint8_t ff[isotp::FrameLen] = { 0x10, 0x20, 1, 2, 3, 4, 5, 6 };
    (void)rx.feed(ff, isotp::FrameLen, kPeer, 0u);
    // Expected sequence 1; send 2.
    std::uint8_t cf[isotp::FrameLen] = { 0x22, 1, 2, 3, 4, 5, 6, 7 };
    TEST_ASSERT_EQUAL(static_cast<int>(isotp::RxStatus::ErrBadSeq),
                      static_cast<int>(rx.feed(cf, isotp::FrameLen, kPeer, 0u)));
    TEST_ASSERT_EQUAL_UINT8(1u, rx.last_expected());
    TEST_ASSERT_EQUAL_UINT8(2u, rx.last_observed());
}

// A reassembly belongs to the peer that opened it.
extern "C" void test_isotp_rejects_consecutive_from_other_peer(void) {
    isotp::Reassembler rx;
    std::uint8_t ff[isotp::FrameLen] = { 0x10, 0x20, 1, 2, 3, 4, 5, 6 };
    (void)rx.feed(ff, isotp::FrameLen, kPeer, 0u);
    std::uint8_t cf[isotp::FrameLen] = { 0x21, 1, 2, 3, 4, 5, 6, 7 };
    TEST_ASSERT_EQUAL(static_cast<int>(isotp::RxStatus::ErrNoFf),
                      static_cast<int>(rx.feed(cf, isotp::FrameLen,
                                               static_cast<std::uint8_t>(kPeer + 1u), 0u)));
}

// ---------------------------------------------------------------------------
// Timeout: a stalled reassembly is abandoned, and the receiver recovers.
// ---------------------------------------------------------------------------
extern "C" void test_isotp_reassembly_times_out(void) {
    isotp::Reassembler rx;
    std::uint8_t ff[isotp::FrameLen] = { 0x10, 0x20, 1, 2, 3, 4, 5, 6 };
    (void)rx.feed(ff, isotp::FrameLen, kPeer, 1000u);

    // Still inside the window.
    TEST_ASSERT_EQUAL(static_cast<int>(isotp::RxStatus::Ok),
                      static_cast<int>(rx.tick(1000u + isotp::TimeoutMs - 1u)));
    // At the deadline it aborts.
    TEST_ASSERT_EQUAL(static_cast<int>(isotp::RxStatus::ErrTimeout),
                      static_cast<int>(rx.tick(1000u + isotp::TimeoutMs)));
    // And having aborted, a stray CF is no longer accepted.
    std::uint8_t cf[isotp::FrameLen] = { 0x21, 1, 2, 3, 4, 5, 6, 7 };
    TEST_ASSERT_EQUAL(static_cast<int>(isotp::RxStatus::ErrNoFf),
                      static_cast<int>(rx.feed(cf, isotp::FrameLen, kPeer, 2500u)));
}

// An idle receiver never times out.
extern "C" void test_isotp_idle_never_times_out(void) {
    isotp::Reassembler rx;
    TEST_ASSERT_EQUAL(static_cast<int>(isotp::RxStatus::Ok),
                      static_cast<int>(rx.tick(0xFFFFFFFFu)));
}

// Received flow-control frames are ignored, not treated as an error.
extern "C" void test_isotp_ignores_received_flow_control(void) {
    isotp::Reassembler rx;
    std::uint8_t fc[isotp::FrameLen] = { 0x30, 0, 0, 0, 0, 0, 0, 0 };
    TEST_ASSERT_EQUAL(static_cast<int>(isotp::RxStatus::Ok),
                      static_cast<int>(rx.feed(fc, isotp::FrameLen, kPeer, 0u)));
}

// After an error the receiver must accept a fresh message.
extern "C" void test_isotp_recovers_after_error(void) {
    isotp::Reassembler rx;
    std::uint8_t cf[isotp::FrameLen] = { 0x21, 1, 2, 3, 4, 5, 6, 7 };
    TEST_ASSERT_EQUAL(static_cast<int>(isotp::RxStatus::ErrNoFf),
                      static_cast<int>(rx.feed(cf, isotp::FrameLen, kPeer, 0u)));
    std::uint8_t sf[isotp::FrameLen] = { 0x02, 0xDE, 0xAD, 0, 0, 0, 0, 0 };
    TEST_ASSERT_EQUAL(static_cast<int>(isotp::RxStatus::MsgComplete),
                      static_cast<int>(rx.feed(sf, isotp::FrameLen, kPeer, 0u)));
    TEST_ASSERT_EQUAL_UINT16(2u, rx.size());
    TEST_ASSERT_EQUAL_HEX8(0xDE, rx.data()[0]);
    TEST_ASSERT_EQUAL_HEX8(0xAD, rx.data()[1]);
}

// Segmenter refuses inputs it cannot represent.
extern "C" void test_isotp_segmenter_rejects_invalid_input(void) {
    isotp::Segmenter seg;
    std::uint8_t one = 0x5A;
    TEST_ASSERT_FALSE(seg.begin(nullptr, 4u));
    TEST_ASSERT_FALSE(seg.begin(&one, 0u));
    TEST_ASSERT_FALSE(seg.begin(&one, isotp::MaxMsg + 1u));
    TEST_ASSERT_TRUE(seg.begin(&one, 1u));
}
