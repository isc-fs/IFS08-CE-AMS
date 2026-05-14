// SPDX-License-Identifier: proprietary
//
// Tests for ams::Bootloader::matches_trigger -- the pure decoder
// half of the jump-to-bootloader path. The actual request_reboot()
// path resets the chip, so we test the decision logic only.

#include "ams_config.hpp"
#include "bootloader.hpp"
#include "can_frame.hpp"

#include "unity.h"

#include <cstdint>
#include <cstring>

namespace {

ams::CanFrame make_trigger_frame() {
    ams::CanFrame f = {};
    f.id  = ams::config::kBlBootReqCanId;
    f.dlc = ams::config::kBlBootReqDlc;
    f.bus = static_cast<std::uint8_t>(ams::CanBus::Acu);
    std::memcpy(f.data, ams::config::kBlBootReqPayload,
                ams::config::kBlBootReqDlc);
    return f;
}

}  // namespace

extern "C" void test_bootloader_trigger_exact_match(void) {
    auto f = make_trigger_frame();
    TEST_ASSERT_TRUE(ams::Bootloader::matches_trigger(f));
}

extern "C" void test_bootloader_trigger_wrong_bus(void) {
    auto f = make_trigger_frame();
    f.bus = static_cast<std::uint8_t>(ams::CanBus::Bms);
    TEST_ASSERT_FALSE(ams::Bootloader::matches_trigger(f));
}

extern "C" void test_bootloader_trigger_wrong_id(void) {
    auto f = make_trigger_frame();
    f.id  = ams::config::kBlBootReqCanId + 1;
    TEST_ASSERT_FALSE(ams::Bootloader::matches_trigger(f));

    f.id  = 0;
    TEST_ASSERT_FALSE(ams::Bootloader::matches_trigger(f));

    f.id  = 0x12D;  // a real BMS-response ID
    TEST_ASSERT_FALSE(ams::Bootloader::matches_trigger(f));
}

extern "C" void test_bootloader_trigger_wrong_dlc(void) {
    auto f = make_trigger_frame();

    f.dlc = 3;
    TEST_ASSERT_FALSE(ams::Bootloader::matches_trigger(f));

    f.dlc = 5;
    TEST_ASSERT_FALSE(ams::Bootloader::matches_trigger(f));

    f.dlc = 8;
    TEST_ASSERT_FALSE(ams::Bootloader::matches_trigger(f));
}

extern "C" void test_bootloader_trigger_each_magic_byte_flipped(void) {
    // Each byte of the 4-byte magic, flipped one at a time, must
    // make the frame not match. Catches single-bit corruption + any
    // future change to the magic value that forgets a byte.
    for (std::uint8_t i = 0; i < ams::config::kBlBootReqDlc; ++i) {
        auto f = make_trigger_frame();
        f.data[i] ^= 0xFF;
        TEST_ASSERT_FALSE(ams::Bootloader::matches_trigger(f));
    }
}

extern "C" void test_bootloader_trigger_trailing_bytes_ignored(void) {
    // Bytes 4..7 are outside the DLC and must not affect matching.
    // (A 4-byte frame on the wire only ever carries 4 bytes; the
    // remaining slots in the CanFrame struct are uninitialized
    // memory in practice.)
    auto f = make_trigger_frame();
    f.data[4] = 0xCC;
    f.data[5] = 0xDD;
    f.data[6] = 0xEE;
    f.data[7] = 0xFF;
    TEST_ASSERT_TRUE(ams::Bootloader::matches_trigger(f));
}
