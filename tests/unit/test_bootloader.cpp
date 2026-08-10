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
    f.id  = ams::config::BlBootReqCanId;
    f.dlc = ams::config::BlBootReqDlc;
    f.bus = static_cast<std::uint8_t>(ams::CanBus::Acu);
    std::memcpy(f.data, ams::config::BlBootReqPayload,
                ams::config::BlBootReqDlc);
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
    f.id  = ams::config::BlBootReqCanId + 1;
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
    for (std::uint8_t i = 0; i < ams::config::BlBootReqDlc; ++i) {
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

// The reboot trigger opens all relays and resets. Honouring it in an energised
// state means opening AIR+ under inverter load -- how contactors weld -- on a
// 4-byte frame anyone on the accumulator bus can send.
extern "C" void test_bootloader_reboot_only_when_contactors_open(void) {
    using ams::Bootloader;
    using S = ams::fsm::State;

    // Contactors already open -> the reboot costs nothing new.
    TEST_ASSERT_TRUE(Bootloader::reboot_allowed_in(S::Start));

    // Error is the case that matters most, not a grudging exception: ErrorLatch
    // is sticky across resets, so a faulted car boots back INTO Error. Refusing
    // here would make reflashing a faulted AMS impossible without first
    // clearing the latch -- exactly when you most want to reflash it.
    TEST_ASSERT_TRUE(Bootloader::reboot_allowed_in(S::Error));

    // Every energised state refuses.
    TEST_ASSERT_FALSE(Bootloader::reboot_allowed_in(S::Precharge));
    TEST_ASSERT_FALSE(Bootloader::reboot_allowed_in(S::Transition));
    TEST_ASSERT_FALSE(Bootloader::reboot_allowed_in(S::Run));
    TEST_ASSERT_FALSE(Bootloader::reboot_allowed_in(S::Charge));
}

// The gate is orthogonal to payload matching: a valid trigger frame stays valid
// in Run, it just must not be acted on. Keeping these separate means a future
// change to one cannot silently disable the other.
extern "C" void test_bootloader_trigger_still_matches_while_energised(void) {
    ams::CanFrame f{};
    f.bus = static_cast<std::uint8_t>(ams::CanBus::Acu);
    f.id  = ams::config::BlBootReqCanId;
    f.dlc = ams::config::BlBootReqDlc;
    for (std::uint8_t i = 0; i < ams::config::BlBootReqDlc; ++i) {
        f.data[i] = ams::config::BlBootReqPayload[i];
    }
    TEST_ASSERT_TRUE(ams::Bootloader::matches_trigger(f));
    TEST_ASSERT_FALSE(ams::Bootloader::reboot_allowed_in(ams::fsm::State::Run));
}
