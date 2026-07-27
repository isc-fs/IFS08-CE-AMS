// SPDX-License-Identifier: proprietary
//
// Tests for ams::crc -- the log-file CRC-32 (#406 / #439).
//
// Every expected value below was produced by Python's zlib.crc32, which is
// what the host tools verify with. That is the entire point of these tests:
// an internally-consistent CRC that disagrees with zlib would make every
// extracted log look corrupt, and the argument would land on the firmware.

#include "crc32.hpp"

#include "unity.h"

#include <cstdint>
#include <cstring>

namespace {

using namespace ams;

std::uint32_t crc_str(const char* s) {
    return crc::compute(s, std::strlen(s));
}

}  // namespace

// --- known vectors (zlib.crc32) --------------------------------------------

extern "C" void test_crc32_check_vector(void) {
    // The canonical CRC-32 check value.
    TEST_ASSERT_EQUAL_HEX32(0xCBF43926u, crc_str("123456789"));
}

extern "C" void test_crc32_empty_is_zero(void) {
    TEST_ASSERT_EQUAL_HEX32(0x00000000u, crc_str(""));
    TEST_ASSERT_EQUAL_HEX32(0x00000000u, crc::compute(nullptr, 0));
}

extern "C" void test_crc32_short_vectors(void) {
    TEST_ASSERT_EQUAL_HEX32(0xE8B7BE43u, crc_str("a"));
    TEST_ASSERT_EQUAL_HEX32(0x352441C2u, crc_str("abc"));
    TEST_ASSERT_EQUAL_HEX32(0x414FA339u,
                            crc_str("The quick brown fox jumps over the lazy dog"));
}

// A CSV header line, i.e. the actual first thing written to every log file.
extern "C" void test_crc32_csv_header_shape(void) {
    TEST_ASSERT_EQUAL_HEX32(0x75905075u, crc_str("t_ms,state,v_min\n"));
}

extern "C" void test_crc32_zero_filled_block(void) {
    std::uint8_t zeros[64] = {};
    TEST_ASSERT_EQUAL_HEX32(0x758D6336u, crc::compute(zeros, sizeof zeros));
}

// --- incremental use --------------------------------------------------------

// This is how the logger actually uses it: fold each row in as it is written,
// finalize once at seal. Splitting must not change the answer.
extern "C" void test_crc32_incremental_matches_one_shot(void) {
    std::uint32_t running = crc::Crc32Init;
    running = crc::update(running, "1234", 4);
    running = crc::update(running, "56789", 5);
    TEST_ASSERT_EQUAL_HEX32(0xCBF43926u, crc::finalize(running));
}

// Rows arrive one at a time; a byte-at-a-time split is the extreme case.
extern "C" void test_crc32_byte_at_a_time_matches(void) {
    const char*   s       = "The quick brown fox jumps over the lazy dog";
    std::uint32_t running = crc::Crc32Init;
    for (const char* p = s; *p != '\0'; ++p) running = crc::update(running, p, 1);
    TEST_ASSERT_EQUAL_HEX32(0x414FA339u, crc::finalize(running));
}

// Folding nothing in must not disturb a running CRC -- the logger calls
// update() on every drain, including ticks where the ring was empty.
extern "C" void test_crc32_zero_length_update_is_identity(void) {
    std::uint32_t running = crc::update(crc::Crc32Init, "123456789", 9);
    const std::uint32_t before = running;
    running = crc::update(running, "xyz", 0);
    running = crc::update(running, nullptr, 0);
    TEST_ASSERT_EQUAL_HEX32(before, running);
    TEST_ASSERT_EQUAL_HEX32(0xCBF43926u, crc::finalize(running));
}

// finalize() must not be applied twice -- the running state is deliberately
// NOT the wire value, and confusing the two is the easy mistake here.
extern "C" void test_crc32_finalize_is_not_idempotent(void) {
    const std::uint32_t running = crc::update(crc::Crc32Init, "123456789", 9);
    const std::uint32_t once    = crc::finalize(running);
    TEST_ASSERT_EQUAL_HEX32(0xCBF43926u, once);
    TEST_ASSERT_NOT_EQUAL(once, crc::finalize(once));
}

// --- sensitivity ------------------------------------------------------------

// A single flipped bit must change the CRC; otherwise it is not detecting the
// corruption it exists to detect.
extern "C" void test_crc32_detects_single_bit_flip(void) {
    std::uint8_t buf[128];
    for (std::uint8_t i = 0; i < 128; ++i) buf[i] = i;
    const std::uint32_t base = crc::compute(buf, sizeof buf);
    for (std::size_t i = 0; i < sizeof buf; i += 17) {
        buf[i] ^= 0x01u;
        TEST_ASSERT_NOT_EQUAL(base, crc::compute(buf, sizeof buf));
        buf[i] ^= 0x01u;
    }
    TEST_ASSERT_EQUAL_HEX32(base, crc::compute(buf, sizeof buf));
}

// Transposed blocks are the classic checksum blind spot; CRC-32 catches them.
extern "C" void test_crc32_detects_reordering(void) {
    TEST_ASSERT_NOT_EQUAL(crc_str("abcd"), crc_str("abdc"));
    TEST_ASSERT_NOT_EQUAL(crc_str("12,34\n"), crc_str("34,12\n"));
}
