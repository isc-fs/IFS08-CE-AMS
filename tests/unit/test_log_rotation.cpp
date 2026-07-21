// SPDX-License-Identifier: proprietary
//
// Tests for ams::log_rotation -- file index selection and rotation policy.
//
// These exist because of a real data-loss bug: index selection looked only at
// LOGnnnn.CSV, so a run ending before the size cap left LOGnnnn.TMP behind, the
// next boot chose the SAME index, and the file was reopened with
// FA_CREATE_ALWAYS -- silently truncating the previous run. The symptom was a
// card that only ever held one .TMP and no .CSV at all.

#include "log_rotation.hpp"

#include "unity.h"

#include <cstdint>

namespace {

using namespace ams;

// Fake card: which indices hold a .CSV and/or a .TMP.
struct FakeCard {
    static constexpr std::uint32_t kMax = 16u;
    bool csv[kMax] = {};
    bool tmp[kMax] = {};

    int  seals            = 0;
    bool seal_should_fail = false;
    std::uint32_t last_sealed = 0xFFFFFFFFu;

    auto exists_fn() {
        return [this](std::uint32_t i, bool sealed) noexcept {
            if (i >= kMax) return false;
            return sealed ? csv[i] : tmp[i];
        };
    }
    auto seal_fn() {
        return [this](std::uint32_t i) noexcept {
            ++seals;
            last_sealed = i;
            if (seal_should_fail) return false;
            if (i < kMax) { tmp[i] = false; csv[i] = true; }
            return true;
        };
    }
    std::uint32_t next() {
        return log_rotation::next_index(exists_fn(), seal_fn());
    }
};

}  // namespace

// --- index selection --------------------------------------------------------

extern "C" void test_logrot_empty_card_starts_at_zero(void) {
    FakeCard c;
    TEST_ASSERT_EQUAL_UINT32(0u, c.next());
    TEST_ASSERT_EQUAL_INT(0, c.seals);
}

extern "C" void test_logrot_skips_sealed_files(void) {
    FakeCard c;
    c.csv[0] = c.csv[1] = c.csv[2] = true;
    TEST_ASSERT_EQUAL_UINT32(3u, c.next());
    TEST_ASSERT_EQUAL_INT(0, c.seals);   // nothing to seal
}

// THE BUG: an orphan .TMP must make its index taken. Returning 0 here is what
// destroyed the previous run.
extern "C" void test_logrot_orphan_tmp_does_not_reuse_index(void) {
    FakeCard c;
    c.tmp[0] = true;
    const std::uint32_t idx = c.next();
    TEST_ASSERT_NOT_EQUAL(0u, idx);
    TEST_ASSERT_EQUAL_UINT32(1u, idx);
}

// ...and it is sealed, not merely skipped, so the previous run becomes a real
// .CSV that tools (and the #406 extractor) can see.
extern "C" void test_logrot_orphan_tmp_is_sealed(void) {
    FakeCard c;
    c.tmp[0] = true;
    (void)c.next();
    TEST_ASSERT_EQUAL_INT(1, c.seals);
    TEST_ASSERT_EQUAL_UINT32(0u, c.last_sealed);
    TEST_ASSERT_TRUE(c.csv[0]);
    TEST_ASSERT_FALSE(c.tmp[0]);
}

extern "C" void test_logrot_seals_every_orphan_on_the_way(void) {
    FakeCard c;
    c.csv[0] = true;
    c.tmp[1] = true;
    c.tmp[2] = true;
    c.csv[3] = true;
    TEST_ASSERT_EQUAL_UINT32(4u, c.next());
    TEST_ASSERT_EQUAL_INT(2, c.seals);
    TEST_ASSERT_TRUE(c.csv[1]);
    TEST_ASSERT_TRUE(c.csv[2]);
}

// A failed rename must NOT hand the index back -- that is the truncation bug
// again, just via a different route.
extern "C" void test_logrot_failed_seal_still_consumes_index(void) {
    FakeCard c;
    c.tmp[0]           = true;
    c.seal_should_fail = true;
    TEST_ASSERT_EQUAL_UINT32(1u, c.next());
    TEST_ASSERT_EQUAL_INT(1, c.seals);
    TEST_ASSERT_TRUE(c.tmp[0]);          // still there, but not reused
}

// A .TMP sitting beside an existing .CSV of the same index is left alone: the
// sealed file is authoritative and must never be overwritten by a rename.
extern "C" void test_logrot_tmp_beside_existing_csv_is_not_sealed(void) {
    FakeCard c;
    c.csv[0] = true;
    c.tmp[0] = true;
    TEST_ASSERT_EQUAL_UINT32(1u, c.next());
    TEST_ASSERT_EQUAL_INT(0, c.seals);
}

// Consecutive boots must keep marching forward, never back onto live data.
extern "C" void test_logrot_successive_runs_advance(void) {
    FakeCard c;
    std::uint32_t idx = c.next();
    TEST_ASSERT_EQUAL_UINT32(0u, idx);
    for (std::uint32_t run = 1; run < 6u; ++run) {
        c.tmp[idx] = true;              // the run we just "did" ended unsealed
        idx = c.next();                 // reboot
        TEST_ASSERT_EQUAL_UINT32(run, idx);
        TEST_ASSERT_TRUE(c.csv[run - 1]);   // previous run survived, sealed
    }
}

// --- rotation policy --------------------------------------------------------

extern "C" void test_logrot_rotates_on_size(void) {
    TEST_ASSERT_TRUE(log_rotation::should_rotate(config::LogFileMaxBytes, 100u, 0u));
    TEST_ASSERT_TRUE(log_rotation::should_rotate(config::LogFileMaxBytes + 1u, 1u, 0u));
    TEST_ASSERT_FALSE(log_rotation::should_rotate(config::LogFileMaxBytes - 1u, 100u, 0u));
}

extern "C" void test_logrot_rotates_on_time(void) {
    TEST_ASSERT_TRUE(log_rotation::should_rotate(1024u, 10u, config::LogFileMaxMs));
    TEST_ASSERT_TRUE(log_rotation::should_rotate(1024u, 10u, config::LogFileMaxMs + 1u));
    TEST_ASSERT_FALSE(log_rotation::should_rotate(1024u, 10u, config::LogFileMaxMs - 1u));
}

// A stalled producer must not litter the card with header-only files.
extern "C" void test_logrot_time_rotation_needs_rows(void) {
    TEST_ASSERT_FALSE(log_rotation::should_rotate(200u, 0u, config::LogFileMaxMs * 10u));
    TEST_ASSERT_TRUE(log_rotation::should_rotate(200u, 1u, config::LogFileMaxMs));
}

extern "C" void test_logrot_fresh_file_does_not_rotate(void) {
    TEST_ASSERT_FALSE(log_rotation::should_rotate(0u, 0u, 0u));
    TEST_ASSERT_FALSE(log_rotation::should_rotate(4096u, 3u, 1000u));
}

// The time cap must actually be reachable in a normal session -- that is the
// whole point of adding it. At ~5.3 KiB/s a 4 MiB file is ~13 min.
extern "C" void test_logrot_time_cap_is_shorter_than_size_cap(void) {
    const std::uint32_t bytes_per_s = 5408u;   // ~1352 B/row at 4 Hz
    const std::uint32_t secs_to_size_cap = config::LogFileMaxBytes / bytes_per_s;
    const std::uint32_t secs_to_time_cap = config::LogFileMaxMs / 1000u;
    TEST_ASSERT_LESS_THAN_UINT32(secs_to_size_cap, secs_to_time_cap);
}
