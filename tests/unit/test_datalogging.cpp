// SPDX-License-Identifier: proprietary
//
// Unit tests for the microSD datalogging core (both HAL-free):
//   - SpscRing  (log_ring.hpp)    : wait-free FIFO, drop-newest-on-full, wrap
//   - LogRecord (log_record.hpp)  : CSV header/row formatting, column parity
//
// The SdLoggerTask itself (mount/rotate/FatFs) is hardware-bound and lives
// under the HIL acceptance (#407), not here.

#include "log_ring.hpp"
#include "log_record.hpp"

#include "unity.h"

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace {

// Expected total CSV columns: 19 scalars + per-cell + per-thermistor.
constexpr int kExpectedCols =
    19 + ams::config::BmsModuleCount * ams::config::CellsPerModule
       + ams::config::BmsModuleCount * ams::config::TempsPerModule;

int count_cols(const char* s, std::size_t n) {
    int cols = 1;
    for (std::size_t i = 0; i < n; ++i) if (s[i] == ',') ++cols;
    return cols;
}

}  // namespace

// ===========================================================================
// SpscRing
// ===========================================================================

extern "C" void test_logring_empty_pop_false(void) {
    ams::SpscRing<int, 4> r;
    int v = 123;
    TEST_ASSERT_TRUE(r.empty());
    TEST_ASSERT_FALSE(r.pop(v));     // nothing to pop
    TEST_ASSERT_EQUAL_INT(123, v);   // out param untouched
}

extern "C" void test_logring_fifo_order(void) {
    ams::SpscRing<int, 8> r;
    TEST_ASSERT_TRUE(r.push(10));
    TEST_ASSERT_TRUE(r.push(20));
    TEST_ASSERT_TRUE(r.push(30));
    int v = 0;
    TEST_ASSERT_TRUE(r.pop(v)); TEST_ASSERT_EQUAL_INT(10, v);
    TEST_ASSERT_TRUE(r.pop(v)); TEST_ASSERT_EQUAL_INT(20, v);
    TEST_ASSERT_TRUE(r.pop(v)); TEST_ASSERT_EQUAL_INT(30, v);
    TEST_ASSERT_FALSE(r.pop(v));      // drained
}

extern "C" void test_logring_fills_and_drops_newest(void) {
    ams::SpscRing<int, 4> r;
    for (int i = 0; i < 4; ++i) TEST_ASSERT_TRUE(r.push(i));   // fill capacity
    TEST_ASSERT_EQUAL_UINT32(4u, r.size());
    TEST_ASSERT_FALSE(r.push(99));    // full -> drop the NEW record
    int v = 0;
    TEST_ASSERT_TRUE(r.pop(v)); TEST_ASSERT_EQUAL_INT(0, v);   // oldest still 0
    TEST_ASSERT_TRUE(r.push(99));     // room again
}

extern "C" void test_logring_wraparound_fifo(void) {
    // Push/pop far past capacity to exercise the index wrap + masking.
    ams::SpscRing<int, 4> r;
    for (int i = 0; i < 1000; ++i) {
        TEST_ASSERT_TRUE(r.push(i));
        int v = -1;
        TEST_ASSERT_TRUE(r.pop(v));
        TEST_ASSERT_EQUAL_INT(i, v);
    }
    TEST_ASSERT_TRUE(r.empty());
}

extern "C" void test_logring_size_tracking(void) {
    ams::SpscRing<int, 8> r;
    TEST_ASSERT_EQUAL_UINT32(0u, r.size());
    r.push(1); r.push(2); r.push(3);
    TEST_ASSERT_EQUAL_UINT32(3u, r.size());
    TEST_ASSERT_FALSE(r.empty());
    int v;
    r.pop(v); r.pop(v); r.pop(v);
    TEST_ASSERT_EQUAL_UINT32(0u, r.size());
    TEST_ASSERT_TRUE(r.empty());
}

// ===========================================================================
// LogRecord CSV
// ===========================================================================

extern "C" void test_logcsv_header_column_count(void) {
    char buf[ams::log_csv::MaxRowBytes];
    const std::size_t n = ams::log_csv::build_header(buf, sizeof buf);
    TEST_ASSERT_GREATER_THAN(0u, n);
    TEST_ASSERT_EQUAL_CHAR('\n', buf[n - 1]);      // newline-terminated
    TEST_ASSERT_EQUAL_INT(kExpectedCols, count_cols(buf, n));
}

extern "C" void test_logcsv_row_matches_header_columns(void) {
    ams::LogRecord rec{};
    char buf[ams::log_csv::MaxRowBytes];
    const std::size_t n = ams::log_csv::format_row(rec, buf, sizeof buf);
    TEST_ASSERT_GREATER_THAN(0u, n);
    TEST_ASSERT_EQUAL_CHAR('\n', buf[n - 1]);
    TEST_ASSERT_EQUAL_INT(kExpectedCols, count_cols(buf, n));  // parity w/ header
}

extern "C" void test_logcsv_row_scalar_values(void) {
    ams::LogRecord rec{};
    rec.tick_ms = 12345; rec.fsm_state = 4; rec.mode = 2; rec.ams_ok = 1;
    rec.pack_current_mA = -1480;
    char buf[ams::log_csv::MaxRowBytes];
    const std::size_t n = ams::log_csv::format_row(rec, buf, sizeof buf);
    TEST_ASSERT_GREATER_THAN(0u, n);
    // Scalar block leads the row: tick,fsm,mode,ams_ok,...
    TEST_ASSERT_EQUAL_INT(0, std::strncmp(buf, "12345,4,2,1,", 12));
    // Signed current renders with its sign.
    TEST_ASSERT_NOT_NULL(std::strstr(buf, ",-1480,"));
}

extern "C" void test_logcsv_row_cell_and_temp_values(void) {
    ams::LogRecord rec{};
    rec.cell_mV[4][ams::config::CellsPerModule - 1]    = 3777;   // last cell
    rec.cell_tempC[4][ams::config::TempsPerModule - 1] = -5;     // last temp
    char buf[ams::log_csv::MaxRowBytes];
    const std::size_t n = ams::log_csv::format_row(rec, buf, sizeof buf);
    TEST_ASSERT_GREATER_THAN(0u, n);
    TEST_ASSERT_NOT_NULL(std::strstr(buf, ",3777,"));   // last cell, mid-row
    TEST_ASSERT_NOT_NULL(std::strstr(buf, ",-5\n"));     // last temp, end-of-row
}

extern "C" void test_logcsv_truncation_returns_zero(void) {
    ams::LogRecord rec{};
    char tiny[10];                                       // far too small
    TEST_ASSERT_EQUAL_INT(0, (int)ams::log_csv::format_row(rec, tiny, sizeof tiny));
}
