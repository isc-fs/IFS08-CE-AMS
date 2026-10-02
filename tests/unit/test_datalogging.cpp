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
#include <string>
#include <vector>

namespace {

// Expected total CSV columns: 19 head scalars + 95 cells + 200 temperatures +
// 18 tail scalars. Pinned as a literal so adding a column is a deliberate edit
// here as well as in the table.
constexpr int kExpectedCols = 19 + 95 + 200 + 18;

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

namespace {

// Split one CSV line (newline-terminated) into fields. Empty fields stay as
// empty strings, which is what the format uses for "no valid value".
std::vector<std::string> split_csv(const char* line, std::size_t n) {
    std::vector<std::string> out(1);
    for (std::size_t i = 0; i < n; ++i) {
        const char c = line[i];
        if (c == '\n') break;
        if (c == ',') out.emplace_back(); else out.back().push_back(c);
    }
    return out;
}

struct Csv {
    std::vector<std::string> names;
    std::vector<std::string> values;

    // Value of the named column; fails the test if the column does not exist.
    std::string get(const char* name) const {
        for (std::size_t i = 0; i < names.size(); ++i)
            if (names[i] == name) return values[i];
        TEST_FAIL_MESSAGE(name);
        return {};
    }
};

Csv format(const ams::LogRecord& rec) {
    static char hdr[ams::log_csv::MaxRowBytes];
    static char row[ams::log_csv::MaxRowBytes];
    const std::size_t hn = ams::log_csv::build_header(hdr, sizeof hdr);
    const std::size_t rn = ams::log_csv::format_row(rec, row, sizeof row);
    TEST_ASSERT_GREATER_THAN(0u, hn);
    TEST_ASSERT_GREATER_THAN(0u, rn);
    return Csv{ split_csv(hdr, hn), split_csv(row, rn) };
}

}  // namespace

extern "C" void test_logcsv_header_column_count(void) {
    char buf[ams::log_csv::MaxRowBytes];
    const std::size_t n = ams::log_csv::build_header(buf, sizeof buf);
    TEST_ASSERT_GREATER_THAN(0u, n);
    TEST_ASSERT_EQUAL_CHAR('\n', buf[n - 1]);      // newline-terminated
    TEST_ASSERT_EQUAL_INT(kExpectedCols, count_cols(buf, n));
    TEST_ASSERT_EQUAL_INT(kExpectedCols, (int)ams::log_csv::TotalColumns);
}

// Same column count with and without valid BMS data: empty fields still count.
extern "C" void test_logcsv_row_matches_header_columns(void) {
    ams::LogRecord rec{};
    char buf[ams::log_csv::MaxRowBytes];
    for (std::uint8_t valid : {0u, 1u}) {
        rec.bms_valid = valid;
        const std::size_t n = ams::log_csv::format_row(rec, buf, sizeof buf);
        TEST_ASSERT_GREATER_THAN(0u, n);
        TEST_ASSERT_EQUAL_CHAR('\n', buf[n - 1]);
        TEST_ASSERT_EQUAL_INT(kExpectedCols, count_cols(buf, n));  // parity w/ header
    }
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
    rec.bms_valid = 1;
    rec.cell_mV[4][ams::config::CellsPerModule - 1]    = 3777;   // last cell
    rec.cell_tempC[4][ams::config::TempsPerModule - 1] = -5;     // last temp
    const Csv c = format(rec);
    TEST_ASSERT_EQUAL_STRING("3777", c.get("c4_18").c_str());
    TEST_ASSERT_EQUAL_STRING("-5",   c.get("t4_39").c_str());
}

// Existing columns never move: new ones are appended to the tail only.
extern "C" void test_logcsv_existing_columns_keep_positions(void) {
    const Csv c = format(ams::LogRecord{});
    TEST_ASSERT_EQUAL_STRING("tick_ms",    c.names[0].c_str());
    TEST_ASSERT_EQUAL_STRING("tavg_C",     c.names[18].c_str());
    TEST_ASSERT_EQUAL_STRING("c0_0",       c.names[19].c_str());
    TEST_ASSERT_EQUAL_STRING("c4_18",      c.names[113].c_str());
    TEST_ASSERT_EQUAL_STRING("t0_0",       c.names[114].c_str());
    TEST_ASSERT_EQUAL_STRING("t4_39",      c.names[313].c_str());
    TEST_ASSERT_EQUAL_STRING("bal_state",  c.names[314].c_str());
    TEST_ASSERT_EQUAL_STRING("bal_active", c.names[316].c_str());
    TEST_ASSERT_EQUAL_STRING("bms_valid",  c.names[317].c_str());
}

// Every scalar field lands in the column named for it. Each field gets a
// distinct value, so two swapped fields of the same type cannot pass -- the
// check a column count alone cannot make.
extern "C" void test_logcsv_every_field_maps_to_its_column(void) {
    ams::LogRecord r{};
    r.tick_ms = 4000000001u;  r.fsm_state = 3;      r.mode = 2;
    r.ams_ok = 1;             r.fault_reason = 16;  r.fault_detail = 7;
    r.tsms = 5;               r.dash_chg = 6;       r.module_online_mask = 29;
    r.pack_mV = 352100;       r.pack_current_raw_mA = -12345;
    r.pack_current_mA = -12000; r.dcdc_current_mA = -345; r.dc_bus_V = 351;
    r.min_cell_mV = 3301;     r.max_cell_mV = 3402;
    r.min_tempC = -12;        r.max_tempC = 45;     r.avg_tempC = 21;
    r.bal_state = 2;          r.bal_inhibit = 0x0122; r.bal_active = 9;
    r.bms_valid = 1;          r.bms_age_ms = 123;
    r.soc_ppm = 654321;       r.soc_sig_ppm = 1234;
    r.soc_flags = ams::soc::flags::Valid | ams::soc::flags::Corrected;
    r.soc_seeds = 4;
    r.q_dis_mAs = 987654321u; r.q_chg_mAs = 12345678u; r.q_gaps = 11;
    r.dcbus_age_ms = 456;     r.veh_flags = 5;      r.chg_age_ms = 789;
    r.pec_err = 10001;        r.spi_err = 20002;    r.chain_rec = 30003;

    const struct { const char* name; const char* value; } expected[] = {
        {"tick_ms", "4000000001"}, {"fsm", "3"}, {"mode", "2"}, {"ams_ok", "1"},
        {"fault", "16"}, {"detail", "7"}, {"tsms", "5"}, {"dash_chg", "6"},
        {"mod_mask", "29"}, {"pack_mV", "352100"}, {"I_raw_mA", "-12345"},
        {"I_filt_mA", "-12000"}, {"Idcdc_mA", "-345"}, {"dcbus_V", "351"},
        {"vmin_mV", "3301"}, {"vmax_mV", "3402"}, {"tmin_C", "-12"},
        {"tmax_C", "45"}, {"tavg_C", "21"},
        {"bal_state", "2"}, {"bal_inhibit", "290"}, {"bal_active", "9"},
        {"bms_valid", "1"}, {"bms_age_ms", "123"}, {"soc_ppm", "654321"},
        {"soc_sig_ppm", "1234"}, {"soc_flags", "3"}, {"soc_seeds", "4"},
        {"q_dis_mAs", "987654321"}, {"q_chg_mAs", "12345678"}, {"q_gaps", "11"},
        {"dcbus_age_ms", "456"}, {"veh_flags", "5"}, {"chg_age_ms", "789"},
        {"pec_err", "10001"}, {"spi_err", "20002"}, {"chain_rec", "30003"},
    };
    // Every scalar column is covered, so a new column must be added here too.
    TEST_ASSERT_EQUAL_UINT(ams::log_csv::HeadColumns + ams::log_csv::TailColumns,
                           sizeof expected / sizeof expected[0]);

    const Csv c = format(r);
    for (const auto& e : expected) {
        TEST_ASSERT_EQUAL_STRING_MESSAGE(e.value, c.get(e.name).c_str(), e.name);
    }
}

// Before the first full BMS poll every BMS-derived field is empty (the cells
// hold the boot seed), while state, faults and current are still recorded.
extern "C" void test_logcsv_bms_columns_empty_until_valid(void) {
    ams::LogRecord r{};
    r.tick_ms = 250; r.pack_current_raw_mA = 1500;
    r.pack_mV = 351500; r.min_cell_mV = 3700; r.max_cell_mV = 3700;
    r.cell_mV[0][0] = 3700; r.cell_tempC[0][0] = 25;
    r.bms_valid = 0;
    const Csv c = format(r);
    for (const char* n : {"pack_mV", "vmin_mV", "vmax_mV", "tmin_C", "tmax_C",
                          "tavg_C", "c0_0", "c4_18", "t0_0", "t4_39"}) {
        TEST_ASSERT_EQUAL_STRING_MESSAGE("", c.get(n).c_str(), n);
    }
    TEST_ASSERT_EQUAL_STRING("250",  c.get("tick_ms").c_str());
    TEST_ASSERT_EQUAL_STRING("1500", c.get("I_raw_mA").c_str());
    TEST_ASSERT_EQUAL_STRING("0",    c.get("bms_valid").c_str());
}

// SoC and its sigma are empty while the estimator has no estimate -- never 0,
// which is a real reading -- and present once it does.
extern "C" void test_logcsv_soc_empty_while_invalid(void) {
    ams::LogRecord r{};
    r.soc_ppm = 0; r.soc_sig_ppm = 0; r.soc_flags = ams::soc::flags::CoulombOnly;
    Csv c = format(r);
    TEST_ASSERT_EQUAL_STRING("", c.get("soc_ppm").c_str());
    TEST_ASSERT_EQUAL_STRING("", c.get("soc_sig_ppm").c_str());
    TEST_ASSERT_EQUAL_STRING("8", c.get("soc_flags").c_str());

    r.soc_flags = ams::soc::flags::Valid;     // a valid 0 % reading
    c = format(r);
    TEST_ASSERT_EQUAL_STRING("0", c.get("soc_ppm").c_str());
}

// The widest possible row and the header both fit MaxRowBytes.
extern "C" void test_logcsv_widest_row_fits(void) {
    ams::LogRecord r{};
    r.tick_ms = 0xFFFFFFFFu; r.pack_mV = 0xFFFFFFFFu;
    r.pack_current_raw_mA = INT32_MIN; r.pack_current_mA = INT32_MIN;
    r.dcdc_current_mA = INT32_MIN;
    r.min_tempC = r.max_tempC = r.avg_tempC = INT16_MIN;
    for (auto& mod : r.cell_mV)    for (auto& v : mod) v = 0xFFFFu;
    for (auto& mod : r.cell_tempC) for (auto& v : mod) v = INT16_MIN;
    r.bms_valid = 1; r.soc_flags = 0xFF;
    r.soc_ppm = r.soc_sig_ppm = r.q_dis_mAs = r.q_chg_mAs = 0xFFFFFFFFu;
    r.pec_err = r.spi_err = r.chain_rec = 0xFFFFFFFFu;
    r.bal_inhibit = r.bms_age_ms = r.q_gaps = r.dcbus_age_ms = r.chg_age_ms = 0xFFFFu;
    char buf[ams::log_csv::MaxRowBytes];
    TEST_ASSERT_GREATER_THAN(0u, ams::log_csv::format_row(r, buf, sizeof buf));
    TEST_ASSERT_GREATER_THAN(0u, ams::log_csv::build_header(buf, sizeof buf));
}

extern "C" void test_logcsv_truncation_returns_zero(void) {
    ams::LogRecord rec{};
    char tiny[10];                                       // far too small
    TEST_ASSERT_EQUAL_INT(0, (int)ams::log_csv::format_row(rec, tiny, sizeof tiny));
}

// Rows are written from boot, before the first full BMS poll: only the cadence
// decides.
extern "C" void test_logcsv_samples_from_boot(void) {
    constexpr std::uint32_t P = ams::config::LogSamplePeriodMs;
    TEST_ASSERT_TRUE(ams::log_csv::sample_due(P, 0u));
}

extern "C" void test_logcsv_sample_cadence(void) {
    constexpr std::uint32_t P = ams::config::LogSamplePeriodMs;
    TEST_ASSERT_FALSE(ams::log_csv::sample_due(1000u + P - 1u, 1000u));
    TEST_ASSERT_TRUE (ams::log_csv::sample_due(1000u + P,      1000u));
    // Tick wraparound: unsigned subtraction still measures the real gap.
    TEST_ASSERT_TRUE (ams::log_csv::sample_due(P - 10u, 0xFFFFFFF6u));
}

extern "C" void test_logcsv_age_saturates(void) {
    TEST_ASSERT_EQUAL_UINT16(0xFFFF, ams::log_csv::age_ms(5000u, 0u));      // never
    TEST_ASSERT_EQUAL_UINT16(200,    ams::log_csv::age_ms(5200u, 5000u));
    TEST_ASSERT_EQUAL_UINT16(0xFFFF, ams::log_csv::age_ms(200000u, 1000u)); // saturates
    TEST_ASSERT_EQUAL_UINT16(20,     ams::log_csv::age_ms(10u, 0xFFFFFFF6u)); // wrap
}

extern "C" void test_logcsv_newest_age_picks_freshest(void) {
    const std::uint32_t ticks[5] = {1000u, 0u, 1800u, 1500u, 0u};
    TEST_ASSERT_EQUAL_UINT16(200, ams::log_csv::newest_age_ms(2000u, ticks, 5));
    const std::uint32_t none[5] = {};
    TEST_ASSERT_EQUAL_UINT16(0xFFFF, ams::log_csv::newest_age_ms(2000u, none, 5));
}
