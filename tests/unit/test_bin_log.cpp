// SPDX-License-Identifier: proprietary
//
// Tests for bin_log.hpp -- the binary companion-file header and the schemas of
// IMUnnnn.BIN and CELnnnn.BIN. The schema text is what tools/log_decode.py
// trusts, so these tests parse it the way the decoder does and hold it to the
// record structs and the firmware's own unit conversions.

#include "bin_log.hpp"
#include "imu_record.hpp"

#include "unity.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

using namespace ams;

struct Field {
    std::string name, type, count, scale, unit, flags;
};

std::vector<Field> parse_schema(const char* text) {
    std::vector<Field> out;
    std::string s(text);
    std::size_t pos = 0;
    while (pos < s.size()) {
        const std::size_t eol = s.find('\n', pos);
        const std::string line = s.substr(pos, eol - pos);
        pos = (eol == std::string::npos) ? s.size() : eol + 1;
        std::vector<std::string> tok;
        std::size_t a = 0;
        while (a < line.size()) {
            const std::size_t b = line.find(' ', a);
            tok.push_back(line.substr(a, b - a));
            a = (b == std::string::npos) ? line.size() : b + 1;
        }
        if (tok.size() < 5) { out.push_back(Field{}); continue; }   // malformed -> empty name
        out.push_back(Field{tok[0], tok[1], tok[2], tok[3], tok[4], tok.size() > 5 ? tok[5] : ""});
    }
    return out;
}

std::size_t type_bytes(const std::string& t) {
    if (t == "u8"  || t == "i8")  return 1;
    if (t == "u16" || t == "i16") return 2;
    if (t == "u32" || t == "i32") return 4;
    return 0;
}

std::size_t count_of(const std::string& c) {
    const std::size_t x = c.find('x');
    if (x == std::string::npos) return std::strtoul(c.c_str(), nullptr, 10);
    return std::strtoul(c.substr(0, x).c_str(), nullptr, 10) *
           std::strtoul(c.substr(x + 1).c_str(), nullptr, 10);
}

double scale_of(const std::string& s) {
    const std::size_t slash = s.find('/');
    if (slash == std::string::npos) return std::strtod(s.c_str(), nullptr);
    return std::strtod(s.substr(0, slash).c_str(), nullptr) /
           std::strtod(s.substr(slash + 1).c_str(), nullptr);
}

// Sum of the field sizes the schema declares; 0 if any line is malformed.
std::size_t schema_record_bytes(const char* text) {
    std::size_t total = 0;
    for (const Field& f : parse_schema(text)) {
        const std::size_t tb = type_bytes(f.type);
        const std::size_t n  = count_of(f.count);
        if (f.name.empty() || tb == 0 || n == 0) return 0;
        total += tb * n;
    }
    return total;
}

const Field* find(const std::vector<Field>& fs, const char* name) {
    for (const Field& f : fs) if (f.name == name) return &f;
    return nullptr;
}

std::uint32_t le32(const std::uint8_t* p) {
    return p[0] | (p[1] << 8) | (p[2] << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
}

}  // namespace

extern "C" void test_binlog_header_fields(void) {
    std::uint8_t h[bin_log::HeaderBytes];
    const bin_log::HeaderInfo info{bin_log::CelStream, bin_log::CelSchema,
                                   static_cast<std::uint16_t>(sizeof(bin_log::CelFrame)),
                                   42u, 0x01020304u, {3, 1, 0}, {0xDE, 0xAD, 0xBE, 0xEF}};
    TEST_ASSERT_TRUE(bin_log::build_header(h, sizeof h, info));

    TEST_ASSERT_EQUAL_MEMORY("AMSBIN1", h, 8);                  // incl. NUL
    TEST_ASSERT_EQUAL_UINT16(bin_log::FormatVersion, h[8] | (h[9] << 8));
    TEST_ASSERT_EQUAL_UINT16(208u, h[10] | (h[11] << 8));
    TEST_ASSERT_EQUAL_UINT32(42u, le32(h + 12));
    TEST_ASSERT_EQUAL_UINT32(0x01020304u, le32(h + 16));
    TEST_ASSERT_EQUAL_STRING("CEL", reinterpret_cast<const char*>(h + 20));
    TEST_ASSERT_EQUAL_UINT8(3u, h[28]);
    TEST_ASSERT_EQUAL_UINT8(1u, h[29]);
    TEST_ASSERT_EQUAL_UINT8(0u, h[30]);
    TEST_ASSERT_EQUAL_HEX32(0xEFBEADDEu, le32(h + 32));
    TEST_ASSERT_EQUAL_STRING(bin_log::CelSchema,
                             reinterpret_cast<const char*>(h + bin_log::SchemaOffset));
    TEST_ASSERT_EQUAL_UINT8(0u, h[bin_log::HeaderBytes - 1]);   // schema NUL-terminated, tail zero
}

extern "C" void test_binlog_header_rejects_what_does_not_fit(void) {
    std::uint8_t h[bin_log::HeaderBytes];
    bin_log::HeaderInfo info{"TOOLONGX", bin_log::ImuSchema, 16u, 0u, 0u, {0, 0, 0}, {0, 0, 0, 0}};
    TEST_ASSERT_FALSE(bin_log::build_header(h, sizeof h, info));   // stream name needs its NUL

    const std::string big(bin_log::SchemaMax, 'x');
    info.stream = "IMU";
    info.schema = big.c_str();
    TEST_ASSERT_FALSE(bin_log::build_header(h, sizeof h, info));

    std::uint8_t small[bin_log::HeaderBytes - 1];
    info.schema = bin_log::ImuSchema;
    TEST_ASSERT_FALSE(bin_log::build_header(small, sizeof small, info));
}

// The schema must describe the record byte for byte, or the decoder misreads
// every field after the first mismatch.
extern "C" void test_binlog_schema_sizes_match_records(void) {
    TEST_ASSERT_EQUAL_UINT(sizeof(ImuSample), schema_record_bytes(bin_log::ImuSchema));
    TEST_ASSERT_EQUAL_UINT(sizeof(bin_log::CelFrame), schema_record_bytes(bin_log::CelSchema));
}

// The decoder scales IMU counts by the schema's ratios; they must give the
// same numbers as the firmware's reference conversions.
extern "C" void test_binlog_imu_scales_match_firmware(void) {
    const auto fs = parse_schema(bin_log::ImuSchema);
    const Field* a = find(fs, "a");
    const Field* g = find(fs, "g");
    TEST_ASSERT_NOT_NULL(a);
    TEST_ASSERT_NOT_NULL(g);
    TEST_ASSERT_EQUAL_STRING("g", a->unit.c_str());
    TEST_ASSERT_EQUAL_STRING("rad/s", g->unit.c_str());
    for (std::int16_t c : {std::int16_t{-32768}, std::int16_t{-1}, std::int16_t{1},
                           std::int16_t{16384}, std::int16_t{32767}}) {
        TEST_ASSERT_EQUAL_INT(bmi088::acc_g_e4(c),
                              static_cast<int>(std::lround(c * scale_of(a->scale) * 1e4)));
        TEST_ASSERT_EQUAL_INT(bmi088::gyr_rad_s_e4(c),
                              static_cast<int>(std::lround(c * scale_of(g->scale) * 1e4)));
    }
}

// Cells: a 5x19 matrix in mV where a raw 0 means "not read on this frame".
extern "C" void test_binlog_cel_schema_cells(void) {
    const auto fs = parse_schema(bin_log::CelSchema);
    const Field* c = find(fs, "c");
    TEST_ASSERT_NOT_NULL(c);
    TEST_ASSERT_EQUAL_STRING("5x19", c->count.c_str());
    TEST_ASSERT_EQUAL_STRING("mV", c->unit.c_str());
    TEST_ASSERT_EQUAL_STRING("z", c->flags.c_str());
    const Field* i = find(fs, "i");
    TEST_ASSERT_NOT_NULL(i);
    TEST_ASSERT_EQUAL_STRING("A", i->unit.c_str());
    TEST_ASSERT_TRUE(std::fabs(scale_of(i->scale) - 0.001) < 1e-12);
}
