// SPDX-License-Identifier: proprietary
//
// NTC resistance-to-temperature table for the BMS_LITE cell thermistors.
//
// The fitted part is a Fenghua CMFB103F3950FANT: R25 = 10 kOhm (1 pct), with
// B25/50 = 3950 K and B25/85 = 4021 K. Source data is the manufacturer's R-T
// appendix, committed as docs/ntc_rt_table.csv alongside this header.
//
// WHY A TABLE AND NOT THE BETA MODEL
// -----------------------------------
// The firmware previously used a single-beta Steinhart approximation with TWO
// wrong constants, and they partially cancelled:
//
//   * NtcBeta    = 3380  -- the beta of a Murata NCP15XH103J, NOT the fitted
//                           Fenghua part (3950). Too small a beta reads HOT
//                           above 25 degC.
//   * NtcSeriesR = 10000 -- the divider pull-up is actually 6.8 kOhm
//                           (R145 / R170). Overstating it inflates the
//                           recovered R_ntc by 1.47x and reads COLD.
//
// Net was ~7 degC COLD at 50 degC, so BalanceTempMax = 50 tripped at ~56 degC
// true and CellOverTempC = 60 at ~65 degC true -- both ABOVE the cell's rated
// envelope. Fixing either constant ALONE makes it worse, which is why the
// table and the divider fix land in one change.
//
// A single beta is only accurate near its fitting interval anyway: at -40 degC
// it misestimates R by tens of percent, worth several degrees. The table is
// exact at every listed point.
//
// Cross-check: this is the same conversion tools/bms_monitor.py runs on the
// bare-LTC bench harness (RPULL_KOHM = 6.8, VREF2_MV = 3000), which was
// self-checked against the datasheet rows.
//
// Pure logic: no HAL, no RTOS, host-testable.

#pragma once

#include <cstddef>
#include <cstdint>

namespace ams::ntc {

// Table covers TableMinC..TableMaxC inclusive in 1 degC steps.
inline constexpr std::int16_t TableMinC = -55;
inline constexpr std::int16_t TableMaxC = 125;
inline constexpr std::size_t  TableLen  = 181;

// R_ntc in ohms at (TableMinC + index), strictly DECREASING (it is an NTC).
inline constexpr std::uint32_t ResistanceOhm[TableLen] = {
     583542u,  554647u,  526968u,  500480u,  475159u,  450974u,
     427897u,  405892u,  384927u,  364967u,  345975u,  327915u,
     310751u,  294448u,  278969u,  264279u,  250344u,  237130u,
     224603u,  212733u,  201487u,  190836u,  180750u,  171201u,
     162163u,  153610u,  145516u,  137858u,  130614u,  123761u,
     117280u,  111149u,  105351u,   99867u,   94681u,   89776u,
      85137u,   80750u,   76600u,   72676u,   68963u,   65451u,
      62129u,   58986u,   56012u,   53198u,   50534u,   48013u,
      45627u,   43368u,   41229u,   39204u,   37285u,   35468u,
      33747u,   32116u,   30570u,   29105u,   27716u,   26399u,
      25150u,   23965u,   22842u,   21776u,   20764u,   19783u,
      18892u,   18026u,   17204u,   16423u,   15681u,   14976u,
      14306u,   13669u,   13063u,   12487u,   11939u,   11418u,
      10921u,   10449u,   10000u,    9571u,    9164u,    8775u,
       8405u,    8052u,    7716u,    7396u,    7090u,    6798u,
       6520u,    6255u,    6002u,    5760u,    5529u,    5309u,
       5098u,    4897u,    4704u,    4521u,    4345u,    4177u,
       4016u,    3863u,    3716u,    3588u,    3440u,    3311u,
       3188u,    3069u,    2956u,    2848u,    2744u,    2644u,
       2548u,    2457u,    2369u,    2284u,    2204u,    2126u,
       2051u,    1980u,    1911u,    1845u,    1782u,    1721u,
       1663u,    1606u,    1552u,    1500u,    1450u,    1402u,
       1356u,    1312u,    1269u,    1228u,    1188u,    1150u,
       1113u,    1078u,    1044u,    1011u,     979u,     948u,
        919u,     891u,     863u,     837u,     811u,     787u,
        763u,     740u,     718u,     697u,     676u,     657u,
        637u,     619u,     601u,     584u,     567u,     551u,
        535u,     520u,     505u,     491u,     477u,     464u,
        451u,     439u,     427u,     415u,     404u,     393u,
        383u,     373u,     363u,     353u,     344u,     335u,
        326u,
};

// Reverse-interpolate a measured resistance to degC, scaled by 10 so the
// caller can round without a second float. Returns false if r_ohm falls
// outside the table -- an open or shorted channel, or a part this table does
// not describe.
//
// Linear interpolation between 1 degC points. R is exponential in T, so this
// is not exact, but across a single degree the error is far below the part's
// own tolerance and the LTC's measurement error. Interpolating ln(R) would
// cost a logf for no useful gain.
[[nodiscard]] inline bool resistance_to_decideg(std::uint32_t r_ohm,
                                                std::int32_t& decideg_out) noexcept {
    if (r_ohm > ResistanceOhm[0] || r_ohm < ResistanceOhm[TableLen - 1]) {
        return false;
    }
    // Binary search for the bracketing pair: ResistanceOhm[lo] >= r > [lo+1].
    std::size_t lo = 0, hi = TableLen - 1;
    while (hi - lo > 1) {
        const std::size_t mid = lo + (hi - lo) / 2;
        if (ResistanceOhm[mid] >= r_ohm) lo = mid; else hi = mid;
    }
    const std::uint32_t r_lo = ResistanceOhm[lo];      // higher R, lower T
    const std::uint32_t r_hi = ResistanceOhm[lo + 1];  // lower  R, higher T
    const std::uint32_t span = r_lo - r_hi;            // > 0: table is strict

    const std::int32_t frac_tenths =
        (span == 0u) ? 0
                     : static_cast<std::int32_t>(
                           ((r_lo - r_ohm) * 10u + span / 2u) / span);

    decideg_out = (static_cast<std::int32_t>(TableMinC) + static_cast<std::int32_t>(lo)) * 10
                  + frac_tenths;
    return true;
}

}  // namespace ams::ntc
