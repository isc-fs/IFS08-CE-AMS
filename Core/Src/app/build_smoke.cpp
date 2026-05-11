// SPDX-License-Identifier: proprietary
//
// Smoke test that the C++ compilation path is wired up. Pulls in the
// foundational app headers (ams_config, scoped_mutex, can_frame) and
// exposes one extern "C" symbol so the linker sees an actual .cpp
// object in the build.
//
// Delete or replace this file when the first real service lands in
// feat/5+.

#include "ams_config.hpp"
#include "can_frame.hpp"
#include "scoped_mutex.hpp"

#include <cstdint>

namespace {

// Touch every constexpr so any future change that breaks compilation
// fails here at the earliest possible point. ODR-used via the function
// below; otherwise the compiler may drop them silently.
constexpr std::uint32_t kSanityMagic =
    static_cast<std::uint32_t>(ams::config::kCellUVmV)        ^
    static_cast<std::uint32_t>(ams::config::kCellOVmV)        ^
    static_cast<std::uint32_t>(ams::config::kSafetyPeriodMs)  ^
    static_cast<std::uint32_t>(ams::config::kBmsPollVoltMs)   ^
    ams::config::kAcuRxChargerId                              ^
    ams::config::kBkpErrorMagic;

static_assert(sizeof(ams::CanFrame) <= 32, "CanFrame layout drifted");
static_assert(ams::kCanFrameMaxData == AMS_CAN_FRAME_MAX_DATA,
              "C and C++ CanFrame max-data must agree");

}  // namespace

extern "C" std::uint32_t ams_build_sanity_magic(void);
extern "C" std::uint32_t ams_build_sanity_magic(void)
{
    return kSanityMagic;
}
