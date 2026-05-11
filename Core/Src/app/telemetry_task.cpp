// SPDX-License-Identifier: proprietary
//
// UART2 telemetry @ 500 ms. One human-readable line per cycle, ASCII
// for log capture. Format intentionally fixed-width so a downstream
// parser (or grep) can pull columns out.

#include "app/telemetry_task.h"

#include "ams_config.hpp"
#include "bms_service.hpp"
#include "current_service.hpp"
#include "vehicle_service.hpp"

#include "cmsis_os2.h"
#include "main.h"

#include <cstdio>

extern "C" {
extern UART_HandleTypeDef huart2;

// Telemeter the current FSM state so we can see it in the UART log
// without dragging StateTask deps into here.
extern volatile std::uint8_t g_state_telemetry;
}

namespace {

constexpr char    kStateChar[6] = { 'S', 'P', 'T', 'R', 'C', 'E' };
constexpr std::size_t kBufLen   = 128;

}  // namespace

extern "C" void ams_telemetry_task_run(void *argument) {
    (void)argument;

    std::uint32_t last_wake = osKernelGetTickCount();

    for (;;) {
        last_wake += ams::config::kTelemetryPeriodMs;
        osDelayUntil(last_wake);

        const auto bms = ams::BmsService::instance().snapshot();
        const auto cur = ams::CurrentService::instance().snapshot();
        const auto veh = ams::VehicleService::instance().snapshot();

        char buf[kBufLen];
        const std::uint8_t st = g_state_telemetry;
        const char schar = (st < 6u) ? kStateChar[st] : '?';

        const int n = std::snprintf(
            buf, kBufLen,
            "AMS s=%c pV=%lumV minV=%umV maxV=%umV minT=%dC maxT=%dC I=%ldmA dcBus=%uV\r\n",
            schar,
            (unsigned long)bms.pack_voltage_mV,
            (unsigned)bms.min_cell_mV,
            (unsigned)bms.max_cell_mV,
            (int)bms.min_tempC,
            (int)bms.max_tempC,
            (long)cur.filtered_mA,
            (unsigned)veh.dc_bus_V);

        if (n > 0) {
            HAL_UART_Transmit(&huart2,
                              reinterpret_cast<std::uint8_t *>(buf),
                              static_cast<std::uint16_t>(n),
                              50u);
        }
    }
}
