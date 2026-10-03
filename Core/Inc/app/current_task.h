/* SPDX-License-Identifier: proprietary
 *
 * CurrentSensorTask. main.c's USER CODE BEGIN StartCurrentSensorTask block
 * calls the C entry point; the C++ section is the read-only window other
 * tasks get onto the pack-current capture stream.
 */

#ifndef AMS_CURRENT_TASK_H
#define AMS_CURRENT_TASK_H

#ifdef __cplusplus
#include <cstdint>

namespace ams {

// A point in the pack-current capture stream: capture id + sample index.
struct CurrentMark {
    std::uint32_t capture;
    std::uint16_t index;
};

// Mean pack current over a window that starts at a mark.
struct CurrentWindow {
    std::uint16_t n;        // samples averaged; 0 = not available
    std::uint16_t span_us;  // time those samples cover
    std::int32_t  mA;       // + = discharge
};

// Where the running capture is right now. False if no capture is running.
// TELEMETRY ONLY. Callable only from tasks BELOW CurrentSensorTask's priority
// (the bookkeeping is a seqlock whose writer must outrank its readers).
bool current_mark(CurrentMark& out) noexcept;

// Mean current over `window_us` starting at `from`, from the capture buffers.
// Shorter (n smaller) if the capture ended inside the window; n = 0 if the
// samples were already overwritten or never existed. TELEMETRY ONLY; same
// caller rule as current_mark().
CurrentWindow current_window_mean(const CurrentMark& from, std::uint32_t window_us) noexcept;

}  // namespace ams

extern "C" {
#endif

void ams_current_sensor_task_run(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* AMS_CURRENT_TASK_H */
