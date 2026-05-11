// SPDX-License-Identifier: proprietary

#include "cmsis_os2.h"

#include <cstdint>

extern "C" {

// Single-threaded test driver -> mutex is degenerate but always
// "acquired". A future thread-aware fixture would replace these
// with a real pthread/mutex; for now success-always is correct.
osStatus_t osMutexAcquire(osMutexId_t /*mutex*/, std::uint32_t /*timeout*/) {
    return osOK;
}

osStatus_t osMutexRelease(osMutexId_t /*mutex*/) {
    return osOK;
}

// Tick counter is controllable from tests via fake_set_tick / fake_advance_tick
// (declared in test_helpers.hpp). Default is 0.
static std::uint32_t s_tick = 0u;

uint32_t osKernelGetTickCount(void) {
    return s_tick;
}

// Services dereference these; non-null is the only requirement.
osMutexId_t bms_mutexHandle     = reinterpret_cast<osMutexId_t>(0x1);
osMutexId_t current_mutexHandle = reinterpret_cast<osMutexId_t>(0x2);

// Test-only helpers: not declared in the mock cmsis_os2.h since they
// are unique to the test fixture.
void fake_set_tick(std::uint32_t t)    { s_tick = t; }
void fake_advance_tick(std::uint32_t d) { s_tick += d; }

}  // extern "C"
