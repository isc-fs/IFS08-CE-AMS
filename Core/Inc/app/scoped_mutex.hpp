// SPDX-License-Identifier: proprietary
//
// RAII wrapper around osMutexAcquire / osMutexRelease.
//
// All mutex acquisition in app code MUST go through ScopedMutex (per
// docs/CONTRIBUTING.md "C++ rules"). Eliminates the "forgot to release
// on early return / exception path" class of bugs.

#pragma once

#include "cmsis_os2.h"
#include <cstdint>

namespace ams {

// Acquires `mutex` for the lifetime of the object.
//
// Constructor blocks for at most `timeout_ms`. If acquisition fails
// (timeout, invalid id), `acquired()` returns false and the destructor
// is a no-op. Callers MUST check `acquired()` before touching the
// protected resource if they pass a non-default timeout.
//
// Example:
//   {
//       ScopedMutex lock(BmsService::mutex());
//       if (!lock.acquired()) {
//           return;  // log / count / FORCE_ERROR per ARCHITECTURE.md §8
//       }
//       //... read or write BmsState ...
//   }
class ScopedMutex {
public:
    explicit ScopedMutex(osMutexId_t mutex,
                         std::uint32_t timeout_ms = osWaitForever) noexcept
        : mutex_{mutex},
          status_{osMutexAcquire(mutex, timeout_ms)} {}

    ~ScopedMutex() noexcept {
        if (status_ == osOK) {
            osMutexRelease(mutex_);
        }
    }

    // Non-copyable, non-movable: must die in the scope where it was born.
    ScopedMutex(const ScopedMutex&)            = delete;
    ScopedMutex& operator=(const ScopedMutex&) = delete;
    ScopedMutex(ScopedMutex&&)                 = delete;
    ScopedMutex& operator=(ScopedMutex&&)      = delete;

    [[nodiscard]] bool acquired() const noexcept {
        return status_ == osOK;
    }

private:
    osMutexId_t  mutex_;
    osStatus_t   status_;
};

}  // namespace ams
