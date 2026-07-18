// SPDX-License-Identifier: proprietary
//
// SpscRing -- a wait-free single-producer / single-consumer ring buffer.
//
// The datalogging hand-off: SafetyTask (realtime, the SOLE producer) pushes a
// LogRecord each sample; SdLoggerTask (low priority, the SOLE consumer) drains
// it to the microSD. No mutex -- the producer owns `head_`, the consumer owns
// `tail_`, and release/acquire ordering publishes the slot data before the
// index advance becomes visible to the other side. Capacity MUST be a power
// of two so the index wrap is a mask.
//
// Full-buffer policy: push() returns false and the producer drops the NEW
// record. This is the only SPSC-safe choice -- dropping the OLDEST would mean
// the producer advancing `tail_`, which the consumer owns. A dropped-record
// counter on the producer side gives visibility. Logging is best-effort and
// must NEVER block or perturb the 10 ms safety loop.
//
// Pure (no HAL, no RTOS) -> host-testable alongside the FSM/predicate core.

#pragma once

#include <atomic>
#include <cstdint>

namespace ams {

template <typename T, std::uint32_t Capacity>
class SpscRing {
    static_assert(Capacity >= 2u, "Capacity must be >= 2");
    static_assert((Capacity & (Capacity - 1u)) == 0u, "Capacity must be a power of two");

public:
    // Producer side (SafetyTask). Returns false if full -> caller drops.
    bool push(const T& item) noexcept {
        const std::uint32_t head = head_.load(std::memory_order_relaxed);
        const std::uint32_t tail = tail_.load(std::memory_order_acquire);
        if ((head - tail) >= Capacity) {
            return false;  // full
        }
        buf_[head & kMask] = item;
        head_.store(head + 1u, std::memory_order_release);
        return true;
    }

    // Consumer side (SdLoggerTask). Returns false if empty.
    bool pop(T& out) noexcept {
        const std::uint32_t tail = tail_.load(std::memory_order_relaxed);
        const std::uint32_t head = head_.load(std::memory_order_acquire);
        if (head == tail) {
            return false;  // empty
        }
        out = buf_[tail & kMask];
        tail_.store(tail + 1u, std::memory_order_release);
        return true;
    }

    // Approximate occupancy (exact for the owning side). uint32 monotonic
    // counters -> the unsigned difference is wrap-safe.
    [[nodiscard]] std::uint32_t size() const noexcept {
        return head_.load(std::memory_order_acquire) -
               tail_.load(std::memory_order_acquire);
    }

    [[nodiscard]] bool empty() const noexcept { return size() == 0u; }

    static constexpr std::uint32_t capacity() noexcept { return Capacity; }

private:
    static constexpr std::uint32_t kMask = Capacity - 1u;

    std::atomic<std::uint32_t> head_{0};  // producer writes, consumer reads
    std::atomic<std::uint32_t> tail_{0};  // consumer writes, producer reads
    T buf_[Capacity]{};
};

}  // namespace ams
