// SPDX-License-Identifier: proprietary
//
// ISO-TP (ISO 15765-2) style multi-frame transport over classic CAN --
// application side. Pure logic: no HAL, no FreeRTOS, no allocation, so the
// host unit-test build exercises reassembly + segmentation directly.
//
// WIRE FORMAT IS NOT OURS TO CHOOSE. This is a faithful port of the
// bootloader's transport (stm32-can-bootloader `Core/Inc/bl_isotp.h` +
// `bl_isotp.c`), which the host (`can-flasher` / MingoCAN) already speaks.
// KEEP THE CONSTANTS BELOW IN SYNC with bl_isotp.h -- a divergence here is a
// silent interop break, not a compile error.
//
// Every protocol frame carries a PCI byte as byte 0 of its 8-byte payload;
// the high nibble selects the frame kind:
//
//   Single Frame    0x0L   L = payload length 1..7
//   First Frame     0x1H   H = high nibble of the 12-bit total length,
//                          byte 1 = low byte of that length (6 data bytes)
//   Consecutive     0x2N   N = sequence number 0..15 (wraps)
//   Flow Control    0x3S   S = 0 CTS / 1 Wait / 2 Overflow,
//                          byte 1 = BlockSize, byte 2 = SeparationTime
//
// The transport is deliberately TYPE-AGNOSTIC: the reassembled message's
// byte 0 is the protocol's msg_type (CMD/ACK/NACK/...) and byte 1 the opcode,
// but that is the dispatcher's business, not ours. We only move bytes.
//
// Scope (mirrors the bootloader):
//   - RX reassembly of SF and FF+CF, bounded by MaxMsg.
//   - Receiver answers FF with FC(CTS, BS=0, STmin=0) -- no real backpressure.
//   - Single reassembly at a time, bound to one peer.
//   - Total reassembly timeout (TimeoutMs), collapsing ISO 15765's N_Bs/N_Cr.
//   - TX segmentation as an iterator over caller-owned bytes.
// Not supported (same as the bootloader): the 32-bit escape FF (>4095 B),
// parallel reassemblies, and sender-side FC handling (received FCs ignored).

#pragma once

#include <cstddef>
#include <cstdint>

namespace ams::isotp {

// ---------------------------------------------------------------------------
// Wire constants -- mirror of bl_isotp.h. Do not "tidy" these values.
// ---------------------------------------------------------------------------
inline constexpr std::uint16_t MaxMsg    = 1024u;  // BL_ISOTP_MAX_MSG
inline constexpr std::uint32_t TimeoutMs = 1000u;  // BL_ISOTP_TIMEOUT_MS

inline constexpr std::uint8_t PciMaskHi = 0xF0u;
inline constexpr std::uint8_t PciMaskLo = 0x0Fu;

inline constexpr std::uint8_t PciSf = 0x00u;
inline constexpr std::uint8_t PciFf = 0x10u;
inline constexpr std::uint8_t PciCf = 0x20u;
inline constexpr std::uint8_t PciFc = 0x30u;

inline constexpr std::uint8_t SfMaxLen = 7u;   // bytes 1..7 of an 8-byte frame
inline constexpr std::uint8_t FfDataLen = 6u;  // FF spends bytes 0,1 on PCI+len
inline constexpr std::uint8_t CfDataLen = 7u;

inline constexpr std::uint8_t FcCts      = 0x00u;
inline constexpr std::uint8_t FcWait     = 0x01u;
inline constexpr std::uint8_t FcOverflow = 0x02u;

inline constexpr std::uint8_t FcBsDefault    = 0x00u;  // every CF, no further FC
inline constexpr std::uint8_t FcSTminDefault = 0x00u;  // no inter-CF gap

// Every frame we emit is a full 8 bytes, zero-padded. Classic-CAN ISO-TP
// stacks commonly assume DLC 8, and the bootloader's receiver tolerates both,
// so padding is the interoperable choice.
inline constexpr std::uint8_t FrameLen = 8u;

// ---------------------------------------------------------------------------
// RX status -- mirror of bl_isotp_rx_status_t.
// ---------------------------------------------------------------------------
enum class RxStatus : std::uint8_t {
    Ok           = 0,  // frame consumed, no completed message
    MsgComplete  = 1,  // data()/size() now hold a full message
    ErrBadPci    = 2,  // malformed PCI byte or length
    ErrBadSeq    = 3,  // CF arrived with the wrong sequence number
    ErrOverflow  = 4,  // declared length > MaxMsg
    ErrNoFf      = 5,  // CF arrived with no FF active (or wrong peer)
    ErrTimeout   = 6,  // reassembly timed out (from tick())
};

// ---------------------------------------------------------------------------
// Reassembler -- feed it raw 8-byte frame payloads, it hands back complete
// messages. One reassembly in flight, bound to the peer that sent the FF.
// ---------------------------------------------------------------------------
class Reassembler {
public:
    // Consume one frame payload. `dlc` is the CAN data length (1..8), `peer`
    // the sender's node id, `now_ms` a monotonic millisecond tick.
    [[nodiscard]] RxStatus feed(const std::uint8_t* frame,
                                std::uint8_t        dlc,
                                std::uint8_t        peer,
                                std::uint32_t       now_ms) noexcept {
        if (frame == nullptr || dlc == 0u) return fail(RxStatus::ErrBadPci);

        const std::uint8_t pci = frame[0];
        switch (pci & PciMaskHi) {
        case PciSf: return feed_sf(frame, dlc, pci);
        case PciFf: return feed_ff(frame, dlc, pci, peer, now_ms);
        case PciCf: return feed_cf(frame, dlc, pci, peer);
        case PciFc: return RxStatus::Ok;   // sender-side FC handling: ignored
        default:    return fail(RxStatus::ErrBadPci);
        }
    }

    // Call periodically. Aborts an in-flight reassembly that has gone quiet.
    [[nodiscard]] RxStatus tick(std::uint32_t now_ms) noexcept {
        if (state_ != State::WaitCf) return RxStatus::Ok;
        // Future-tick safe: only a genuine elapse past the deadline aborts.
        if (now_ms < deadline_ms_) return RxStatus::Ok;
        return fail(RxStatus::ErrTimeout);
    }

    // Set once an FF is accepted; the caller should emit FC(CTS) and clear it.
    [[nodiscard]] bool fc_pending() const noexcept { return fc_pending_; }
    void clear_fc_pending() noexcept { fc_pending_ = false; }

    // Valid only immediately after feed() returned MsgComplete.
    [[nodiscard]] const std::uint8_t* data() const noexcept { return buf_; }
    [[nodiscard]] std::uint16_t       size() const noexcept { return size_; }
    [[nodiscard]] std::uint8_t        peer() const noexcept { return peer_; }

    // Diagnostics for the dispatcher's error logging (mirrors the BL's
    // last_err_* fields): on ErrBadSeq, what we wanted vs what arrived.
    [[nodiscard]] std::uint8_t last_expected() const noexcept { return last_expected_; }
    [[nodiscard]] std::uint8_t last_observed() const noexcept { return last_observed_; }

    void reset() noexcept {
        state_          = State::Idle;
        total_len_      = 0;
        received_       = 0;
        next_seq_       = 0;
        fc_pending_     = false;
        last_expected_  = 0;
        last_observed_  = 0;
    }

private:
    enum class State : std::uint8_t { Idle, WaitCf };

    RxStatus fail(RxStatus s) noexcept {
        // Preserve the diagnostic fields the caller is about to read; only the
        // reassembly state is torn down.
        state_      = State::Idle;
        total_len_  = 0;
        received_   = 0;
        next_seq_   = 0;
        fc_pending_ = false;
        return s;
    }

    RxStatus feed_sf(const std::uint8_t* frame, std::uint8_t dlc,
                     std::uint8_t pci) noexcept {
        const std::uint8_t len = static_cast<std::uint8_t>(pci & PciMaskLo);
        if (len == 0u || len > SfMaxLen) return fail(RxStatus::ErrBadPci);
        if (static_cast<std::uint16_t>(len) + 1u > dlc) return fail(RxStatus::ErrBadPci);
        for (std::uint8_t i = 0; i < len; ++i) buf_[i] = frame[1u + i];
        size_  = len;
        state_ = State::Idle;
        return RxStatus::MsgComplete;
    }

    RxStatus feed_ff(const std::uint8_t* frame, std::uint8_t dlc,
                     std::uint8_t pci, std::uint8_t peer,
                     std::uint32_t now_ms) noexcept {
        if (dlc < FrameLen) return fail(RxStatus::ErrBadPci);
        const std::uint16_t total =
            static_cast<std::uint16_t>((static_cast<std::uint16_t>(pci & PciMaskLo) << 8) |
                                        frame[1]);
        // A message that fits in an SF must not arrive as an FF.
        if (total <= SfMaxLen) return fail(RxStatus::ErrBadPci);
        if (total > MaxMsg)    return fail(RxStatus::ErrOverflow);

        for (std::uint8_t i = 0; i < FfDataLen; ++i) buf_[i] = frame[2u + i];
        total_len_   = total;
        received_    = FfDataLen;
        next_seq_    = 1u;                 // CF sequence starts at 1
        peer_        = peer;
        state_       = State::WaitCf;
        deadline_ms_ = now_ms + TimeoutMs;
        fc_pending_  = true;               // caller emits FC(CTS)
        size_        = 0;
        return RxStatus::Ok;
    }

    RxStatus feed_cf(const std::uint8_t* frame, std::uint8_t dlc,
                     std::uint8_t pci, std::uint8_t peer) noexcept {
        if (state_ != State::WaitCf) return fail(RxStatus::ErrNoFf);
        // A reassembly is bound to the peer that opened it; a CF from anyone
        // else is not ours to consume.
        if (peer != peer_) return fail(RxStatus::ErrNoFf);

        const std::uint8_t seq = static_cast<std::uint8_t>(pci & PciMaskLo);
        if (seq != next_seq_) {
            last_expected_ = next_seq_;
            last_observed_ = seq;
            return fail(RxStatus::ErrBadSeq);
        }

        const std::uint16_t remaining = static_cast<std::uint16_t>(total_len_ - received_);
        std::uint16_t n = remaining < CfDataLen ? remaining : CfDataLen;
        if (n > static_cast<std::uint16_t>(dlc - 1u)) {
            n = static_cast<std::uint16_t>(dlc - 1u);
        }
        for (std::uint16_t i = 0; i < n; ++i) buf_[received_ + i] = frame[1u + i];
        received_ = static_cast<std::uint16_t>(received_ + n);
        next_seq_ = static_cast<std::uint8_t>((next_seq_ + 1u) & PciMaskLo);

        if (received_ >= total_len_) {
            size_  = total_len_;
            state_ = State::Idle;
            return RxStatus::MsgComplete;
        }
        return RxStatus::Ok;
    }

    std::uint8_t  buf_[MaxMsg] = {};
    State         state_        = State::Idle;
    std::uint8_t  peer_         = 0;
    std::uint16_t total_len_    = 0;
    std::uint16_t received_     = 0;
    std::uint16_t size_         = 0;
    std::uint8_t  next_seq_     = 0;
    std::uint32_t deadline_ms_  = 0;
    bool          fc_pending_   = false;
    std::uint8_t  last_expected_ = 0;
    std::uint8_t  last_observed_ = 0;
};

// ---------------------------------------------------------------------------
// Segmenter -- turns one logical message into successive 8-byte frame
// payloads (SF, or FF followed by CFs). The caller owns `msg` for the whole
// iteration, wraps each frame in a CAN header, and transmits it.
//
//   Segmenter seg; seg.begin(msg, len);
//   std::uint8_t f[8];
//   while (seg.next(f)) { transmit(id, f, 8); }
// ---------------------------------------------------------------------------
class Segmenter {
public:
    // Returns false if the message cannot be segmented (null, empty, > MaxMsg).
    bool begin(const std::uint8_t* msg, std::uint16_t len) noexcept {
        msg_  = msg;
        len_  = len;
        sent_ = 0;
        seq_  = 1u;
        first_ = true;
        done_  = (msg == nullptr) || (len == 0u) || (len > MaxMsg);
        return !done_;
    }

    // Fills `out` with the next 8-byte, zero-padded frame payload.
    // Returns false once the message is fully emitted.
    bool next(std::uint8_t out[FrameLen]) noexcept {
        if (done_ || out == nullptr) return false;
        for (std::uint8_t i = 0; i < FrameLen; ++i) out[i] = 0u;

        if (first_) {
            first_ = false;
            if (len_ <= SfMaxLen) {                      // Single Frame
                out[0] = static_cast<std::uint8_t>(PciSf | (len_ & PciMaskLo));
                for (std::uint16_t i = 0; i < len_; ++i) out[1u + i] = msg_[i];
                sent_ = len_;
                done_ = true;
                return true;
            }
            out[0] = static_cast<std::uint8_t>(PciFf | ((len_ >> 8) & PciMaskLo));
            out[1] = static_cast<std::uint8_t>(len_ & 0xFFu);
            for (std::uint8_t i = 0; i < FfDataLen; ++i) out[2u + i] = msg_[i];
            sent_ = FfDataLen;
            return true;
        }

        const std::uint16_t remaining = static_cast<std::uint16_t>(len_ - sent_);
        const std::uint16_t n = remaining < CfDataLen ? remaining : CfDataLen;
        out[0] = static_cast<std::uint8_t>(PciCf | (seq_ & PciMaskLo));
        for (std::uint16_t i = 0; i < n; ++i) out[1u + i] = msg_[sent_ + i];
        sent_ = static_cast<std::uint16_t>(sent_ + n);
        seq_  = static_cast<std::uint8_t>((seq_ + 1u) & PciMaskLo);
        if (sent_ >= len_) done_ = true;
        return true;
    }

    [[nodiscard]] bool done() const noexcept { return done_; }

private:
    const std::uint8_t* msg_   = nullptr;
    std::uint16_t       len_   = 0;
    std::uint16_t       sent_  = 0;
    std::uint8_t        seq_   = 1u;
    bool                first_ = true;
    bool                done_  = true;
};

// Build the FC(CTS) frame a receiver owes after accepting an FF.
inline void build_fc_cts(std::uint8_t out[FrameLen]) noexcept {
    for (std::uint8_t i = 0; i < FrameLen; ++i) out[i] = 0u;
    out[0] = static_cast<std::uint8_t>(PciFc | FcCts);
    out[1] = FcBsDefault;
    out[2] = FcSTminDefault;
}

}  // namespace ams::isotp
