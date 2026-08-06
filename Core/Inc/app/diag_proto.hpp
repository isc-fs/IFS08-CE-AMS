// SPDX-License-Identifier: proprietary
//
// Application-side diagnostic command protocol -- constants + framing.
//
// This is the layer above ams::isotp: the transport moves opaque bytes, this
// file says what those bytes MEAN. A reassembled message is
//
//     [0] msg_type   (CMD / ACK / NACK / APP_CTRL...)
//     [1] opcode     (or, in an ACK/NACK, the opcode being answered)
//     [2..] payload
//
// which is the bootloader's framing (stm32-can-bootloader `bl_proto.c`
// send_message/send_ack/send_nack) and what the host already speaks.
//
// OPCODE NAMESPACE OWNERSHIP -- the reason for MsgType::AppCtrl
// -------------------------------------------------------------
// LOGFS commands arrive as msg_type APP_CTRL (0x06), NOT CMD (0x00).
// Under CMD the application and the bootloader would share ONE opcode
// registry, and 0x2x is bootloader-adjacent (it reserves 0x20 for a future
// CMD_MEM_READ). Riding APP_CTRL -- which the bootloader defines as
// "application-level traffic; BL silently drops, app firmware handles" --
// makes the opcode space below entirely ours, so it can never collide with a
// future bootloader opcode.
//
// Consequence to be aware of: a LOGFS command sent while the node sits in the
// BOOTLOADER is silently dropped (no NACK), so the host sees a timeout. Hosts
// are expected to disambiguate by probing with a real bootloader command
// (DISCOVER/GET_FW_INFO under CMD), which the BL does answer.
//
// Responses reuse the bootloader's ACK/NACK types and NACK numbering: only one
// firmware is ever running, so a response is unambiguous, and sharing the NACK
// space keeps one coherent diagnostic vocabulary across BL and app.
//
// Pure logic -- no HAL, no RTOS, no allocation. Host-testable.

#pragma once

#include "isotp.hpp"

#include <cstdint>

namespace ams::diag {

// ---------------------------------------------------------------------------
// Message types -- mirror of bl_msg_type_t (bl_proto.h). KEEP IN SYNC.
// ---------------------------------------------------------------------------
enum class MsgType : std::uint8_t {
    Cmd             = 0x00u,  // host -> device, bootloader command space
    Ack             = 0x01u,  // device -> host, positive
    Nack            = 0x02u,  // device -> host, negative
    Notify          = 0x03u,  // device -> host, unsolicited
    DiscoverRequest = 0x04u,
    DiscoverReply   = 0x05u,
    AppCtrl         = 0x06u,  // host -> node, APPLICATION traffic  <-- ours
};

// ---------------------------------------------------------------------------
// Opcodes in the APP_CTRL namespace (ours -- independent of bootloader opcodes).
//
// Session opcodes deliberately reuse the bootloader's numbering so the host's
// existing session handling works unchanged; they are still scoped to APP_CTRL.
// ---------------------------------------------------------------------------
// Version of the APP_CTRL diag contract, returned in the CONNECT ACK body as
// [major, minor]. Distinct from the bootloader's protocol version: this
// describes the application's opcode/payload shapes only. Bump MAJOR on any
// incompatible wire change, MINOR on a backward-compatible addition.
inline constexpr std::uint8_t DiagProtoVersionMajor = 1u;
inline constexpr std::uint8_t DiagProtoVersionMinor = 0u;

inline constexpr std::uint8_t OpConnect    = 0x01u;
inline constexpr std::uint8_t OpDisconnect = 0x02u;

// LOGFS group.
inline constexpr std::uint8_t OpLogfsList  = 0x21u;
inline constexpr std::uint8_t OpLogfsOpen  = 0x22u;
inline constexpr std::uint8_t OpLogfsRead  = 0x23u;
inline constexpr std::uint8_t OpLogfsCrc   = 0x24u;
inline constexpr std::uint8_t OpLogfsClose = 0x25u;
// 0x26 LOGFS_DELETE intentionally NOT implemented -- v1 is read-only.

// Seal the ACTIVE log so the run that just happened becomes retrievable.
// Without it the only route to "give me the log from the run that just
// faulted" is to wait for rotation or power-cycle the node.
// Flush -> close -> rename to .CSV -> write the CRC sidecar;
// the ACK carries the sealed index so the host can go straight to OPEN.
inline constexpr std::uint8_t OpLogfsFinalize = 0x27u;

// ---------------------------------------------------------------------------
// NACK codes.
//
// Values <= 0x10 are the bootloader's (bl_proto.h) and are reused verbatim so
// the vocabulary stays coherent. LOGFS-specific codes take FREE slots -- note
// 0x08 is BL_NACK_BUSY and could NOT be used for BAD_HANDLE as originally
// proposed earlier; BAD_HANDLE moved to 0x11.
// ---------------------------------------------------------------------------
inline constexpr std::uint8_t NackOutOfBounds  = 0x02u;  // BL: OUT_OF_BOUNDS
inline constexpr std::uint8_t NackFileNotFound = 0x04u;  // free slot
inline constexpr std::uint8_t NackBadSession   = 0x06u;  // BL: BAD_SESSION
inline constexpr std::uint8_t NackBusy         = 0x08u;  // BL: BUSY
inline constexpr std::uint8_t NackBadHandle    = 0x11u;  // was 0x08 -- collided
inline constexpr std::uint8_t NackNoSdCard     = 0x12u;
inline constexpr std::uint8_t NackFsError      = 0x13u;
inline constexpr std::uint8_t NackReadError    = 0x14u;
// Refused because the vehicle is not in a state where log extraction is
// permitted -- the tractive system must be OFF and the car stopped.
// Distinct from BAD_SESSION: the host is talking correctly, the CAR is wrong.
inline constexpr std::uint8_t NackVehicleState = 0x15u;
inline constexpr std::uint8_t NackUnsupported  = 0xFEu;  // BL: UNSUPPORTED

// ---------------------------------------------------------------------------
// LOGFS wire shapes (little-endian on the wire).
// ---------------------------------------------------------------------------

// A directory entry is FIXED 22 bytes so the host can parse a LIST reply by
// stride alone: { index:u16, size:u32, mtime:u32, name[12] }.
// name is a FAT 8.3 short name, null-padded (matches LOGnnnn.CSV).
inline constexpr std::uint8_t LogfsNameLen   = 12u;
inline constexpr std::uint8_t LogfsEntryLen  = 2u + 4u + 4u + LogfsNameLen;  // 22

// Largest READ the firmware will serve. A client may request less and gets
// exactly what it asked for; a larger request is clamped (short read = EOF is
// the client's normal termination signal either way).
inline constexpr std::uint16_t LogfsMaxRead = 512u;

// Entries per LIST reply, bounded so one reply fits an ISO-TP message. The
// reply header is [type][opcode][next_cursor:u16][count] = 5 bytes; 8 is used
// here to keep a little headroom if the header ever grows.
inline constexpr std::uint8_t LogfsMaxListEntries =
    static_cast<std::uint8_t>((isotp::MaxMsg - 8u) / LogfsEntryLen);   // 46

// ---------------------------------------------------------------------------
// Little-endian helpers. Explicit rather than memcpy so the wire order is
// visible at the call site and independent of host endianness.
// ---------------------------------------------------------------------------
inline void put_u16(std::uint8_t* p, std::uint16_t v) noexcept {
    p[0] = static_cast<std::uint8_t>(v & 0xFFu);
    p[1] = static_cast<std::uint8_t>((v >> 8) & 0xFFu);
}
inline void put_u32(std::uint8_t* p, std::uint32_t v) noexcept {
    p[0] = static_cast<std::uint8_t>(v & 0xFFu);
    p[1] = static_cast<std::uint8_t>((v >> 8) & 0xFFu);
    p[2] = static_cast<std::uint8_t>((v >> 16) & 0xFFu);
    p[3] = static_cast<std::uint8_t>((v >> 24) & 0xFFu);
}
[[nodiscard]] inline std::uint16_t get_u16(const std::uint8_t* p) noexcept {
    return static_cast<std::uint16_t>(p[0] | (static_cast<std::uint16_t>(p[1]) << 8));
}
[[nodiscard]] inline std::uint32_t get_u32(const std::uint8_t* p) noexcept {
    return static_cast<std::uint32_t>(p[0]) |
           (static_cast<std::uint32_t>(p[1]) << 8) |
           (static_cast<std::uint32_t>(p[2]) << 16) |
           (static_cast<std::uint32_t>(p[3]) << 24);
}

// ---------------------------------------------------------------------------
// Request view -- a parsed, validated reassembled message.
// ---------------------------------------------------------------------------
struct Request {
    MsgType             type    = MsgType::Cmd;
    std::uint8_t        opcode  = 0;
    const std::uint8_t* args    = nullptr;  // points into the caller's buffer
    std::uint16_t       arg_len = 0;

    [[nodiscard]] bool is_app_ctrl() const noexcept { return type == MsgType::AppCtrl; }
};

// Parse a reassembled message. False if it is too short to carry
// [msg_type][opcode].
[[nodiscard]] inline bool parse_request(const std::uint8_t* msg,
                                        std::uint16_t       len,
                                        Request&            out) noexcept {
    if (msg == nullptr || len < 2u) return false;
    out.type    = static_cast<MsgType>(msg[0]);
    out.opcode  = msg[1];
    out.args    = (len > 2u) ? (msg + 2) : nullptr;
    out.arg_len = static_cast<std::uint16_t>(len - 2u);
    return true;
}

// Build an ACK: [Ack][opcode][payload...]. Returns the total message length,
// or 0 if it would not fit. `payload` may be null when `payload_len` is 0.
[[nodiscard]] inline std::uint16_t build_ack(std::uint8_t*       out,
                                             std::uint16_t       out_cap,
                                             std::uint8_t        opcode,
                                             const std::uint8_t* payload,
                                             std::uint16_t       payload_len) noexcept {
    const std::uint32_t total = 2u + static_cast<std::uint32_t>(payload_len);
    if (out == nullptr || total > out_cap || total > isotp::MaxMsg) return 0;
    out[0] = static_cast<std::uint8_t>(MsgType::Ack);
    out[1] = opcode;
    for (std::uint16_t i = 0; i < payload_len; ++i) out[2u + i] = payload[i];
    return static_cast<std::uint16_t>(total);
}

// Build a NACK: [Nack][rejected_opcode][code]. Always 3 bytes.
[[nodiscard]] inline std::uint16_t build_nack(std::uint8_t* out,
                                              std::uint16_t out_cap,
                                              std::uint8_t  rejected_opcode,
                                              std::uint8_t  code) noexcept {
    if (out == nullptr || out_cap < 3u) return 0;
    out[0] = static_cast<std::uint8_t>(MsgType::Nack);
    out[1] = rejected_opcode;
    out[2] = code;
    return 3u;
}

// ---------------------------------------------------------------------------
// Session -- CONNECT... DISCONNECT, so a stray LOGFS command can't stream the
// card to whoever happens to be on the bus. Also self-closes if the host walks
// away mid-transfer, releasing any open file handle.
// ---------------------------------------------------------------------------
class Session {
public:
    // Idle timeout: a session with no traffic for this long is dropped. Sized
    // well above the per-command round trip, but short enough that an
    // abandoned pull doesn't pin a file handle indefinitely.
    static constexpr std::uint32_t IdleTimeoutMs = 10000u;

    void connect(std::uint8_t peer, std::uint32_t now_ms) noexcept {
        open_        = true;
        peer_        = peer;
        last_seen_ms_ = now_ms;
    }
    void disconnect() noexcept { open_ = false; peer_ = 0; }

    // True iff a session is open with this peer; refreshes the idle timer.
    [[nodiscard]] bool touch(std::uint8_t peer, std::uint32_t now_ms) noexcept {
        if (!open_ || peer != peer_) return false;
        last_seen_ms_ = now_ms;
        return true;
    }

    // Call periodically. Returns true if it just expired (caller releases
    // whatever the session owned).
    [[nodiscard]] bool tick(std::uint32_t now_ms) noexcept {
        if (!open_) return false;
        if (now_ms < last_seen_ms_) return false;          // future-tick safe
        if ((now_ms - last_seen_ms_) < IdleTimeoutMs) return false;
        open_ = false;
        peer_ = 0;
        return true;
    }

    [[nodiscard]] bool         is_open() const noexcept { return open_; }
    [[nodiscard]] std::uint8_t peer()    const noexcept { return peer_; }

private:
    bool          open_         = false;
    std::uint8_t  peer_         = 0;
    std::uint32_t last_seen_ms_ = 0;
};

}  // namespace ams::diag
