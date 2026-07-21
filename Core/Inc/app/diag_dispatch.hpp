// SPDX-License-Identifier: proprietary
//
// Diag request dispatch (#406 / #439) -- the routing step between the ISO-TP
// transport and the LOGFS handlers.
//
// Pulled out of sd_logger_task.cpp so it can be tested. What lives here is the
// policy, and the policy is mostly about REFUSING things:
//
//   * anything that is not APP_CTRL is ignored outright -- the application does
//     not answer for the bootloader's namespace, even to say no. Replying to a
//     CMD-typed frame is how an app starts impersonating a bootloader.
//   * anything other than CONNECT without a live session is refused, so a stray
//     or replayed frame cannot stream the card to whoever is on the bus.
//   * an unknown opcode is NACKed rather than dropped, so a client sees
//     "unsupported" instead of a timeout it will blame on wiring.
//
// Generic over the LOGFS server so the tests can drive a fake. Pure logic: no
// HAL, no RTOS, no filesystem.

#pragma once

#include "diag_proto.hpp"

#include <cstdint>

namespace ams::diag {

template <class LogfsServer>
class Dispatcher {
public:
    Dispatcher(Session& session, LogfsServer& logfs) noexcept
        : session_(session), logfs_(logfs) {}

    // Handle one reassembled message. Returns the reply length written to
    // `out`, or 0 meaning SAY NOTHING (not "empty reply").
    [[nodiscard]] std::uint16_t handle(const std::uint8_t* msg,
                                       std::uint16_t       len,
                                       std::uint8_t        peer,
                                       std::uint32_t       now_ms,
                                       std::uint8_t*       out,
                                       std::uint16_t       cap) noexcept {
        Request req;
        if (!parse_request(msg, len, req)) return 0;   // runt -> silence
        if (!req.is_app_ctrl())            return 0;   // not ours -> silence

        switch (req.opcode) {
        case OpConnect:
            // Idempotent, and a fresh CONNECT from a different host takes over
            // rather than being refused: the previous holder is, by
            // definition, either gone or about to time out, and a pit tool
            // that cannot connect because a dead session is parked is worse
            // than one that interrupts a session nobody is driving.
            logfs_.release();
            session_.connect(peer, now_ms);
            // Body is [major, minor] of the APP diag protocol (#452): gives the
            // host version negotiation without another round trip. Distinct
            // from the bootloader's protocol version -- this describes the
            // APP_CTRL contract only.
            {
                const std::uint8_t ver[2] = { DiagProtoVersionMajor,
                                              DiagProtoVersionMinor };
                return build_ack(out, cap, req.opcode, ver, sizeof ver);
            }

        case OpDisconnect:
            logfs_.release();
            session_.disconnect();
            return build_ack(out, cap, req.opcode, nullptr, 0);

        default:
            break;
        }

        if (!session_.touch(peer, now_ms)) {
            return build_nack(out, cap, req.opcode, NackBadSession);
        }
        if (LogfsServer::owns(req.opcode)) {
            return logfs_.handle(req, out, cap);
        }
        return build_nack(out, cap, req.opcode, NackUnsupported);
    }

    // Drive the session idle-timeout. Releases any file handle held by a host
    // that walked away mid-transfer. Returns true on the tick it expired.
    bool tick(std::uint32_t now_ms) noexcept {
        if (!session_.tick(now_ms)) return false;
        logfs_.release();
        return true;
    }

private:
    Session&     session_;
    LogfsServer& logfs_;
};

}  // namespace ams::diag
