// SPDX-License-Identifier: proprietary
//
// Diag request dispatch -- the routing step between the ISO-TP
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
#include "state_machine.hpp"

#include <cstdint>

namespace ams::diag {

// May log extraction run in this vehicle state?
//
// Operational rule: extraction is permitted ONLY with the car stopped
// and the tractive system OFF -- never in Run with TS live. Two reasons, and
// the second is the sharp one:
//
//   * A pull is a multi-minute, bus-heavy operation, and nobody should be
//     inviting it while the car can move.
//   * Our diag TX id (0x010 + NodeID) is NUMERICALLY LOWER than the VCU
//     heartbeat 0x100, so every queued LOGFS frame WINS arbitration against
//     the heartbeat the FSM depends on. VcuStale is 200 ms and latches Error,
//     which opens the contactors. A reply burst is only ~17-33 ms so the
//     margin is large, but the failure mode is "log pull drops the AIRs" and
//     it is not worth carrying at all when the alternative is simply not
//     extracting while the car is live.
//
// Permitted states are Start and Error. Both have the contactors OPEN and the
// tractive system DOWN -- which is the property the rule is really about; the
// state name is incidental.
//
// Error is not a grudging exception, it is the case the feature exists for.
// Its whole purpose is "grab the log from the run that just faulted", and a
// faulted car sits in Error. ErrorLatch is sticky across resets, so a
// power-cycled post-fault car boots back INTO Error -- refusing it would have
// made the primary use case reachable only by clearing the latch first.
//
// It is also the state where the arbitration hazard cannot bite: the concern
// is LOGFS out-prioritising the VCU heartbeat until VcuStale latches Error and
// opens the contactors. In Error that has already happened. There is no
// further harm available.
[[nodiscard]] inline bool logfs_allowed_in(fsm::State s) noexcept {
    return s == fsm::State::Start || s == fsm::State::Error;
}

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
                                       fsm::State          vehicle_state,
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
            // Body is [major, minor] of the APP diag protocol: gives the
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
            // Vehicle-state gate. Checked AFTER the session so a host
            // that has connected gets a specific reason rather than a generic
            // refusal, and BEFORE reaching the card so a live car never starts
            // a multi-minute transfer. CONNECT/DISCONNECT stay permitted in any
            // state: they cost nothing and let the host discover WHY it is
            // being refused instead of guessing.
            if (!logfs_allowed_in(vehicle_state)) {
                logfs_.release();   // drop any handle held from a permitted state
                return build_nack(out, cap, req.opcode, NackVehicleState);
            }
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
