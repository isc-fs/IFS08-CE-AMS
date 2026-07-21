// SPDX-License-Identifier: proprietary
//
// Tests for ams::diag::Dispatcher (#406 / #439).
//
// This layer is almost entirely about refusal, so that is what is tested: what
// happens to a frame with no session, a frame in the bootloader's namespace, a
// second host barging in, and a host that walks away mid-transfer holding a
// file open. The happy path is one test; the rest is everything else.

#include "diag_dispatch.hpp"

#include "unity.h"

#include <cstdint>

namespace {

using namespace ams;

// Stand-in for logfs::Server -- records what the dispatcher asked of it.
class FakeLogfs {
public:
    int          releases = 0;
    int          handled  = 0;
    std::uint8_t last_op  = 0;

    static bool owns(std::uint8_t opcode) noexcept {
        return opcode >= diag::OpLogfsList && opcode <= diag::OpLogfsClose;
    }

    std::uint16_t handle(const diag::Request& req, std::uint8_t* out,
                         std::uint16_t cap) noexcept {
        ++handled;
        last_op = req.opcode;
        return diag::build_ack(out, cap, req.opcode, nullptr, 0);
    }

    void release() noexcept { ++releases; }
};

constexpr std::uint8_t kHost  = 2u;
constexpr std::uint8_t kOther = 7u;

std::uint8_t g_out[64];

struct Fixture {
    diag::Session               session;
    FakeLogfs                   logfs;
    diag::Dispatcher<FakeLogfs> disp{session, logfs};

    // Default to Start: log extraction is only permitted with the car stopped
    // and the TS off (#449), so every pre-existing test runs in that state.
    fsm::State state = fsm::State::Start;

    std::uint16_t send(std::uint8_t type, std::uint8_t opcode, std::uint8_t peer,
                       std::uint32_t now) {
        const std::uint8_t msg[2] = {type, opcode};
        return disp.handle(msg, sizeof msg, peer, now, state, g_out, sizeof g_out);
    }
    std::uint16_t app(std::uint8_t opcode, std::uint8_t peer, std::uint32_t now) {
        return send(static_cast<std::uint8_t>(diag::MsgType::AppCtrl), opcode, peer, now);
    }
    void connect(std::uint32_t now = 1000u) {
        (void)app(diag::OpConnect, kHost, now);
    }
};

}  // namespace

// --- silence (not refusal) --------------------------------------------------

// A CMD-typed frame belongs to the bootloader's namespace. Answering it -- even
// with a NACK -- is the app impersonating a bootloader.
extern "C" void test_dispatch_ignores_bootloader_namespace(void) {
    Fixture f;
    TEST_ASSERT_EQUAL_UINT16(0u, f.send(0x00u, diag::OpLogfsList, kHost, 1000u));
    TEST_ASSERT_EQUAL_UINT16(0u, f.send(0x03u, 0x04u, kHost, 1000u));
    TEST_ASSERT_EQUAL_INT(0, f.logfs.handled);
}

extern "C" void test_dispatch_ignores_runt_message(void) {
    Fixture f;
    const std::uint8_t one[1] = {0x06u};
    TEST_ASSERT_EQUAL_UINT16(0u, f.disp.handle(one, 1u, kHost, 1000u, f.state, g_out, sizeof g_out));
    TEST_ASSERT_EQUAL_UINT16(0u, f.disp.handle(nullptr, 8u, kHost, 1000u, f.state, g_out, sizeof g_out));
}

// --- session gating ---------------------------------------------------------

// The failure this prevents: a stray or replayed frame streaming the card to
// whoever happens to be on the bus.
extern "C" void test_dispatch_refuses_logfs_without_session(void) {
    Fixture f;
    const std::uint16_t n = f.app(diag::OpLogfsList, kHost, 1000u);
    TEST_ASSERT_EQUAL_UINT16(3u, n);
    TEST_ASSERT_EQUAL_HEX8(0x02, g_out[0]);
    TEST_ASSERT_EQUAL_HEX8(diag::NackBadSession, g_out[2]);
    TEST_ASSERT_EQUAL_INT(0, f.logfs.handled);
}

extern "C" void test_dispatch_connect_then_logfs_reaches_server(void) {
    Fixture f;
    const std::uint16_t c = f.app(diag::OpConnect, kHost, 1000u);
    // [Ack][opcode][major][minor] -- the version body is wire (#452).
    TEST_ASSERT_EQUAL_UINT16(4u, c);
    TEST_ASSERT_EQUAL_HEX8(0x01, g_out[0]);
    TEST_ASSERT_EQUAL_UINT8(diag::DiagProtoVersionMajor, g_out[2]);
    TEST_ASSERT_EQUAL_UINT8(diag::DiagProtoVersionMinor, g_out[3]);

    (void)f.app(diag::OpLogfsRead, kHost, 1100u);
    TEST_ASSERT_EQUAL_INT(1, f.logfs.handled);
    TEST_ASSERT_EQUAL_HEX8(diag::OpLogfsRead, f.logfs.last_op);
}

// A second host must not ride the first host's session.
extern "C" void test_dispatch_refuses_logfs_from_other_peer(void) {
    Fixture f;
    f.connect();
    (void)f.app(diag::OpLogfsList, kOther, 1100u);
    TEST_ASSERT_EQUAL_HEX8(0x02, g_out[0]);
    TEST_ASSERT_EQUAL_HEX8(diag::NackBadSession, g_out[2]);
    TEST_ASSERT_EQUAL_INT(0, f.logfs.handled);
}

// ...but it may take over by connecting, and that must drop the old file
// handle -- otherwise a dead session parks a handle a live tool cannot reclaim.
extern "C" void test_dispatch_new_connect_takes_over_and_releases(void) {
    Fixture f;
    f.connect();
    (void)f.app(diag::OpLogfsOpen, kHost, 1100u);
    const int before = f.logfs.releases;

    (void)f.app(diag::OpConnect, kOther, 1200u);
    TEST_ASSERT_EQUAL_INT(before + 1, f.logfs.releases);

    (void)f.app(diag::OpLogfsRead, kOther, 1300u);   // new owner works
    TEST_ASSERT_EQUAL_HEX8(0x01, g_out[0]);
    (void)f.app(diag::OpLogfsRead, kHost, 1300u);    // old owner does not
    TEST_ASSERT_EQUAL_HEX8(diag::NackBadSession, g_out[2]);
}

extern "C" void test_dispatch_connect_is_idempotent(void) {
    Fixture f;
    f.connect(1000u);
    const std::uint16_t n = f.app(diag::OpConnect, kHost, 1100u);
    TEST_ASSERT_EQUAL_UINT16(4u, n);
    TEST_ASSERT_EQUAL_HEX8(0x01, g_out[0]);
    (void)f.app(diag::OpLogfsList, kHost, 1200u);
    TEST_ASSERT_EQUAL_INT(1, f.logfs.handled);
}

// --- disconnect / timeout ---------------------------------------------------

extern "C" void test_dispatch_disconnect_releases_and_closes_session(void) {
    Fixture f;
    f.connect();
    (void)f.app(diag::OpLogfsOpen, kHost, 1100u);
    const int before = f.logfs.releases;

    const std::uint16_t n = f.app(diag::OpDisconnect, kHost, 1200u);
    TEST_ASSERT_EQUAL_UINT16(2u, n);
    TEST_ASSERT_EQUAL_HEX8(0x01, g_out[0]);
    TEST_ASSERT_EQUAL_INT(before + 1, f.logfs.releases);

    (void)f.app(diag::OpLogfsList, kHost, 1300u);
    TEST_ASSERT_EQUAL_HEX8(diag::NackBadSession, g_out[2]);
}

// DISCONNECT is accepted with no session so a confused client can always
// return to a known state.
extern "C" void test_dispatch_disconnect_without_session_still_acks(void) {
    Fixture f;
    const std::uint16_t n = f.app(diag::OpDisconnect, kHost, 1000u);
    TEST_ASSERT_EQUAL_UINT16(2u, n);
    TEST_ASSERT_EQUAL_HEX8(0x01, g_out[0]);
}

// An operator who unplugs mid-pull must not pin a file handle forever.
extern "C" void test_dispatch_tick_expires_session_and_releases(void) {
    Fixture f;
    f.connect(1000u);
    (void)f.app(diag::OpLogfsOpen, kHost, 1000u);
    const int before = f.logfs.releases;

    TEST_ASSERT_FALSE(f.disp.tick(1000u + diag::Session::IdleTimeoutMs - 1u));
    TEST_ASSERT_EQUAL_INT(before, f.logfs.releases);

    TEST_ASSERT_TRUE(f.disp.tick(1000u + diag::Session::IdleTimeoutMs));
    TEST_ASSERT_EQUAL_INT(before + 1, f.logfs.releases);

    (void)f.app(diag::OpLogfsList, kHost, 99999u);
    TEST_ASSERT_EQUAL_HEX8(diag::NackBadSession, g_out[2]);
}

// Traffic keeps a multi-minute transfer alive -- the whole point of a 4 MiB
// pull is that it outlasts the idle timeout many times over.
extern "C" void test_dispatch_traffic_keeps_long_transfer_alive(void) {
    Fixture f;
    f.connect(0u);
    for (std::uint32_t t = 5000u; t <= 200000u; t += 5000u) {
        (void)f.app(diag::OpLogfsRead, kHost, t);
        TEST_ASSERT_EQUAL_HEX8(0x01, g_out[0]);
        TEST_ASSERT_FALSE(f.disp.tick(t));
    }
    TEST_ASSERT_EQUAL_INT(40, f.logfs.handled);
}

// --- unknown opcodes --------------------------------------------------------

// NACK, not silence: a client should see "unsupported", not a timeout it will
// blame on the cable.
extern "C" void test_dispatch_unknown_opcode_nacks(void) {
    Fixture f;
    f.connect();
    (void)f.app(0x7Fu, kHost, 1100u);
    TEST_ASSERT_EQUAL_HEX8(0x02, g_out[0]);
    TEST_ASSERT_EQUAL_HEX8(diag::NackUnsupported, g_out[2]);
    TEST_ASSERT_EQUAL_INT(0, f.logfs.handled);
}

// v1 is read-only: DELETE (0x26) is outside the server's range and must be
// refused rather than quietly accepted.
extern "C" void test_dispatch_delete_opcode_is_unsupported(void) {
    Fixture f;
    f.connect();
    (void)f.app(0x26u, kHost, 1100u);
    TEST_ASSERT_EQUAL_HEX8(diag::NackUnsupported, g_out[2]);
    TEST_ASSERT_EQUAL_INT(0, f.logfs.handled);
}

// Session state must gate BEFORE opcode validity, so probing for supported
// opcodes without connecting tells an attacker nothing.
extern "C" void test_dispatch_session_check_precedes_opcode_check(void) {
    Fixture f;
    (void)f.app(0x7Fu, kHost, 1000u);
    TEST_ASSERT_EQUAL_HEX8(diag::NackBadSession, g_out[2]);
}

// The CONNECT ACK carries the APP diag protocol version so the host can
// negotiate without another round trip (#452). Pinned literally: the host
// compares majors, so a silent bump would strand it.
extern "C" void test_dispatch_connect_ack_carries_protocol_version(void) {
    Fixture f;
    const std::uint16_t n = f.app(diag::OpConnect, kHost, 1000u);
    TEST_ASSERT_EQUAL_UINT16(4u, n);
    TEST_ASSERT_EQUAL_HEX8(0x01, g_out[0]);
    TEST_ASSERT_EQUAL_HEX8(diag::OpConnect, g_out[1]);
    TEST_ASSERT_EQUAL_UINT8(1u, g_out[2]);   // major
    TEST_ASSERT_EQUAL_UINT8(0u, g_out[3]);   // minor
}

// --- vehicle-state gate (#449) ----------------------------------------------
//
// Operational rule: log extraction runs ONLY with the car stopped and the
// tractive system off. Beyond "don't invite a multi-minute bus-heavy operation
// while the car can move", there is a concrete mechanism: our diag TX id
// (0x010 + NodeID) is numerically LOWER than the VCU heartbeat 0x100, so every
// queued LOGFS frame WINS arbitration against the heartbeat the FSM depends
// on -- and VcuStale latches Error, which opens the contactors.

extern "C" void test_dispatch_refuses_logfs_with_ts_live(void) {
    for (auto st : { fsm::State::Precharge, fsm::State::Transition,
                     fsm::State::Run, fsm::State::Charge }) {
        Fixture f;
        f.state = fsm::State::Start;
        f.connect();                       // connect while still permitted
        f.state = st;                      // ...then the car comes alive

        (void)f.app(diag::OpLogfsList, kHost, 1100u);
        TEST_ASSERT_EQUAL_HEX8_MESSAGE(0x02, g_out[0], "must NACK with TS live");
        TEST_ASSERT_EQUAL_HEX8_MESSAGE(diag::NackVehicleState, g_out[2],
                                       "must say WHY: vehicle state");
        TEST_ASSERT_EQUAL_INT_MESSAGE(0, f.logfs.handled,
                                      "must not reach the card with TS live");
    }
}

extern "C" void test_dispatch_allows_logfs_in_start(void) {
    Fixture f;
    f.state = fsm::State::Start;
    f.connect();
    (void)f.app(diag::OpLogfsList, kHost, 1100u);
    TEST_ASSERT_EQUAL_HEX8(0x01, g_out[0]);
    TEST_ASSERT_EQUAL_INT(1, f.logfs.handled);
}

// Every LOGFS opcode is gated, not just the bulk ones -- FINALIZE especially,
// since sealing the active log mid-run is exactly what an impatient operator
// would try while the car is live.
extern "C" void test_dispatch_gate_covers_every_logfs_opcode(void) {
    const std::uint8_t ops[] = {diag::OpLogfsList,  diag::OpLogfsOpen,
                                diag::OpLogfsRead,  diag::OpLogfsCrc,
                                diag::OpLogfsClose, diag::OpLogfsFinalize};
    for (std::uint8_t op : ops) {
        Fixture f;
        f.state = fsm::State::Start;
        f.connect();
        f.state = fsm::State::Run;
        (void)f.app(op, kHost, 1100u);
        TEST_ASSERT_EQUAL_HEX8(diag::NackVehicleState, g_out[2]);
    }
}

// CONNECT/DISCONNECT stay permitted in any state: they cost nothing on the bus
// and let the host discover WHY it is being refused rather than guessing at a
// silent node.
extern "C" void test_dispatch_session_ops_work_in_any_state(void) {
    for (auto st : { fsm::State::Run, fsm::State::Charge, fsm::State::Error }) {
        Fixture f;
        f.state = st;
        TEST_ASSERT_EQUAL_UINT16(4u, f.app(diag::OpConnect, kHost, 1000u));
        TEST_ASSERT_EQUAL_HEX8(0x01, g_out[0]);
        TEST_ASSERT_EQUAL_UINT16(2u, f.app(diag::OpDisconnect, kHost, 1100u));
        TEST_ASSERT_EQUAL_HEX8(0x01, g_out[0]);
    }
}

// A pull in progress when the car comes alive must drop its handle, not just
// stop serving -- otherwise the host holds a handle across a state change.
extern "C" void test_dispatch_releases_handle_when_car_goes_live(void) {
    Fixture f;
    f.state = fsm::State::Start;
    f.connect();
    (void)f.app(diag::OpLogfsOpen, kHost, 1100u);
    const int before = f.logfs.releases;

    f.state = fsm::State::Run;
    (void)f.app(diag::OpLogfsRead, kHost, 1200u);
    TEST_ASSERT_EQUAL_HEX8(diag::NackVehicleState, g_out[2]);
    TEST_ASSERT_EQUAL_INT_MESSAGE(before + 1, f.logfs.releases,
                                  "handle must be released when the car goes live");
}

// Error has the contactors open and the TS down, and is where the car sits
// after the fault whose log an operator most wants (#448). It is currently NOT
// permitted -- pinned so widening it is a deliberate act, not a drift.
extern "C" void test_dispatch_error_state_currently_refused(void) {
    Fixture f;
    f.state = fsm::State::Start;
    f.connect();
    f.state = fsm::State::Error;
    (void)f.app(diag::OpLogfsList, kHost, 1100u);
    TEST_ASSERT_EQUAL_HEX8_MESSAGE(diag::NackVehicleState, g_out[2],
                                   "Error is not currently a permitted state");
}
