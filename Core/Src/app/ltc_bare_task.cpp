// SPDX-License-Identifier: proprietary
//
// BARE LTC6811-1 COMMS HARNESS (do-not-merge, AMS_LTC_BARE=1).
//
// Hardware bring-up tool. When AMS_LTC_BARE is set, App_InitTask calls
// ams_ltc_bare_run() instead of doing its normal work, and EVERY other
// app task (safety/FSM, relays, current, ACU, BMS service) self-exits.
// The result is a board that does exactly one thing in a tight loop:
//
//   1. refresh the watchdog
//   2. wake the isoSPI chain
//   3. RDCFGA  -> pure comms proof (no ADC); count PEC-clean ICs
//   4. ADCV + RDCVA -> real cell voltages (cells 1..3 of IC0)
//   5. broadcast both results on FDCAN1 (0x7E0 / 0x7E1)
//
// so you can iterate the isoSPI hardware and watch CAN for the first
// PEC-clean reply, with zero FSM / safety / service noise on the bus.
//
// Whole file compiles to nothing when AMS_LTC_BARE=0 (flight).

#include "ams_config.hpp"

#if AMS_LTC_BARE

#include "bootloader.hpp"
#include "ltc6811.hpp"
#include "ltc6820.hpp"
#include "watchdog.h"

#include "cmsis_os2.h"
#include "main.h"

#include <array>
#include <cstdint>
#include <cstring>

extern "C" {
extern FDCAN_HandleTypeDef hfdcan1;
extern SPI_HandleTypeDef   hspi1;
}

namespace {

// === EDIT ME ===========================================================
// Number of LTC6811-1 ICs physically on the isoSPI bus. Driven by
// config::LtcChainLength (set under AMS_LTC_BARE in ams_config.hpp to match
// the ICs on your bench). read_register_group() sizes/clocks by that same
// constant,
// so this MUST equal it or the reads self-reject on the buffer guard.
constexpr std::uint8_t kChainLen = ams::config::LtcChainLength;
// Poll cadence. Lower for a faster scope-trigger rate; keep < ~80 ms so
// the watchdog (100 ms IWDG) is always fed in time.
constexpr std::uint32_t kPollDelayMs = 10;
// Aggressive-wake parameters for a MARGINAL daisy hop. wakeup() sends the
// bare-minimum pulses with 30 us gaps -- fine for the direct first IC, too
// weak to reliably wake a *sleeping* downstream module *through* it. We
// send kWakeBursts bursts with ~1 ms gaps (> tWAKE ~400 us) so each pulse
// has time to wake an IC and propagate to the next.
constexpr std::uint8_t kWakeBursts = 6;
// =======================================================================

constexpr std::uint32_t kCanIdStatus  = 0x7E0;  // RDCFGA / PEC summary
// Per-IC cell block: kCanIdCellBase + ic*8 + group (0=A/1-3 .. 3=D/10-12),
// for up to 10 ICs. IC0 0x500.., IC9 0x548.. [ctr, decode_ok, 3x mV LE].
constexpr std::uint32_t kCanIdCellBase = 0x500;
// Per-IC temperature block: ID = kCanIdTempBase + ic*16 + (addr/3), 11 frames
// of 3 = the full 32 ADG731 mux positions, for up to 10 ICs. IC0 0x400..,
// IC1 0x410.., ... IC9 0x490.. (stride 16 = 11 frames + slack). Base 0x400
// keeps it clear of the cell IDs (0x7B./7C./7D.) at high IC counts.
constexpr std::uint32_t kCanIdTempBase = 0x400;  // 32 mux-divider mV per IC
constexpr std::uint8_t  kMuxChannels   = 32;     // full ADG731 sweep (S1..S32)

bool can_send8(std::uint32_t id, const std::uint8_t data[8]) noexcept {
    FDCAN_TxHeaderTypeDef tx = {};
    tx.Identifier          = id;
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = FDCAN_DLC_BYTES_8;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0;
    // 10 ICs burst ~110 frames/loop, far faster than the 500k bus drains, so
    // the TX FIFO overflows and later frames (higher ICs) get dropped. Wait
    // for a free slot before queueing. Bounded (~3 ms) so a bus with no ACKer
    // can't hang the harness -- it just drops that frame and moves on.
    const std::uint32_t t0 = osKernelGetTickCount();
    while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0u) {
        if ((osKernelGetTickCount() - t0) >= 3u) return false;
    }
    return HAL_FDCAN_AddMessageToTxFifoQ(
               &hfdcan1, &tx, const_cast<std::uint8_t*>(data)) == HAL_OK;
}

}  // namespace

extern "C" void ams_ltc_bare_run(void) {
    // Accept unmatched standard frames into RX FIFO0. CRITICAL: this is
    // what lets us still catch the 0x002 bootloader-reboot trigger. The
    // app's normal handler lives in AcuCanTask, which this build silences,
    // so WITHOUT this the bare image would be a dead-end -- no CAN path
    // back into the BL, recoverable only by SWD. We poll FIFO0 in the loop.
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
        FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT,
        FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_Start(&hfdcan1);

    auto& bus = ams::ltc6820::Bus::default_instance();
    bus.configure(&hspi1,
                  ams::ltc6820::CsPin{ LTC6820_CS_GPIO_Port, LTC6820_CS_Pin });

    const auto cmd_rdcfga = ams::ltc6811::pack_command(ams::ltc6811::CmdRDCFGA);
    const auto cmd_rdcva  = ams::ltc6811::pack_command(ams::ltc6811::CmdRDCVA);
    const auto cmd_rdcvb  = ams::ltc6811::pack_command(ams::ltc6811::CmdRDCVB);
    const auto cmd_rdcvc  = ams::ltc6811::pack_command(ams::ltc6811::CmdRDCVC);
    const auto cmd_rdcvd  = ams::ltc6811::pack_command(ams::ltc6811::CmdRDCVD);
    const auto cmd_adcv   = ams::ltc6811::pack_command(
        ams::ltc6811::adcv_cmd(ams::ltc6811::AdcMode::Norm7kHz, /*discharge=*/false));
    const auto cmd_adax   = ams::ltc6811::pack_command(ams::ltc6811::adax_cmd(
        ams::ltc6811::AdcMode::Norm7kHz, ams::ltc6811::AuxSel::Gpio1));
    const auto cmd_rdauxa = ams::ltc6811::pack_command(ams::ltc6811::CmdRDAUXA);

    std::uint8_t counter = 0;

    for (;;) {
        ams_watchdog_refresh();

        // Escape hatch: honour the 0x002#B007AD11 bootloader trigger so
        // this image is never a dead-end. request_reboot() opens relays,
        // writes the BL magic to BKP0R, and resets -- never returns.
        if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0u) {
            FDCAN_RxHeaderTypeDef rxh;
            std::uint8_t          rxd[8] = {};
            if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxh, rxd) == HAL_OK &&
                rxh.Identifier == ams::config::BlBootReqCanId &&
                std::memcmp(rxd, ams::config::BlBootReqPayload,
                            ams::config::BlBootReqDlc) == 0) {
                ams::Bootloader::request_reboot(ams::config::JumpReason::CanTrigger);
            }
        }

        // Aggressive wake: several pulse bursts spaced ~1 ms (> tWAKE) so a
        // sleeping downstream module gets many chances to wake *through* the
        // upstream IC. One burst suffices for a healthy chain; the extras
        // cost only a few ms and are the firmware lever for a marginal hop.
        for (std::uint8_t w = 0; w < kWakeBursts; ++w) {
            bus.wakeup();
            osDelay(1);
        }

        // Configure every IC for AUX/temperature reads: GPIO pull-downs OFF
        // (so the muxed NTC voltage can reach GPIO1) + REFON on. Re-sent each
        // loop in case the chain slept. Harmless to the cell reads.
        {
            const auto cfga = ams::ltc6811::pack_cfga_payload(0);
            std::uint8_t cfg_payload[kChainLen][6];
            for (std::uint8_t ic = 0; ic < kChainLen; ++ic)
                for (std::uint8_t k = 0; k < 6u; ++k) cfg_payload[ic][k] = cfga[k];
            bus.write_chain_command(ams::ltc6811::CmdWRCFGA, cfg_payload);
        }

        // --- (1) RDCFGA: comms proof, no ADC -----------------------------
        std::uint8_t cfg[8u * kChainLen] = {};
        const bool rd_ok = bus.read_register_group(cmd_rdcfga.data(),
                                                   cfg, sizeof(cfg));
        const std::uint8_t good = rd_ok
            ? ams::ltc6811::count_pec_valid_segments(cfg, sizeof(cfg), kChainLen)
            : 0u;

        const std::uint8_t status[8] = {
            counter,                                    // [0] rolling loop count
            kChainLen,                                  // [1] expected ICs
            good,                                       // [2] PEC-clean ICs  <-- watch this
            static_cast<std::uint8_t>(rd_ok ? 1u : 0u), // [3] SPI transfer ok
            cfg[0], cfg[1], cfg[2], cfg[3],             // [4..7] IC0 CFGR bytes 0..3
        };
        can_send8(kCanIdStatus, status);

        // RAW IC1 (module2) RDCFGA segment (6 data + 2 PEC) for the
        // 0xFF-vs-valid-PEC-zeros diagnosis: FF.. = LTC6820 got no return
        // (null=logic1 -> wiring/return-path); 00.. with a valid trailing
        // PEC = module2 IS answering (firmware count/config). 0x7BF.
        if (kChainLen >= 2u) {
            can_send8(0x7BFu, &cfg[8]);
        }

        // --- (2) ADCV + read ALL 4 cell-voltage groups, EVERY IC ---------
        // Per IC, per group: [ctr, decode-ok, cellX..Z mV LE] on
        // kCanIdCellBase + ic*8 + group, for all 10 ICs (IC0 0x500.., IC9
        // 0x548..). decode_cell_voltage_group decodes one 8-byte IC segment.
        bus.wakeup();  // re-wake right before ADCV so a marginal downstream
                       // IC is freshly READY (REFUP) for the conversion+read
        bus.send_command(cmd_adcv.data());
        osDelay(3);  // > 2.3 ms 7 kHz all-cell conversion

        const auto read_send_group =
            [&](const std::array<std::uint8_t, 4>& cmd, std::uint8_t group) {
                std::uint8_t cv[8u * kChainLen] = {};
                bus.read_register_group(cmd.data(), cv, sizeof(cv));
                for (std::uint8_t ic = 0; ic < kChainLen && ic < 10u; ++ic) {
                    std::array<std::uint16_t, 3> mv{};
                    const bool dec_ok =
                        ams::ltc6811::decode_cell_voltage_group(&cv[ic * 8u], mv);
                    const std::uint8_t f[8] = {
                        counter,
                        static_cast<std::uint8_t>(dec_ok ? 1u : 0u),
                        static_cast<std::uint8_t>(mv[0] & 0xFFu),
                        static_cast<std::uint8_t>(mv[0] >> 8),
                        static_cast<std::uint8_t>(mv[1] & 0xFFu),
                        static_cast<std::uint8_t>(mv[1] >> 8),
                        static_cast<std::uint8_t>(mv[2] & 0xFFu),
                        static_cast<std::uint8_t>(mv[2] >> 8),
                    };
                    can_send8(kCanIdCellBase + ic * 8u + group, f);
                }
            };
        read_send_group(cmd_rdcva, 0u);  // A: cells 1-3
        read_send_group(cmd_rdcvb, 1u);  // B: cells 4-6
        read_send_group(cmd_rdcvc, 2u);  // C: cells 7-9
        read_send_group(cmd_rdcvd, 3u);  // D: cells 10-12

        // --- (3) FULL ADG731 mux map (all 32 outputs), EVERY IC ----------
        // Sweep ALL 32 mux addresses (S1..S32 = addr 0..31), not just the 20
        // wired NTC channels, so the bench maps every output. Per address:
        // WRCOMM(select) broadcast to all ICs -> STCOMM -> settle -> ADAX(GPIO1)
        // -> RDAUXA. One sweep drives every IC's mux in lockstep; a single
        // RDAUXA reads AUX1 (temps_mux = muxed NTC divider, NTC + 6.8k pull-up
        // to Vref2) for ALL ICs. Populated NTC addresses read a real divider
        // voltage; unused / NC addresses float to ~Vref2. Flight NTC map =
        // config::Adg731ChannelMap (addr 0-9 -> NTC_1..10, 16-25 -> NTC_11..20).
        // Per-IC block: kCanIdTempBase + ic*16 + (addr/3), [ctr, base, 3x mV LE].
        // static: [10][32] u16 = 640 B, too big for the task stack.
        static std::uint16_t temp_mv[kChainLen][kMuxChannels];
        std::memset(temp_mv, 0, sizeof(temp_mv));
        for (std::uint8_t addr = 0; addr < kMuxChannels; ++addr) {
            ams_watchdog_refresh();  // sweep is long -- keep the dog fed
            const auto sel = ams::ltc6811::pack_adg731_select(addr);  // raw addr 0..31
            std::uint8_t mux_payload[kChainLen][6];
            for (std::uint8_t ic = 0; ic < kChainLen; ++ic)
                for (std::uint8_t k = 0; k < 6u; ++k) mux_payload[ic][k] = sel[k];
            if (!bus.write_chain_command(ams::ltc6811::CmdWRCOMM, mux_payload)) continue;
            if (!bus.stcomm()) continue;
            osDelay(1);  // mux transition + NTC-divider settle
            bus.send_command(cmd_adax.data());
            osDelay(ams::config::AdaxSettleMs);
            std::uint8_t aux[8u * kChainLen] = {};
            if (!bus.read_register_group(cmd_rdauxa.data(), aux, sizeof(aux))) continue;
            for (std::uint8_t ic = 0; ic < kChainLen; ++ic) {
                std::array<std::uint16_t, 3> av{};
                if (ams::ltc6811::decode_aux_voltage_group(&aux[ic * 8u], av))
                    temp_mv[ic][addr] = av[0];  // AUX1 = temps_mux for this IC
            }
        }
        for (std::uint8_t ic = 0; ic < kChainLen && ic < 10u; ++ic) {
            ams_watchdog_refresh();  // paced sends can take a few ms per IC
            for (std::uint8_t base = 0; base < kMuxChannels; base += 3) {
                std::uint8_t tf[8] = { counter, base, 0u, 0u, 0u, 0u, 0u, 0u };
                for (std::uint8_t j = 0; j < 3u && (base + j) < kMuxChannels; ++j) {
                    tf[2 + j * 2] = static_cast<std::uint8_t>(temp_mv[ic][base + j] & 0xFFu);
                    tf[3 + j * 2] = static_cast<std::uint8_t>(temp_mv[ic][base + j] >> 8);
                }
                can_send8(kCanIdTempBase + ic * 16u + (base / 3u), tf);
            }
        }

        // PEC self-test: echo the EXACT command+PEC bytes the firmware
        // computed (pack_command output) so the bench can verify the STM's
        // on-wire PEC against an independent reference. Expected:
        //   0x7E2 = RDCFGA(00 02 2B 0A) + ADCV(03 60 F4 6C)
        //   0x7E3 = RDCVA (00 04 07 C2) + pad
        const std::uint8_t echo_cmds[8] = {
            cmd_rdcfga[0], cmd_rdcfga[1], cmd_rdcfga[2], cmd_rdcfga[3],
            cmd_adcv[0],   cmd_adcv[1],   cmd_adcv[2],   cmd_adcv[3],
        };
        can_send8(0x7E2, echo_cmds);
        const std::uint8_t echo_rdcva[8] = {
            cmd_rdcva[0], cmd_rdcva[1], cmd_rdcva[2], cmd_rdcva[3],
            0u, 0u, 0u, 0u,
        };
        can_send8(0x7E3, echo_rdcva);

        ++counter;
        osDelay(kPollDelayMs);
    }
}

#endif  // AMS_LTC_BARE
