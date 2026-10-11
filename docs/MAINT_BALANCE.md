# Maintenance-balancing build (`AMS_MAINT_BALANCE`)

A firmware variant for rebalancing the accumulator in the pit when part of the
BMS cannot be used: the last module (LTCs 8 and 9, module 4) may be unplugged,
and module 0's temperature sensors are broken. **It is not a flight image.**
Flash the release image back when the maintenance is finished.

## What it changes, and what it does not

**Unchanged:** every safety predicate, the FSM, the relay driver, `AMS_OK` and the
`ErrorLatch`. With module 4 missing (`BmsModuleOffline`) or module 0's sensors
open (`TempSensorDisconnected`), the AMS latches **Error**, keeps the AIRs open
and holds `AMS_OK` low, exactly as the release does. The car cannot precharge,
drive or charge on this image while either condition holds.

**Changed:** only what `BmsPollTask` writes to the LTC discharge (DCC) bits. The
policy is `balance::maint::compute_mask` in
[`balance_controller.hpp`](../Core/Inc/app/balance_controller.hpp); the
module-0 setting is in [`maint_config.hpp`](../Core/Inc/app/maint_config.hpp).
Balancing runs only when **all** of these hold:

| Gate | Rule |
|---|---|
| Operator | WarioCharger is sending **On** (`0x103` `"BALN"`). Auto never runs here; Off, or 5 s without a frame, stops everything. |
| FSM | **Start** or **Error** only — the AIRs are open in both. |
| Modules | Only modules **answering right now** are read, used for the floor, or balanced. An unplugged module's old or boot-seed values are ignored. `0x104` can still disable modules. |
| Cell data | On the answering modules: no open wire, no displaced tap, every cell inside `CellUnderVoltageMv`..`CellOverVoltageMv`. Re-checked every 800 ms from live data, because the latched fault reason only records the *first* fault. |
| Temperatures | Module 0's sensors are **ignored**. On the other answering modules: at least `BalanceMinValidTempCh` valid channels, no sensor lost, hottest ≤ `BalanceTempMax` (50 °C). |
| Limits | **Uncapped** by default: every cell more than 50 mV above the floor bleeds at once, neighbours included. `-DAMS_MAINT_BALANCE_CAPPED=ON` restores the flight limits (at most 8 per module, never two adjacent). |
| Readback | After each discharge write the configuration registers are read back. If a discharge bit is found on an IC it was not written for, balancing stops **until the next reboot**. If an IC that should discharge does not answer, discharge is turned off for that window. |

## Known gaps — read before using

- **Uncapped means ~12 W per board, ~62 W in the pack** (0.65 W per cell through
  23.5 ohm). The flight cap of 8 rests on one pad measured at ~71 C on an open
  bench; nothing has been measured at 19 per module or in a closed box, and no
  sensor reads board temperature. Keep the lid off or the box ventilated and
  watch the boards with an IR camera.
- **Module 0 has no firmware thermal protection.** Its cells can be balanced,
  but heat there is invisible to the 50 °C lockout. Watch module 0 with an IR
  camera or a thermocouple, or exclude it with `0x104` (bit 0 clear).
- **Which IC keeps which payload in a short chain has not been verified on
  hardware.** The readback above is what protects against a wrong mapping;
  the bench check below is what proves it.
- **Readback failures are not on CAN.** If you send On and the `0x6C2`/`0x6C3`
  masks stay at zero with cells clearly imbalanced, the readback may be refusing.
  Stop and investigate before going further.
- Built and unit-tested on the host only. Bench-verify before using it on the pack.

## Build

```bash
cmake -B build-maint-release -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Release -DAMS_MAINT_BALANCE=ON
cmake --build build-maint-release
python3 scripts/check_flash_layout.py build-maint-release/AMS.elf
```

To step back to the flight discharge limits (8 per module, no two neighbours)
while keeping the rest of the maintenance policy, add
`-DAMS_MAINT_BALANCE_CAPPED=ON` to the configure line.

CMake prints a warning when the flag is on. The image is identified on the bus
by the git hash in `0x6C6` `PIT_fw_id`: a hash from the maintenance branch means
this image is on the board.

## Procedure

1. **Bench first.** Flash over the CAN bootloader and confirm on `0x6C6` that it
   is the maintenance hash. With module 4 unplugged, expect Error with
   `BmsModuleOffline`. That is correct.
2. **Prove the IC mapping.** Send On with `0x104` enabling **one** module at a
   time. Check that `0x6C2`/`0x6C3` show cells only in that module and that the
   balance resistors heating (IR) are on **that** module's board. Repeat for each
   connected module.
3. **Balance.** Keep WarioCharger sending On. Watch module 0's temperature by
   other means for as long as it is enabled.
4. **Finish.** Flash the release image. Do a full LV power-cycle, which clears the
   sticky `ErrorLatch` (the backup domain has no VBAT). The release will latch
   Error again until module 4 is reconnected and module 0's sensors are repaired.
