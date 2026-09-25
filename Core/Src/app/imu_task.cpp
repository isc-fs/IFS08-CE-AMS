// SPDX-License-Identifier: proprietary
//
// ImuTask body -- see app/imu_task.h for the contract.
//
// Per sample: two HAL_I2C_Mem_Read_DMA transfers (accelerometer, then gyro,
// 6 bytes each), each waited on with a thread flag set from the HAL completion
// callback. The register-address byte goes out from the I2C event interrupt
// and the six data bytes arrive by DMA, so the task sleeps for the whole
// transfer. Bring-up and verification use short blocking transfers instead --
// they run once, and blocking keeps that path simple.
//
// Failure handling is deliberately flat: any failed or timed-out transfer
// resets the I2C peripheral and drops back to re-initialising the sensor after
// ImuRetryPeriodMs. There is no partial recovery to get wrong.

#include "app/imu_task.h"

#include "ams_config.hpp"
#include "app/sd_logger_task.h"
#include "imu_record.hpp"

#include "cmsis_os2.h"
#include "main.h"

extern "C" {
// Defined by CubeMX in main.c (I2C2 on PF0/PF1, RX on DMA). The handle's
// Init block and its MspInit (pins, DMA link, NVIC) are CubeMX's; this file
// only drives transfers and, on a hung bus, cycles DeInit/Init.
extern I2C_HandleTypeDef hi2c2;
}

namespace {

using namespace ams;

constexpr std::uint32_t FlagDone  = 1u << 0;
constexpr std::uint32_t FlagError = 1u << 1;

// DMA1 cannot reach DTCM, where .bss lives, so the transfer buffer sits in AXI
// SRAM (.imu_dma, NOLOAD in RAM_D1). The D-cache is off on this part, so no
// cache maintenance is needed around the transfer.
alignas(32) std::uint8_t s_dma_buf[12] __attribute__((section(".imu_dma")));

osThreadId_t s_thread = nullptr;

volatile std::uint32_t g_imu_samples     = 0;
volatile std::uint32_t g_imu_read_errors = 0;
volatile std::uint32_t g_imu_inits       = 0;
volatile ImuState      g_imu_state       = ImuState::Init;

constexpr std::uint16_t addr8(std::uint8_t addr7) noexcept {
    return static_cast<std::uint16_t>(addr7 << 1);
}

bool write_reg(std::uint8_t addr7, std::uint8_t reg, std::uint8_t val) noexcept {
    return HAL_I2C_Mem_Write(&hi2c2, addr8(addr7), reg, I2C_MEMADD_SIZE_8BIT,
                             &val, 1, config::ImuXferTimeoutMs) == HAL_OK;
}

bool read_reg(std::uint8_t addr7, std::uint8_t reg, std::uint8_t& val) noexcept {
    return HAL_I2C_Mem_Read(&hi2c2, addr8(addr7), reg, I2C_MEMADD_SIZE_8BIT,
                            &val, 1, config::ImuXferTimeoutMs) == HAL_OK;
}

bool reg_reads_back(std::uint8_t addr7, std::uint8_t reg, std::uint8_t want,
                    std::uint8_t mask = 0xFF) noexcept {
    std::uint8_t got = 0;
    return read_reg(addr7, reg, got) && (got & mask) == (want & mask);
}

// Datasheet start-up sequence, then read every configuration register back.
// ~90 ms, dominated by the settling delays; only runs at bring-up and after a
// failure.
ImuState init_sensor() noexcept {
    const std::uint8_t acc = config::ImuAccAddr7b;
    const std::uint8_t gyr = config::ImuGyrAddr7b;

    if (!reg_reads_back(acc, bmi088::AccChipIdReg, bmi088::AccChipId) ||
        !reg_reads_back(gyr, bmi088::GyrChipIdReg, bmi088::GyrChipId)) {
        return ImuState::NotFound;
    }

    if (!write_reg(acc, bmi088::AccSoftResetReg, bmi088::SoftResetCmd)) return ImuState::BusError;
    osDelay(bmi088::AccSoftResetMs);
    if (!write_reg(gyr, bmi088::GyrSoftResetReg, bmi088::SoftResetCmd)) return ImuState::BusError;
    osDelay(bmi088::GyrSoftResetMs);

    // The accelerometer powers up suspended and off.
    if (!write_reg(acc, bmi088::AccPwrConfReg, bmi088::AccPwrActive)) return ImuState::BusError;
    osDelay(bmi088::AccPwrConfMs);
    if (!write_reg(acc, bmi088::AccPwrCtrlReg, bmi088::AccPwrOn)) return ImuState::BusError;
    osDelay(bmi088::AccPwrOnMs);

    if (!write_reg(acc, bmi088::AccConfReg,      bmi088::AccConf)       ||
        !write_reg(acc, bmi088::AccRangeReg,     bmi088::AccRange6g)    ||
        !write_reg(gyr, bmi088::GyrRangeReg,     bmi088::GyrRange500dps) ||
        !write_reg(gyr, bmi088::GyrBandwidthReg, bmi088::GyrBw400Hz47Hz) ||
        !write_reg(gyr, bmi088::GyrLpm1Reg,      bmi088::GyrLpm1Normal)) {
        return ImuState::BusError;
    }

    if (!reg_reads_back(acc, bmi088::AccConfReg,      bmi088::AccConf)        ||
        !reg_reads_back(acc, bmi088::AccRangeReg,     bmi088::AccRange6g)     ||
        !reg_reads_back(gyr, bmi088::GyrRangeReg,     bmi088::GyrRange500dps) ||
        !reg_reads_back(gyr, bmi088::GyrBandwidthReg, bmi088::GyrBw400Hz47Hz,
                        bmi088::GyrBandwidthReadMask)) {
        return ImuState::BusError;
    }
    return ImuState::Running;
}

// One 6-byte register burst into the DMA buffer at `offset`.
bool read_burst_dma(std::uint8_t addr7, std::uint8_t reg, std::size_t offset) noexcept {
    (void)osThreadFlagsClear(FlagDone | FlagError);
    if (HAL_I2C_Mem_Read_DMA(&hi2c2, addr8(addr7), reg, I2C_MEMADD_SIZE_8BIT,
                             s_dma_buf + offset, 6) != HAL_OK) {
        return false;
    }
    const std::uint32_t f = osThreadFlagsWait(FlagDone | FlagError, osFlagsWaitAny,
                                              config::ImuXferTimeoutMs);
    // CMSIS returns an error code with bit 31 set on timeout.
    if ((f & 0x80000000u) != 0u) return false;
    return (f & FlagError) == 0u && (f & FlagDone) != 0u;
}

// Put the peripheral back in a known state after a failed or hung transfer.
// DeInit/Init run CubeMX's MspDeInit/MspInit, which also re-link the DMA.
void reset_bus() noexcept {
    (void)HAL_I2C_DeInit(&hi2c2);
    (void)HAL_I2C_Init(&hi2c2);
}

}  // namespace

extern "C" {

// Weak in the HAL. I2C2 is the only I2C this firmware uses, but check the
// instance anyway so a second bus added later cannot wake this task.
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* h) {
    if (h == &hi2c2 && s_thread != nullptr) (void)osThreadFlagsSet(s_thread, FlagDone);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* h) {
    if (h == &hi2c2 && s_thread != nullptr) (void)osThreadFlagsSet(s_thread, FlagError);
}

void ams_imu_task_run(void* argument) {
    (void)argument;
    s_thread = osThreadGetId();

    std::uint32_t retry_at = osKernelGetTickCount();
    std::uint32_t next     = retry_at;

    for (;;) {
        next += ams::config::ImuSamplePeriodMs;
        const std::uint32_t now = osKernelGetTickCount();
        // A slow bring-up (or a debugger halt) leaves `next` in the past; catch
        // up in one step instead of firing a burst of back-to-back reads.
        if (static_cast<std::int32_t>(next - now) <= 0) next = now + ams::config::ImuSamplePeriodMs;

        if (g_imu_state != ams::ImuState::Running) {
            if (static_cast<std::int32_t>(now - retry_at) >= 0) {
                const ams::ImuState st = init_sensor();
                g_imu_state = st;
                if (st == ams::ImuState::Running) {
                    ++g_imu_inits;
                } else {
                    reset_bus();
                    retry_at = osKernelGetTickCount() + ams::config::ImuRetryPeriodMs;
                }
            }
            (void)osDelayUntil(next);
            continue;
        }

        ams::ImuSample s{};
        s.tick_ms = now;
        if (!read_burst_dma(ams::config::ImuAccAddr7b, ams::bmi088::AccDataReg, 0) ||
            !read_burst_dma(ams::config::ImuGyrAddr7b, ams::bmi088::GyrDataReg, 6)) {
            ++g_imu_read_errors;
            g_imu_state = ams::ImuState::BusError;
            reset_bus();
            retry_at = osKernelGetTickCount() + ams::config::ImuRetryPeriodMs;
            (void)osDelayUntil(next);
            continue;
        }
        ams::bmi088::decode_axes(s_dma_buf,     s.acc);
        ams::bmi088::decode_axes(s_dma_buf + 6, s.gyr);
        (void)ams::sd_imu_push(s);   // best-effort; the logger counts drops
        ++g_imu_samples;

        (void)osDelayUntil(next);
    }
}

}  // extern "C"

namespace ams {

ImuStats imu_stats() noexcept {
    return ImuStats{ g_imu_samples, g_imu_read_errors, g_imu_inits, g_imu_state };
}

}  // namespace ams
