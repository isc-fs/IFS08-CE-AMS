/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app/acu_can_task.h"
#include "app/app_globals.h"
#include "app/app_init_task.h"
#include "app/bms_poll_task.h"
#include "app/can_frame.h"
#include "app/current_task.h"
#include "app/safety_task.h"
#include "app/state_task.h"
#include "app/telemetry_task.h"
#include "app/watchdog.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

IWDG_HandleTypeDef hiwdg1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for App_InitTask */
osThreadId_t App_InitTaskHandle;
const osThreadAttr_t App_InitTask_attributes = {
  .name = "App_InitTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for SafetyTask */
osThreadId_t SafetyTaskHandle;
const osThreadAttr_t SafetyTask_attributes = {
  .name = "SafetyTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for StateTask */
osThreadId_t StateTaskHandle;
const osThreadAttr_t StateTask_attributes = {
  .name = "StateTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for BmsPollTask */
osThreadId_t BmsPollTaskHandle;
const osThreadAttr_t BmsPollTask_attributes = {
  .name = "BmsPollTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for AcuCanTask */
osThreadId_t AcuCanTaskHandle;
const osThreadAttr_t AcuCanTask_attributes = {
  .name = "AcuCanTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for CurrentSensorTask */
osThreadId_t CurrentSensorTaskHandle;
const osThreadAttr_t CurrentSensorTask_attributes = {
  .name = "CurrentSensorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for TelemetryTask */
osThreadId_t TelemetryTaskHandle;
const osThreadAttr_t TelemetryTask_attributes = {
  .name = "TelemetryTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for acu_rx_queue */
osMessageQueueId_t acu_rx_queueHandle;
const osMessageQueueAttr_t acu_rx_queue_attributes = {
  .name = "acu_rx_queue"
};
/* Definitions for acu_tx_queue */
osMessageQueueId_t acu_tx_queueHandle;
const osMessageQueueAttr_t acu_tx_queue_attributes = {
  .name = "acu_tx_queue"
};
/* Definitions for bms_mutex */
osMutexId_t bms_mutexHandle;
const osMutexAttr_t bms_mutex_attributes = {
  .name = "bms_mutex"
};
/* Definitions for current_mutex */
osMutexId_t current_mutexHandle;
const osMutexAttr_t current_mutex_attributes = {
  .name = "current_mutex"
};
/* Definitions for vehicle_mutex */
osMutexId_t vehicle_mutexHandle;
const osMutexAttr_t vehicle_mutex_attributes = {
  .name = "vehicle_mutex"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM17_Init(void);
static void MX_SPI1_Init(void);
static void MX_IWDG1_Init(void);
void StartDefaultTask(void *argument);
void StartAppInitTask(void *argument);
void StartSafetyTask(void *argument);
void StartStateTask(void *argument);
void StartBmsPollTask(void *argument);
void StartAcuCanTask(void *argument);
void StartCurrentSensorTask(void *argument);
void StartTelemetryTask(void *argument);

/* USER CODE BEGIN PFP */
#if defined(AMS_BMS_HIL_STUB)
/* FreeRTOS heap accessor -- safe pre-osKernelInitialize: it just reads
 * a counter on a static heap array. */
extern size_t xPortGetFreeHeapSize(void);

/* #123 iter 19: per-thread bring-up probe. Emits 0x7B0+idx on can0 with
 * payload[0]=marker, payload[1..4]=phase byte tag (0xBE = "before call"
 * / 0xAF = "after call returned"), payload[5..6] reserved, payload[7]=
 * 0xAA iter marker. Refreshes IWDG before queuing the frame to keep the
 * watchdog from biting mid-probe. Bench-only. */
static void hil_thread_probe(uint8_t idx, uint8_t phase)
{
  HAL_IWDG_Refresh(&hiwdg1);
  FDCAN_TxHeaderTypeDef tx = {0};
  tx.Identifier          = 0x7B0u + idx;
  tx.IdType              = FDCAN_STANDARD_ID;
  tx.TxFrameType         = FDCAN_DATA_FRAME;
  tx.DataLength          = FDCAN_DLC_BYTES_8;
  tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx.BitRateSwitch       = FDCAN_BRS_OFF;
  tx.FDFormat            = FDCAN_CLASSIC_CAN;
  tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
  tx.MessageMarker       = 0;
  uint8_t data[8] = { idx, phase, 0, 0, 0, 0, 0, 0xAAu };
  (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx, data);
}

/* #123 iter 20: generic pre-kernel probe. Emits arbitrary 11-bit ID
 * with payload[0]=id_lo, payload[1..4]=payload32 LE, payload[7]=0xAA.
 * Used to bracket osKernelInitialize + each mutex/queue allocation so
 * the bench can pinpoint which call dies (operator's iter-19 capture
 * showed even 0x7B0 BE doesn't emit -- crash is in the
 * osKernelInitialize / mutex / queue block before any osThreadNew). */
static void hil_probe(uint16_t id, uint32_t payload32)
{
  HAL_IWDG_Refresh(&hiwdg1);
  FDCAN_TxHeaderTypeDef tx = {0};
  tx.Identifier          = id;
  tx.IdType              = FDCAN_STANDARD_ID;
  tx.TxFrameType         = FDCAN_DATA_FRAME;
  tx.DataLength          = FDCAN_DLC_BYTES_8;
  tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx.BitRateSwitch       = FDCAN_BRS_OFF;
  tx.FDFormat            = FDCAN_CLASSIC_CAN;
  tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
  tx.MessageMarker       = 0;
  uint8_t data[8] = {
    (uint8_t)(id & 0xFFu),
    (uint8_t)(payload32        & 0xFFu),
    (uint8_t)((payload32 >>  8) & 0xFFu),
    (uint8_t)((payload32 >> 16) & 0xFFu),
    (uint8_t)((payload32 >> 24) & 0xFFu),
    0, 0, 0xAAu,
  };
  (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx, data);
}
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* Application is linked at 0x08020000 (sector 1+); sector 0 belongs
   * to the bootloader. Set SCB->VTOR before any interrupt could fire
   * so the right vector table is in effect even when this image is
   * flashed directly (without the bootloader's pre-jump VTOR setup).
   * Must precede HAL_Init -- which itself relies on SysTick / NMI
   * paths reading from the correct table.
   * See isc-fs/stm32-can-bootloader Core/Inc/bl_memmap.h. */
  SCB->VTOR = 0x08020000U;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN2_Init();
  MX_FDCAN1_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_TIM17_Init();
  MX_SPI1_Init();
  MX_IWDG1_Init();
  /* USER CODE BEGIN 2 */
  /* IWDG1 is started by MX_IWDG1_Init() above (CubeMX-owned), which
   * runs before osKernelStart and so satisfies the "watchdog alive in
   * the pre-scheduler window" invariant. SafetyTask owns the refresh
   * cadence (10 ms period; ~100 ms reload at nominal LSI). */

  /* refactor/19 phase 2: the pre-scheduler ErrorLatch clear from
   * PR #114 was retired here. BmsPollTask now seeds a nominal-
   * healthy BmsState under -DAMS_BMS_HIL_STUB (see bms_service.hpp
   * ::seed_for_hil_stub), so the predicate no longer trips on the
   * bench's missing LTC chain -- there's no fault to latch in the
   * first place. App_InitTask still calls ErrorLatch::clear() under
   * the same flag as defence-in-depth against a backup register
   * surviving a previous (pre-phase-2) session. */
#if defined(AMS_BMS_HIL_STUB)
  /* #123 iter 17: pre-scheduler FDCAN_Start + boot-trace probe.
   * All three FDCAN-config reverts (#159, #161, #163) failed to
   * restore TX. The remaining suspects are SafetyTask / App_InitTask
   * code paths, but we have no way to observe them on this bench
   * without TX working. Move HAL_FDCAN_Start to here so we can
   * emit a probe frame BEFORE any of the suspect code runs.
   *
   * If candump sees 0x7AA after this point, the FDCAN1 TX path
   * itself works in this firmware tree -- the regression is
   * downstream (App_InitTask hang, or osThreadNew failure for
   * MainTask, or similar). If we see nothing, the regression is
   * even earlier than HAL_FDCAN_Start succeeding (very unlikely
   * given pre-#152 firmware transmitted with this same chip and
   * the same RCC config). App_InitTask's own HAL_FDCAN_Start
   * is now redundant but harmless (already in BUSY state).
   *
   * Bench-only -- flight keeps the post-scheduler Start in
   * App_InitTask exactly as before.
   */
  if (HAL_FDCAN_Start(&hfdcan1) == HAL_OK) {
    FDCAN_TxHeaderTypeDef tx = {0};
    tx.Identifier          = 0x7AAu;
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = FDCAN_DLC_BYTES_8;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0;
    /* #123 iter 18: payload[1..4] carry IWDG->RLR (little-endian uint32)
     * so the bench can confirm the configured reload counter without
     * SWD. Bench observed ~2.1 s reset spacing -- if RLR encodes a
     * value that matches 2.1 s at the LSI frequency (32 kHz / 32 div
     * = 1 kHz tick -> RLR=2100 would yield ~2.1 s), IWDG is the
     * smoking gun and SafetyTask::run never reaches Watchdog::refresh
     * before expiry. Payload[5] = marker 0xAA so the bench can
     * distinguish iter-18 frames from iter-17 frames (which had
     * payload[1..7] all-zero). */
    const uint32_t rlr = IWDG1->RLR;
    uint8_t data[8] = {
      0xA0u,
      (uint8_t)(rlr        & 0xFFu),
      (uint8_t)((rlr >>  8) & 0xFFu),
      (uint8_t)((rlr >> 16) & 0xFFu),
      (uint8_t)((rlr >> 24) & 0xFFu),
      0xAAu, 0, 0,
    };
    (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx, data);
    /* Give the hardware time to actually transmit before we move on
     * to osKernelStart -- otherwise the FIFO write races the start
     * of FreeRTOS scheduling and might not drain. ~1 ms at 500 kbps
     * is well over the time needed to transmit one 8-byte frame
     * (which takes ~250 us on the wire). */
    HAL_Delay(2);
    /* #123 iter 18: explicit IWDG refresh after the probe send.
     * If this single refresh extends time-to-reset measurably (e.g.
     * 0x7AA spacing becomes 4.2 s instead of 2.1 s, or the chip
     * stops resetting once SafetyTask runs and refreshes again),
     * IWDG starvation in the pre-scheduler / early-scheduler window
     * is confirmed as the root cause. Bench-only. */
    HAL_IWDG_Refresh(&hiwdg1);
    /* #123 iter 20: heap snapshot before any FreeRTOS object is
     * allocated. If this is dangerously low (< a few KB) the
     * regression is static-init eating the pool, not the allocations
     * themselves. payload = free heap in bytes (LE u32). */
    hil_probe(0x7ACu, (uint32_t)xPortGetFreeHeapSize());
  }
#endif
  /* USER CODE END 2 */

  /* Init scheduler */
#if defined(AMS_BMS_HIL_STUB)
  hil_probe(0x7C0u, 0xBEEFCAFEu);  /* before osKernelInitialize */
#endif
  osKernelInitialize();
#if defined(AMS_BMS_HIL_STUB)
  hil_probe(0x7C0u, (uint32_t)osKernelGetState());  /* after */
#endif
  /* Create the mutex(es) */
  /* creation of bms_mutex */
  bms_mutexHandle = osMutexNew(&bms_mutex_attributes);
#if defined(AMS_BMS_HIL_STUB)
  hil_probe(0x7C1u, (uint32_t)(uintptr_t)bms_mutexHandle);
#endif

  /* creation of current_mutex */
  current_mutexHandle = osMutexNew(&current_mutex_attributes);
#if defined(AMS_BMS_HIL_STUB)
  hil_probe(0x7C2u, (uint32_t)(uintptr_t)current_mutexHandle);
#endif

  /* creation of vehicle_mutex */
  vehicle_mutexHandle = osMutexNew(&vehicle_mutex_attributes);
#if defined(AMS_BMS_HIL_STUB)
  hil_probe(0x7C3u, (uint32_t)(uintptr_t)vehicle_mutexHandle);
#endif

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of acu_rx_queue */
  acu_rx_queueHandle = osMessageQueueNew (16, sizeof(CanFrame), &acu_rx_queue_attributes);
#if defined(AMS_BMS_HIL_STUB)
  hil_probe(0x7C4u, (uint32_t)(uintptr_t)acu_rx_queueHandle);
#endif

  /* creation of acu_tx_queue */
  acu_tx_queueHandle = osMessageQueueNew (16, sizeof(CanFrame), &acu_tx_queue_attributes);
#if defined(AMS_BMS_HIL_STUB)
  hil_probe(0x7C5u, (uint32_t)(uintptr_t)acu_tx_queueHandle);
#endif

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
#if defined(AMS_BMS_HIL_STUB)
  hil_thread_probe(0, 0xBEu);
#endif
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
#if defined(AMS_BMS_HIL_STUB)
  hil_thread_probe(0, 0xAFu);
#endif

  /* creation of App_InitTask */
#if defined(AMS_BMS_HIL_STUB)
  hil_thread_probe(1, 0xBEu);
#endif
  App_InitTaskHandle = osThreadNew(StartAppInitTask, NULL, &App_InitTask_attributes);
#if defined(AMS_BMS_HIL_STUB)
  hil_thread_probe(1, 0xAFu);
#endif

  /* creation of SafetyTask */
#if defined(AMS_BMS_HIL_STUB)
  hil_thread_probe(2, 0xBEu);
#endif
  SafetyTaskHandle = osThreadNew(StartSafetyTask, NULL, &SafetyTask_attributes);
#if defined(AMS_BMS_HIL_STUB)
  hil_thread_probe(2, 0xAFu);
#endif

  /* creation of StateTask */
#if defined(AMS_BMS_HIL_STUB)
  hil_thread_probe(3, 0xBEu);
#endif
  StateTaskHandle = osThreadNew(StartStateTask, NULL, &StateTask_attributes);
#if defined(AMS_BMS_HIL_STUB)
  hil_thread_probe(3, 0xAFu);
#endif

  /* creation of BmsPollTask */
#if defined(AMS_BMS_HIL_STUB)
  hil_thread_probe(4, 0xBEu);
#endif
  BmsPollTaskHandle = osThreadNew(StartBmsPollTask, NULL, &BmsPollTask_attributes);
#if defined(AMS_BMS_HIL_STUB)
  hil_thread_probe(4, 0xAFu);
#endif

  /* creation of AcuCanTask */
#if defined(AMS_BMS_HIL_STUB)
  hil_thread_probe(5, 0xBEu);
#endif
  AcuCanTaskHandle = osThreadNew(StartAcuCanTask, NULL, &AcuCanTask_attributes);
#if defined(AMS_BMS_HIL_STUB)
  hil_thread_probe(5, 0xAFu);
#endif

  /* creation of CurrentSensorTask */
#if defined(AMS_BMS_HIL_STUB)
  hil_thread_probe(6, 0xBEu);
#endif
  CurrentSensorTaskHandle = osThreadNew(StartCurrentSensorTask, NULL, &CurrentSensorTask_attributes);
#if defined(AMS_BMS_HIL_STUB)
  hil_thread_probe(6, 0xAFu);
#endif

  /* creation of TelemetryTask */
#if defined(AMS_BMS_HIL_STUB)
  hil_thread_probe(7, 0xBEu);
#endif
  TelemetryTaskHandle = osThreadNew(StartTelemetryTask, NULL, &TelemetryTask_attributes);
#if defined(AMS_BMS_HIL_STUB)
  hil_thread_probe(7, 0xAFu);
#endif

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
#if defined(AMS_BMS_HIL_STUB)
  /* #123 iter 18: post-osThreadNew / pre-osKernelStart probe (0x7AB).
   * Brackets the FreeRTOS-init phase so the bench can tell whether
   * the chip dies during osThreadNew (no 0x7AB) or after the
   * scheduler actually starts (0x7AB present, but no later
   * App_InitTask 0xB1..0xB7 trace and no 0x4A0/0x4A1/0x4A2). Also
   * refreshes IWDG -- the osThreadNew + queue + mutex setup above
   * can chew through tens of ms in -O0 builds; combined with the
   * pre-scheduler HAL_Delay(2) and any LSI slack, it may be
   * dangerously close to the 100-tick reload. Bench-only. */
  {
    FDCAN_TxHeaderTypeDef tx = {0};
    tx.Identifier          = 0x7ABu;
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = FDCAN_DLC_BYTES_8;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0;
    const uint32_t rlr = IWDG1->RLR;
    uint8_t data[8] = {
      0xABu,
      (uint8_t)(rlr        & 0xFFu),
      (uint8_t)((rlr >>  8) & 0xFFu),
      (uint8_t)((rlr >> 16) & 0xFFu),
      (uint8_t)((rlr >> 24) & 0xFFu),
      0xAAu, 0, 0,
    };
    (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx, data);
    HAL_Delay(2);
    HAL_IWDG_Refresh(&hiwdg1);
  }
#endif
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* CubeMX 6.16 CMake target does not emit code for event groups
   * declared in the .ioc -- create them here. See app_globals.cpp. */
  ams_app_globals_init();
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 44;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 16;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = 64;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_6;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc3.Init.Resolution = ADC_RESOLUTION_10B;
  hadc3.Init.DataAlign = ADC3_DATAALIGN_RIGHT;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  hadc3.Init.Oversampling.Ratio = ADC3_OVERSAMPLING_RATIO_2;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC3_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSign = ADC3_OFFSET_SIGN_NEGATIVE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;   /* #123 iter 16: PR #144 set this to ENABLE for the 0x4A3 silent-drop chase; turns out it was wedging the whole TX path on bench-without-peer (first frame retries forever, FIFO never drains) */
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 3;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 10;
  hfdcan1.Init.NominalTimeSeg2 = 5;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 5;
  hfdcan1.Init.RxFifo0ElmtsNbr = 32;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 32;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 1;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 1;
  hfdcan1.Init.TxBuffersNbr = 16;           /* #123 iter 14: PR #150 set this to 0 trying to fix the 0x4A3 silent-drop, but op confirms post-#152 firmware now silent on BOTH MLC1 and MLC3 — reverting as the most likely regression in the chain */
  hfdcan1.Init.TxFifoQueueElmtsNbr = 16;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 3;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 10;
  hfdcan2.Init.NominalTimeSeg2 = 5;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.MessageRAMOffset = 0;      /* #123 iter 15: PR #148 set this to 1280 to avoid shared SRAMCAN overlap, but PR #159's revert of #150 didn't restore TX; reverting #148 next */
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 5;
  hfdcan2.Init.RxFifo0ElmtsNbr = 32;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 32;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 1;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 1;
  hfdcan2.Init.TxBuffersNbr = 16;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 16;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief IWDG1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG1_Init(void)
{

  /* USER CODE BEGIN IWDG1_Init 0 */

  /* USER CODE END IWDG1_Init 0 */

  /* USER CODE BEGIN IWDG1_Init 1 */

  /* USER CODE END IWDG1_Init 1 */
  hiwdg1.Instance = IWDG1;
  hiwdg1.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg1.Init.Window = 4095;
  hiwdg1.Init.Reload = 100;
  if (HAL_IWDG_Init(&hiwdg1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG1_Init 2 */

  /* USER CODE END IWDG1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 10559;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LTC6820_CS_GPIO_Port, LTC6820_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AMS_OK_GPIO_Port, AMS_OK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RELAY_AIR_N_Pin|RELAY_AIR_P_Pin|RELAY_PRECHARGE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LTC6820_CS_Pin */
  GPIO_InitStruct.Pin = LTC6820_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LTC6820_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Charge_Button_Pin */
  GPIO_InitStruct.Pin = Charge_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Charge_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins on GPIOB:
   *   PB4 = AMS_OK            (push-pull output)
   *   PB5 = RELAY_AIR_P       (push-pull output)
   *   PB6 = RELAY_AIR_N       (push-pull output)
   *   PB7 = RELAY_PRECHARGE   (push-pull output)
   * PB4 is the JTAG-NJTRST function after reset; we're using SWD (only
   * PA13/PA14), so PB4 is free for GPIO once HAL_GPIO_Init reprograms
   * its AFR + MODER -- no extra SYSCFG dance needed on H7.
   */
  GPIO_InitStruct.Pin = AMS_OK_Pin|RELAY_AIR_P_Pin|RELAY_AIR_N_Pin|RELAY_PRECHARGE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartAppInitTask */
/**
* @brief Function implementing the App_InitTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAppInitTask */
void StartAppInitTask(void *argument)
{
  /* USER CODE BEGIN StartAppInitTask */
  ams_app_init_task_run(argument);
  /* Unreachable: ams_app_init_task_run() calls osThreadExit(). */
  for(;;) { osDelay(1); }
  /* USER CODE END StartAppInitTask */
}

/* USER CODE BEGIN Header_StartSafetyTask */
/**
* @brief Function implementing the SafetyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSafetyTask */
void StartSafetyTask(void *argument)
{
  /* USER CODE BEGIN StartSafetyTask */
  ams_safety_task_run(argument);
  /* Unreachable: ams_safety_task_run() never returns. */
  for(;;) { osDelay(1); }
  /* USER CODE END StartSafetyTask */
}

/* USER CODE BEGIN Header_StartStateTask */
/**
* @brief Function implementing the StateTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStateTask */
void StartStateTask(void *argument)
{
  /* USER CODE BEGIN StartStateTask */
  ams_state_task_run(argument);
  for(;;) { osDelay(1); }
  /* USER CODE END StartStateTask */
}

/* USER CODE BEGIN Header_StartBmsPollTask */
/**
* @brief Function implementing the BmsPollTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBmsPollTask */
void StartBmsPollTask(void *argument)
{
  /* USER CODE BEGIN StartBmsPollTask */
  ams_bms_poll_task_run(argument);
  /* Unreachable: ams_bms_poll_task_run() never returns. */
  for(;;) { osDelay(1); }
  /* USER CODE END StartBmsPollTask */
}

/* USER CODE BEGIN Header_StartAcuCanTask */
/**
* @brief Function implementing the AcuCanTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAcuCanTask */
void StartAcuCanTask(void *argument)
{
  /* USER CODE BEGIN StartAcuCanTask */
  ams_acu_can_task_run(argument);
  /* Unreachable: ams_acu_can_task_run() never returns. */
  for(;;) { osDelay(1); }
  /* USER CODE END StartAcuCanTask */
}

/* USER CODE BEGIN Header_StartCurrentSensorTask */
/**
* @brief Function implementing the CurrentSensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCurrentSensorTask */
void StartCurrentSensorTask(void *argument)
{
  /* USER CODE BEGIN StartCurrentSensorTask */
  ams_current_sensor_task_run(argument);
  /* Unreachable: ams_current_sensor_task_run() never returns. */
  for(;;) { osDelay(1); }
  /* USER CODE END StartCurrentSensorTask */
}

/* USER CODE BEGIN Header_StartTelemetryTask */
/**
* @brief Function implementing the TelemetryTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTelemetryTask */
void StartTelemetryTask(void *argument)
{
  /* USER CODE BEGIN StartTelemetryTask */
  ams_telemetry_task_run(argument);
  for(;;) { osDelay(1); }
  /* USER CODE END StartTelemetryTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
