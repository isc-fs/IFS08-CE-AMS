/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
/* Captured globals for post-mortem readout over CAN (pit-diag
 * frame 0x6C5). Live in .bss so they survive a soft reset; the
 * next boot ships them out before normal telemetry comes up. */
volatile uint32_t g_stack_overflow_task_addr      = 0;
volatile char     g_stack_overflow_task_name[16]  = {0};
/* High-water mark (free stack words remaining) at the moment of
 * overflow detection. Lets the post-mortem distinguish "barely over"
 * (e.g. 0..3 words remaining) from "blew way past the guard pattern"
 * (e.g. the watermark API itself returns 0 because the corruption
 * already wiped it). Reads xPortGetFreeHeapSize-style snapshot from
 * FreeRTOS. 0xFFFFFFFF means the API call itself failed (no task
 * handle), 0 means the guard region is wiped. */
volatile uint32_t g_stack_overflow_watermark      = 0xFFFFFFFFu;

extern void ams_relays_open_all_c(void);   /* relay_driver.cpp wrapper */
extern void ams_error_latch_set_c(void);   /* error_latch.cpp wrapper */

void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* configCHECK_FOR_STACK_OVERFLOW=2 -> guard pattern detected the
    * overflow. Make this loud: open the relays, latch ERROR in
    * backup register (sticky across reset), capture the failing
    * task's name+handle + watermark for post-mortem readout, then spin
    * so the IWDG resets us within ~100 ms with the latch still set.
    * Next boot, the firmware will come up in ERROR — observable
    * from telemetry even without a debugger. */
   g_stack_overflow_task_addr = (uint32_t)xTask;
   if (pcTaskName != NULL) {
      for (int i = 0; i < 15 && pcTaskName[i] != '\0'; ++i) {
         g_stack_overflow_task_name[i] = (char)pcTaskName[i];
      }
   }
   if (xTask != NULL) {
      g_stack_overflow_watermark = (uint32_t)uxTaskGetStackHighWaterMark(xTask);
   }
   ams_relays_open_all_c();
   ams_error_latch_set_c();
   for (;;) { __asm volatile ("nop"); }
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
volatile uint32_t g_malloc_failed_count = 0;

void vApplicationMallocFailedHook(void)
{
   /* pvPortMalloc() returned NULL. In practice this fires when a
    * task / queue / event group creation runs out of contiguous
    * heap. Without this hook the failure is silent and the
    * affected primitive simply never exists -- which is exactly
    * the symptom #123 spent two iterations chasing. Same loud
    * treatment as the stack-overflow path. */
   ++g_malloc_failed_count;
   ams_relays_open_all_c();
   ams_error_latch_set_c();
   for (;;) { __asm volatile ("nop"); }
}
/* USER CODE END 5 */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

