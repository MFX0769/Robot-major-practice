/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* taurus 业务任务 (2026-11-22 迁移) */
#include "can_comm.h"
#include "usb_comm.h"
#include "Vision_Send_task.h"
#include "mode_switch_task.h"
#include "status_task.h"
#include "ui_task.h"
#include "arm_task.h"
#include "chassis_task.h"
#include "bsp_imu_bridge.h"
#include "dbg_print.h"
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
/* 业务队列句柄 (从 taurus 主工程迁移, 2026-11-22) */
#include "can_message.h"
QueueHandle_t VISION_send_queue = NULL;
QueueHandle_t CAN1_Tx_queue = NULL;
QueueHandle_t CAN2_Tx_queue = NULL;
QueueHandle_t USB_tx_queue = NULL;   /* unified CDC tx (2026-11-22) */
/* 业务任务句柄 */
osThreadId can1_transmit_task_t;
osThreadId can2_transmit_task_t;
osThreadId usb_task_t;
osThreadId vision_send_task_t;
osThreadId mode_sw_task_t;
osThreadId status_task_t;
osThreadId ui_task_t;
osThreadId arm_task_t;
osThreadId chassis_task_t;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId KeyTaskHandle;
uint32_t KeyTaskBuffer[ 128 ];
osStaticThreadDef_t KeyTaskControlBlock;
osThreadId LcdTaskHandle;
uint32_t LcdTaskBuffer[ 256 ];
osStaticThreadDef_t LcdTaskControlBlock;
osThreadId ImuTaskHandle;
uint32_t ImuTaskBuffer[ 1024 ];
osStaticThreadDef_t ImuTaskControlBlock;
osThreadId FunTestHandle;
uint32_t FunTestBuffer[ 128 ];
osStaticThreadDef_t FunTestControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void KeyTask_Entry(void const * argument);
void LcdTask_Entry(void const * argument);
void ImuTask_Entry(void const * argument);
void FunTest_Entry(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* 业务队列 (从 taurus 主工程迁移, 2026-11-22) */
  VISION_send_queue = xQueueCreate(5, 44);
  CAN1_Tx_queue     = xQueueCreate(15, sizeof(CanFrame_t));
  CAN2_Tx_queue     = xQueueCreate(8, sizeof(CanFrame_t));
  USB_tx_queue      = xQueueCreate(16, sizeof(usb_tx_msg_t));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityLow, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of KeyTask */
  osThreadStaticDef(KeyTask, KeyTask_Entry, osPriorityLow, 0, 256, KeyTaskBuffer, &KeyTaskControlBlock);
  KeyTaskHandle = osThreadCreate(osThread(KeyTask), NULL);

  /* definition and creation of LcdTask */
  osThreadStaticDef(LcdTask, LcdTask_Entry, osPriorityLow, 0, 256, LcdTaskBuffer, &LcdTaskControlBlock);
  LcdTaskHandle = osThreadCreate(osThread(LcdTask), NULL);

  /* definition and creation of ImuTask */
  osThreadStaticDef(ImuTask, ImuTask_Entry, osPriorityHigh, 0, 1024, ImuTaskBuffer, &ImuTaskControlBlock);
  ImuTaskHandle = osThreadCreate(osThread(ImuTask), NULL);

  /* definition and creation of FunTest -- removed 2026-11-22 (test task, source removed) */

  /* USER CODE BEGIN RTOS_THREADS */
  /* taurus 业务任务 (2026-11-22 迁移) */
  osThreadDef(can1Task, Task_CAN1Transmit, osPriorityHigh, 0, 256);
  can1_transmit_task_t = osThreadCreate(osThread(can1Task), NULL);
  osThreadDef(can2Task, Task_CAN2Transmit, osPriorityHigh, 0, 256);
  can2_transmit_task_t = osThreadCreate(osThread(can2Task), NULL);
  osThreadDef(usbTask, Task_USB_Transmit, osPriorityHigh, 0, 256);
  usb_task_t = osThreadCreate(osThread(usbTask), NULL);
  osThreadDef(modeswTask, mode_switch_task, osPriorityNormal, 0, 128);
  mode_sw_task_t = osThreadCreate(osThread(modeswTask), NULL);
  osThreadDef(statusTask, status_task, osPriorityNormal, 0, 256);
  status_task_t = osThreadCreate(osThread(statusTask), NULL);
  osThreadDef(armTask, arm_task, osPriorityAboveNormal, 0, 768);
  arm_task_t = osThreadCreate(osThread(armTask), NULL);
  osThreadDef(chassisTask, chassis_task, osPriorityNormal, 0, 256);
  chassis_task_t = osThreadCreate(osThread(chassisTask), NULL);
  osThreadDef(dbgTask, dbg_task, osPriorityLow, 0, 256);
  (void)osThreadCreate(osThread(dbgTask), NULL);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_KeyTask_Entry */
/**
* @brief Function implementing the KeyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_KeyTask_Entry */
__weak void KeyTask_Entry(void const * argument)
{
  /* USER CODE BEGIN KeyTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END KeyTask_Entry */
}

/* USER CODE BEGIN Header_LcdTask_Entry */
/**
* @brief Function implementing the LcdTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LcdTask_Entry */
__weak void LcdTask_Entry(void const * argument)
{
  /* USER CODE BEGIN LcdTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END LcdTask_Entry */
}

/* USER CODE BEGIN Header_ImuTask_Entry */
/**
* @brief Function implementing the ImuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ImuTask_Entry */
__weak void ImuTask_Entry(void const * argument)
{
  /* USER CODE BEGIN ImuTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ImuTask_Entry */
}

/* USER CODE BEGIN Header_FunTest_Entry */
/**
* @brief Function implementing the FunTest thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FunTest_Entry */
__weak void FunTest_Entry(void const * argument)
{
  /* USER CODE BEGIN FunTest_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END FunTest_Entry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
