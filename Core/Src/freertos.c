/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usb_comm.h"
#include "can_comm.h"
#include "can_message.h"   // 新增：包含公共 CAN 帧结构体
#include "usart_comm.h"

#include "gimbal_task.h"
#include "shoot_task.h"
#include "Vision_Send_task.h"


#include "mode_switch_task.h"
#include "status_task.h"
#include "debug_task.h"
#include "judge_comm.h"
#include "ui_task.h"
#include "log_task.h"
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

/*Thread handles*/
osThreadId usb_task_t;
osThreadId can1_transmit_task_t;
osThreadId can2_transmit_task_t;
osThreadId can3_transmit_task_t;


osThreadId gimbal_task_t;
osThreadId shoot_task_t;
osThreadId vision_send_task_t;


osThreadId mode_sw_task_t;
osThreadId status_task_t;
osThreadId debug_task_t;
osThreadId ui_task_t;
osThreadId log_task_t;

//osThreadId judge_send_task_t;



/*Queues handles*/
QueueHandle_t VISION_send_queue=NULL; 

QueueHandle_t CAN1_Tx_queue=NULL;
QueueHandle_t CAN2_Tx_queue=NULL;
QueueHandle_t CAN3_Tx_queue=NULL;



/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

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
	taskENTER_CRITICAL();
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  VISION_send_queue = xQueueCreate(5, 30);          //哨兵 1 128 
  CAN1_Tx_queue     = xQueueCreate(15, sizeof(CanFrame_t));          
  CAN2_Tx_queue     = xQueueCreate(15, sizeof(CanFrame_t));          
  CAN3_Tx_queue     = xQueueCreate(15,sizeof(CanFrame_t));  
  
//if (VISION_send_queue == NULL ||
//    CAN1_Tx_queue == NULL ||
//    CAN2_Tx_queue == NULL ||
//    CAN3_Tx_queue == NULL)  {        
//    /* Queue was not created and must not be used. */
//		while (1)
//		{
//			osDelay(1);
//				// 2026.1调试阶段使用，用完就删掉这个while
//		}
//  }
		
	
//	if (CAN2_Tx_queue == NULL)  {        
//    /* Queue was not created and must not be used. */
//		while (1)
//		{
//			osDelay(1);
//				// 2026.1调试阶段使用，用完就删掉这个while
//		}
//  }
		
	
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */

/***************************High priority task***************************/
	osThreadDef(can1Task, Task_CAN1Transmit, osPriorityHigh, 0, 256);
   can1_transmit_task_t = osThreadCreate(osThread(can1Task), NULL);

	osThreadDef(can2Task,Task_CAN2Transmit , osPriorityHigh, 0, 256);
    can2_transmit_task_t = osThreadCreate(osThread(can2Task), NULL);
	
	osThreadDef(can3Task,Task_CAN3Transmit , osPriorityHigh, 0, 256);
   can3_transmit_task_t = osThreadCreate(osThread(can3Task), NULL);



	osThreadDef(usbTask, Task_USB_Transmit, osPriorityHigh, 0, 128);
    usb_task_t = osThreadCreate(osThread(usbTask), NULL);



    
/***********************AboveNormal priority task***********************/
	osThreadDef(gimbalTask, gimbal_task, osPriorityAboveNormal, 0, 512);
    gimbal_task_t = osThreadCreate(osThread(gimbalTask),NULL);

  osThreadDef(shootTask, shoot_task, osPriorityAboveNormal, 0, 256);
    shoot_task_t = osThreadCreate(osThread(shootTask),NULL);

  osThreadDef(visionsendTask, Task_Vision_Send,osPriorityAboveNormal, 0, 128);
    vision_send_task_t = osThreadCreate(osThread(visionsendTask), NULL);
/**************************Normal priority task**************************/
	osThreadDef(modeswTask, mode_switch_task, osPriorityNormal, 0, 128);
    mode_sw_task_t = osThreadCreate(osThread(modeswTask),NULL);
//	
	osThreadDef(debugTask, debug_task, osPriorityNormal, 0, 128);
    debug_task_t = osThreadCreate(osThread(debugTask), NULL);
	
	osThreadDef(uiTask,ui_task, osPriorityNormal, 0, 256);
    ui_task_t = osThreadCreate(osThread(uiTask),NULL);
	
//////	
//	osThreadDef(logTask,AirRobot_Log_Task, osPriorityNormal, 0, 128);
//    log_task_t = osThreadCreate(osThread(logTask),NULL);
/**************************Low priority task**************************/
	osThreadDef(statusTask, status_task, osPriorityNormal, 0, 128);
	status_task_t = osThreadCreate(osThread(statusTask),NULL);
	
  taskEXIT_CRITICAL();
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
  /*电机初始化，从comm_can挪到这里了。因为这些属于“设备初始化”而不是can外设初始化  Motor init,moved from comm_can_init() to here,because "motor" != can Peripherals */
	dji_motor_init(&fric_motor[0], DJI_3508_MOTOR,PROT_CAN_CHANNEL_1, 0x201, 1.0f);
    dji_motor_init(&fric_motor[1], DJI_3508_MOTOR,PROT_CAN_CHANNEL_1, 0x202, 1.0f);
    dji_motor_init(&trigger_motor, DJI_2006_MOTOR,PROT_CAN_CHANNEL_1, 0x203, 82.8);
    dji_motor_init(&yaw_motor,     DJI_6020_MOTOR,PROT_CAN_CHANNEL_2, 0x206, 1.0f);
    
	dm_motor_init(&pitch_motor,DM4310,PROT_CAN_CHANNEL_1,DM_PITCH_RECEIVE_ID,DM_PITCH_SEND_ID,DM_4310_RATIO,DM_PITCH_OFFSET);
	dm_motor_init(&dm3507_motor,DM3507,PROT_CAN_CHANNEL_3,DM_3507_RECEIVE_ID,DM_3507_SEND_ID,DM_3507_RATIO,DM_3507_OFFEST);
	
	
  /* Infinite loop */
  for(;;)
  {
		

    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
