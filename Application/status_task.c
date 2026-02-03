/**
  * @file status_task.c
  * @version 1.2
  * @date  2023.1.15
	*
  * @brief 状态任务
	*        1.0是步兵代码
	*   		 1.1是之前的云台代码
	*        1.2把状态灯全搬过来了，因为之前会影响发射器的任务时序
	*
  *	@author  某空中组硬件
  *
  */
#include "status_task.h"
#include "mode_switch_task.h"
#include "string.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "iwdg.h"
#include "prot_FricMotor.h"
#include "math_calcu.h"
#include "remote_msg.h"



/* 状态合并 */
#define WDG_BITS_TASK           ( WDG_BIT_TASK_MODES                  \
                                | WDG_BIT_TASK_GIMBAL         | WDG_BIT_TASK_SHOOT         ) 
//#define WDG_BITS_TASK           ( WDG_BIT_TASK_MODES          | WDG_BIT_TASK_JUDGE        \
//                                | WDG_BIT_TASK_GIMBAL         | WDG_BIT_TASK_SHOOT        \
//																|	WDG_BIT_TASK_DEBUG ) 
																
#define WDG_BITS_BSP_FRIC    		( WDG_BIT_BSP_FRIC_MOTO1   	  | WDG_BIT_BSP_FRIC_MOTO1    )	

#define WDG_BITS_BSP_GIMBAL     ( WDG_BIT_BSP_GIMBAL_YAW      | WDG_BIT_BSP_GIMBAL_PIT      )                                
																
#define WDG_BITS_BSP_IMU        ( WDG_BIT_BSP_IMU_ANGLE       | WDG_BIT_BSP_IMU_SPEED       )

/* 状态显示 LED从上到下 */
#define LED_STATUS_1 status.gimbal
#define LED_STATUS_2 status.fric
#define LED_STATUS_3 status.stir
#define LED_STATUS_4 status.imu

extern uint8_t AMMO_STUCK_FLAG;              
extern uint8_t lock_flag ;
extern ctrl_mode_e ctrl_mode;
extern _gimbal_ctrl_ gimbal_ctrl;




__packed union { /* 系统状态标志位 */
    uint32_t wdg_event_group;
    __packed struct { /* 需要与.h文件中的宏一一对应 */
        uint32_t mode_task      :1; //1
        uint32_t gimbal_task    :1; //2
        uint32_t shoot_task     :1; //3
        uint32_t judge_task     :1; //4
        uint32_t debug_task     :1; //5
        
        uint32_t fric_moto1  :1; //6
        uint32_t fric_moto2  :1; //7
       
        
        uint32_t gimbal_yaw     :1; //8
        uint32_t gimbal_pit     :1; //9
			
        uint32_t imu_angle      :1; //10
        uint32_t imu_speed      :1; //11
        
        uint32_t trigger_moto   :1; //12
			
        uint32_t remote         :1; //13
			
        uint32_t judge          :1; //14
				
        uint32_t else_rsvd      :18;//15-32
    } bit;
} s;

status_t status;

void wdg_user_set_bit(uint32_t status_bit) { /* 待检测量执行标记 */
    s.wdg_event_group |= status_bit;
}

static void bit_clear(void) { /* 标记清空 */
    s.wdg_event_group &= 0;
}

static uint8_t bit_check(uint32_t status_bit) { /* 待检测量执行检查 */
    return !((s.wdg_event_group & status_bit) ^ status_bit);
}

static void status_init(void) {
    bit_clear();
    memset(&status, 0, sizeof(status_t));
//    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8,  GPIO_PIN_SET);  //LED_D
//    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9,  GPIO_PIN_SET);  //LED_C
//    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);  //LED_B
//    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);  //LED_A
}

//static void status_deinit(void) { /* 亮灯 */
////    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8,  GPIO_PIN_RESET);  //LED_D
////    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9,  GPIO_PIN_RESET);  //LED_C
////    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);  //LED_B
////    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);  //LED_A
//}

void status_task(void const *argu) {
    status_init();
	
	  uint32_t status_task_time = osKernelSysTick();

    for (;;) {
//        static uint8_t led_status;
        taskENTER_CRITICAL();
        

			
        /* 通信状态检测与输出 */
        status.remote   = bit_check(WDG_BIT_BSP_REMOTE);;//bit_check(WDG_BIT_BSP_REMOTE);
        status.fric     = bit_check(WDG_BITS_BSP_FRIC);
        status.gimbal   = bit_check(WDG_BITS_BSP_GIMBAL);
        status.imu      = bit_check(WDG_BITS_BSP_IMU);
				status.stir     = bit_check(WDG_BIT_BSP_TRIGGER);
				status.judge    = bit_check(WDG_BIT_BSP_JUDGE);
        
        /* 任务状态检测 */
//        if (bit_check(WDG_BITS_TASK)) { 
//            HAL_IWDG_Refresh(&hiwdg); /* 喂狗 */
//        }
        
				        bit_clear(); /* 状态清空，等待下一次检测 */

        /* 遥控器失联则紧急切图传链路  25分区赛版*/
//        if (!status.remote) 
//				{/* 遥控通信周期大约14ms 若3000ms都没有遥控器串口中断，则判定为遥控器失联。若该任务的周期设置为5ms，则CNT应该为300/5=60次 */
//          rc_fail_cnt++;//若遥控器收不到数据，没有喂遥控器狗，就开始怀疑遥控器断联， 使用CNT++来充当一定程度上的计时器
//					
//					if(rc_fail_cnt>=60)
//					{
//						RC_FAIL_FLAG=1;//
//					}
//        }
//				else//即遥控器有连接
//				{
//					rc_fail_cnt=0;//清除质疑cnt
//					RC_FAIL_FLAG=0;
//				}
				
				/*旧遥控失联切新遥控或图传链路  25国赛版*/
//				NEW_OLD_RC_Switch();
				
				
        taskEXIT_CRITICAL();
        osDelay(30); // 决定LED time_cnt 单位，比任务最长的周期长两到三倍即可  
//        osDelayUntil(&status_task_time,20);//必须使用绝对延时！否则会影响遥控器断联时间的判断！！！  2025年5月14日 2：22 好家伙改出石了，小于15就会，导致 lock_flag内八松掉之后马上变回0

    }
}
