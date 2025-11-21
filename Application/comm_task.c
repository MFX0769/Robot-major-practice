#include "comm_task.h"
#include "can_comm.h"
#include "cmsis_os.h"
#include "prot_dm_motor.h"
#include "prot_rm_motor.h"
#include "mode_switch_task.h"
motor_current_t motor_cur;

uint32_t can_send_cnt = 0;

void can_msg_send_task(void const *argu)
{
	osEvent event;
    uint32_t wake_time = osKernelSysTick();

    for (;;)
    {
        if( ctrl_mode==PROTECT_MODE || !lock_flag )
            {
//                can1_send_message(GIMBAL_CAN_TX_ID, 0, 0, 0, 0);
//			    can1_send_message( FRIC_CAN_TX_ID , 0, 0, 0, 0);
				dm_mit_control(&pitch_motor);
				dji_send_current(&fric_motor[0],&fric_motor[1],NULL,NULL);
				dji_send_current(&pit_motor,&yaw_motor,&trigger_motor ,NULL);
				can_send_cnt++;
            }
            else if( lock_flag )  //有陀螺仪数据才给电流
            {
                if( event.value.signals  )
                {
						
					dm_mit_control(&pitch_motor);				
//					can1_send_message(FRIC_CAN_TX_ID,motor_cur.fric_cur[0], motor_cur.fric_cur[1], 0, 0);
//					can1_send_message(GIMBAL_CAN_TX_ID,motor_cur.gimbal_cur[1], motor_cur.gimbal_cur[0], motor_cur.trigger_cur, 0);
				 	dji_send_current(&fric_motor[0],&fric_motor[1],NULL,NULL);
					dji_send_current(&pit_motor,&yaw_motor,&trigger_motor ,NULL);				
					can_send_cnt++;

                }
            }
						
          osDelayUntil(&wake_time, 1);
    }
}
