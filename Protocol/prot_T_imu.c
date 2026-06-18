#include "prot_T_imu.h"
#include "bsp_can.h"
#include "gimbal_task.h"
#include "KalmanFilter.h"
#include "usart.h"


int sys_us[2];
int incement;
int err[2];
float 	palstance_buffer[2];
float 	angle_buffer[2];

uint32_t imucnt=0;
//static float yaw_temp;
//static float last_yaw;
//uint32_t yaw_cnt;

Taurus_imu_data_t   imu_data;
void T_imu_calcu(uint32_t can_id,uint8_t * CAN_Rx_data)
{
				imucnt++;

	sys_us[0] = HAL_GetTick();
	#if OLD_IMU
	/******飞机陀螺仪的安装位置*******/
	/*     箭头朝枪管，箭头面朝上    */
	/*       枪头朝右和朝上为正      */
    switch(can_id)
    {
			case TIMU_PALSTANCE_ID:	//角速度
			{
//2025年2月15日 目前要做三轴云台，需要有roll轴的角度数据，但是旧imu是不会发送roll数据的，角度数据如果CAN一帧8byte，也发不了4位的数据。
//但是云台并没有用到加速度数据，所以暂时先将用加速度的名额给roll角度发。用前4位发，后四位发0
//          float palstance_buffer[2];
					memcpy(palstance_buffer,CAN_Rx_data,8);

				/*2025年2月15日之前的原代码，负责接收解算imu传来的加速度数据，以下*/
					imu_data.wy = -palstance_buffer[0]/16.3835f;  //当陀螺仪水平旋转180度时需要加-号
					imu_data.wz = -palstance_buffer[1]/16.3835f;
				/*---------------------------以上旧代码，以下新代码-----------------------------*/
//					imu_data.roll =  -palstance_buffer[0] ;  //这个接收还没试过有没有问题
//					imu_data.wz = -palstance_buffer[1]/16.3835f;
				/*---------------------------新代码结束（测试的时候注意下面几行和时间有关的有没有影响----------------------------------------*/
				
			}
			break;
			case TIMU_ANGLE_ID:			//角度
			{
				
								if(imu_data.msg_cnt++ <= 50)
							imu_data.cnt = 0;
				
					imu_data.last_yaw = imu_data.yaw;
					memcpy(angle_buffer,CAN_Rx_data,8);
					imu_data.pitch =  -angle_buffer[0] ;  //当陀螺仪水平旋转180度时需要加-号
					imu_data.yaw   =  -(angle_buffer[1]-360.0);

					if(imu_data.yaw - imu_data.last_yaw > 180)
						imu_data.cnt--;
					else if(imu_data.yaw - imu_data.last_yaw < -180)
						imu_data.cnt++;
					
					imu_data.real_yaw = imu_data.cnt*360 + imu_data.yaw ;
					
			}
			break;
    }
		
	#else 
	    switch(can_id)
    {
			case TIMU_PALSTANCE_ID:	//角速度
			{
					memcpy(palstance_buffer,CAN_Rx_data,8);
					imu_data.wy = palstance_buffer[0] *57.3f;  
					imu_data.wz = palstance_buffer[1] *57.3f;

				
			}
			break;
			case TIMU_ANGLE_ID:			//角度
			{
					memcpy(angle_buffer,CAN_Rx_data,8);
					imu_data.pitch =  angle_buffer[0];  
					imu_data.yaw   =  angle_buffer[1];
			}
			break;
			case TIMU_ROL_ID:			//角度
			{
					memcpy(angle_buffer,CAN_Rx_data,8);
					imu_data.roll =  angle_buffer[0];  
					imu_data.wx   =  angle_buffer[1];
			}
			break;
    }
		
	#endif
}
