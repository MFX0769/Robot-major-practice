#include "imu_task.h"
#include "bmi088driver.h"
#include "MahonyAHRS.h"
#include "tim.h"
#include "vofa.h"
#include "bsp_imu_bridge.h"
#include "dbg_print.h"
#include <math.h>

#define DES_TEMP    40.0f
#define KP          100.f
#define KI          50.f
#define KD          10.f
#define MAX_OUT     500

float gyro[3] = {0.0f};
float acc[3] = {0.0f};
static float temp = 0.0f;

float imuQuat[4] = {0.0f};
float imuAngle[3] = {0.0f};

float out = 0;
float imu_err = 0;
float err_l = 0;
float err_ll = 0;

void AHRS_init(float quat[4])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;

}

void AHRS_update(float quat[4], float gyro[3], float accel[3])
{
    MahonyAHRSupdateIMU(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
}

void GetAngle(float q[4], float *yaw, float *pitch, float *roll)
{
    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}

/* USER CODE BEGIN Header_ImuTask_Entry */
/**
* @brief Function implementing the ImuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ImuTask_Entry */
void ImuTask_Entry(void const * argument)
{
    /* USER CODE BEGIN ImuTask_Entry */
    osDelay(10);
    /* 2026-11-25 安全关断：IMU 加热 PWM 默认不启动，PB1 强制推挽低电平。
     * 防止断点/任务失控时 CCR4 残留导致持续加热。 */
    htim3.Instance->CCR4 = 0U;
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
    {
        GPIO_InitTypeDef heater = {0};
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_1);
        heater.Pin = GPIO_PIN_1;
        heater.Mode = GPIO_MODE_OUTPUT_PP;
        heater.Pull = GPIO_NOPULL;
        heater.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOB, &heater);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    }
    while(BMI088_init())
    {
        osDelay(100);
    }
    
    AHRS_init(imuQuat);
    /* Infinite loop */
    for(;;)
    {
        BMI088_read(gyro, acc, &temp);
        
        AHRS_update(imuQuat, gyro, acc);
        GetAngle(imuQuat, imuAngle + INS_YAW_ADDRESS_OFFSET, imuAngle + INS_PITCH_ADDRESS_OFFSET, imuAngle + INS_ROLL_ADDRESS_OFFSET);
        
        err_ll = err_l;
        err_l = imu_err;
        imu_err = DES_TEMP - temp;
        out = KP*imu_err + KI*(imu_err + err_l + err_ll) + KD*(imu_err - err_l);
        if (out > MAX_OUT) out = MAX_OUT;
        if (out < 0) out = 0.f;
        htim3.Instance->CCR4 = 0U;   /* 加热强制关断，PB1 已改为普通输出低 */
        {
            static uint32_t dbg_div;
            if (++dbg_div >= 1000U) {
                dbg_div = 0U;
                dbg_print("[HEAT] t=%.1f ccr4=%lu cen=%u pb1=%u pc15=%u\r\n",
                          temp,
                          (unsigned long)htim3.Instance->CCR4,
                          (unsigned)(TIM3->CR1 & TIM_CR1_CEN),
                          (unsigned)((GPIOB->ODR >> 1) & 1U),
                          (unsigned)((GPIOC->ODR >> 15) & 1U));
            }
        }
        
        bsp_imu_bridge_update();  /* feed taurus imu_data (2026-11-22) */
        vofa_send_data(0, imuAngle[0]);
        vofa_send_data(1, imuAngle[1]);
        vofa_send_data(2, imuAngle[2]);
        vofa_sendframetail();
        
        osDelay(1);
    }
    /* USER CODE END ImuTask_Entry */
}



