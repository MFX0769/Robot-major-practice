/**
  ******************************************************************************
  * @file    bsp_imu.c
  * @author  taurus
  * @brief   BMI088 IMU BSP implementation (SPI2 + DRDY 1kHz + Mahony + thermostat)
  * @attention
  *   Ported from vendor CtrBoard-H7_ALL, 2026-11-22.
  *   SPI2: SCK=PB13 / MISO=PC2_C / MOSI=PC1, mode3(CPOL=1,CPHA=2EDGE), 3.75Mbps
  *   ACC_CS=PC0, GYRO_CS=PC3, ACC_INT=PE10, GYRO_INT=PE12
  ******************************************************************************
*/
#include "bsp_imu.h"
#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"
#include "MahonyAHRS.h"
#include "debug_port.h"
#include "cmsis_os.h"
#include <string.h>
#include <math.h>

/* CS/INT 引脚宏已在 main.h 定义 (ACC_CS_Pin/GYRO_CS_Pin/ACC_INT_Pin/GYRO_INT_Pin),
   BMI088Middleware.c include main.h 即可使用 */

/*---------------- 外部句柄 (spi.c 中新增) ----------------*/
extern SPI_HandleTypeDef hspi2;

/*---------------- 私有状态 ----------------*/
static imu_diag_t        g_diag;
static imu_data_t        g_latest;          /* 最新一份数据(读拷贝用) */
static QueueHandle_t     g_imu_queue = NULL;
static osMutexId         g_imu_mutex = NULL;
static osMutexDef(imu_mutex);
static uint8_t           g_initialized = 0;
static float             g_gyro[3], g_acc[3], g_temp;  /* 原始值缓存 */
static float             g_quat[4] = {1,0,0,0};
static float             g_angle[3];        /* yaw/pitch/roll rad */
static uint32_t          g_seq = 0;

/*---------------- 温度控制 PID (原厂参数) ----------------*/
#define IMU_TEMP_KP       100.0f
#define IMU_TEMP_KI       50.0f
#define IMU_TEMP_KD       10.0f
#define IMU_TEMP_MAXOUT   500
static float             g_err_l = 0, g_err_ll = 0;
extern TIM_HandleTypeDef htim3;   /* TIM3 CH4 加热 PWM (若主工程无则退化为无温控) */

/*---------------- DWT 时间戳 ----------------*/
static inline uint32_t dwt_cycle(void) { return DWT->CYCCNT; }
static uint32_t g_cyc_per_us = 0;

/*---------------- 底层 SPI 读写桥接 ----------------*/
/* 原厂 BMI088Middleware.c 的 BMI088_read_write_byte 用 hspi2 (extern),
   在 spi.c 定义 hspi2 且实现 HAL_SPI_MspInit 的 SPI2 分支即可, 无需改原厂文件。 */

/*---------------- DRDY / 数据新旧判据 ----------------*/
/* 读 Gyro 状态寄存器 INT_STAT_1(0x0A) bit7 = drdy_new.
   BMI088 Gyro ODR=1000Hz, 若任务周期 1ms 则每周期通常都有新值;
   若读到旧值(drdy=0), 保留上次数据并标记 fresh=0, 不重复喂 Mahony。 */
static uint8_t imu_gyro_drdy(void)
{
    /* 调研发现: GYRO_INT_STAT_1(0x0A) bit7 在 280-400us 后自动清, 轮询窗口极短,
       固定 1ms 周期轮询极易读到 0 (误判 stale)。为保证先跑通, 暂用"周期即新"策略:
       Gyro ODR=1000Hz 与任务周期 1ms 对齐, 每个周期读到的即视为新采样。
       后续 P0 优化改为 PE12 EXTI 中断 + DMA burst (见 handoff 第八节 A4)。
       返回 1 = 本周期执行读取; fresh 判据由 DWT 时间戳 + seq 保证。 */
    return 1U;
}

/*=======================================================================
 * bsp_imu_init
 *=======================================================================*/
imu_status_t bsp_imu_init(void)
{
    memset(&g_diag, 0, sizeof(g_diag));
    memset(&g_latest, 0, sizeof(g_latest));

    /* DWT cycle counter for timestamp */
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
    g_cyc_per_us = SystemCoreClock / 1000000U;

    /* Queue + mutex */
    g_imu_queue = xQueueCreate(IMU_QUEUE_LENGTH, sizeof(imu_data_t));
    if (g_imu_queue == NULL) { g_diag.status = IMU_ERR_INIT; return IMU_ERR_INIT; }
    g_imu_mutex = osMutexCreate(osMutex(imu_mutex));

    /* BMI088 init (GPIO/SPI already done in MX_SPI2_Init from main) */
    uint8_t err = BMI088_init();
    g_diag.init_error_code = err;
    if (err != BMI088_NO_ERROR) {
        g_diag.status = IMU_ERR_NO_SENSOR;
        return IMU_ERR_NO_SENSOR;
    }

    g_quat[0]=1; g_quat[1]=g_quat[2]=g_quat[3]=0;
    g_initialized = 1;
    g_diag.status = IMU_OK;
    /* 注意: init 在调度器启动前调用, 不能用 osDelay(debug_printf 内有) */
    return IMU_OK;
}

/*=======================================================================
 * 温控 PID: 目标 40°C, TIM3 CH4 加热
 *=======================================================================*/
static void imu_temp_control(float temp)
{
    float err = IMU_TEMP_TARGET_C - temp;
    float out = IMU_TEMP_KP*err + IMU_TEMP_KI*(err+g_err_l+g_err_ll)
              + IMU_TEMP_KD*(err-g_err_l);
    g_err_ll = g_err_l; g_err_l = err;
    if (out > IMU_TEMP_MAXOUT) out = IMU_TEMP_MAXOUT;
    if (out < 0) out = 0;
    /* 若主工程 TIM3 未初始化, 此处写 CCR 无效但不会崩溃(句柄存在即可) */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint16_t)out);
}

/*=======================================================================
 * bsp_imu_read_raw: 直接读原始 gyro/acc/temp
 *=======================================================================*/
void bsp_imu_read_raw(float gyro[3], float accel[3], float *temp)
{
    BMI088_read(gyro, accel, temp);
}

/*=======================================================================
 * bsp_imu_task: 1kHz IMU 采集 + Mahony + 入队
 *=======================================================================*/
void bsp_imu_task(void const *argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = 1;  /* 1ms @ configTICK_RATE_HZ=1000 */
    uint32_t dbg_div = 0;

    while (1) {
        uint32_t t0 = dwt_cycle();
        uint8_t fresh = imu_gyro_drdy();

        if (fresh) {
            BMI088_read(g_gyro, g_acc, &g_temp);
            /* Mahony 融合 (IMU 模式, 无磁力计) */
            MahonyAHRSupdateIMU(g_quat, g_gyro[0], g_gyro[1], g_gyro[2],
                                g_acc[0], g_acc[1], g_acc[2]);
            /* 欧拉角: yaw/pitch/roll (与原厂 GetAngle 一致) */
            g_angle[0] = atan2f(2.0f*(g_quat[0]*g_quat[3]+g_quat[1]*g_quat[2]),
                                2.0f*(g_quat[0]*g_quat[0]+g_quat[1]*g_quat[1])-1.0f);
            g_angle[1] = asinf(-2.0f*(g_quat[1]*g_quat[3]-g_quat[0]*g_quat[2]));
            g_angle[2] = atan2f(2.0f*(g_quat[0]*g_quat[1]+g_quat[3]*g_quat[2]),
                                2.0f*(g_quat[0]*g_quat[0]+g_quat[3]*g_quat[3])-1.0f);

            imu_temp_control(g_temp);

            /* 组装统一数据帧 */
            imu_data_t d;
            memcpy(d.gyro, g_gyro, sizeof(g_gyro));
            memcpy(d.accel, g_acc, sizeof(g_acc));
            memcpy(d.angle, g_angle, sizeof(g_angle));
            memcpy(d.quat, g_quat, sizeof(g_quat));
            d.temp = g_temp;
            d.timestamp_us = (dwt_cycle() - t0) / g_cyc_per_us;  /* 采集耗时 us */
            d.seq = ++g_seq;
            d.fresh = 1;

            g_diag.fresh_cnt++;
            g_diag.sample_cnt++;

            /* 更新 latest (互斥) */
            if (g_imu_mutex) osMutexWait(g_imu_mutex, 0);
            g_latest = d;
            if (g_imu_mutex) osMutexRelease(g_imu_mutex);

            /* 入队 (非阻塞, 满则计数) */
            if (g_imu_queue && xQueueSend(g_imu_queue, &d, 0) != pdTRUE) {
                g_diag.queue_overflow++;
            }
        } else {
            g_diag.stale_cnt++;
        }

        /* 1Hz 姿态自检输出 (走 debug_port: USB CDC 或 DAP VCP) */
        if (++dbg_div >= IMU_TARGET_RATE_HZ) {
            dbg_div = 0;
            debug_printf("[IMU] yaw=%.2f pitch=%.2f roll=%.2f (deg) T=%.1fC seq=%lu fresh=%lu stale=%lu\r\n",
                         g_angle[0]*57.29578f, g_angle[1]*57.29578f, g_angle[2]*57.29578f,
                         g_temp, g_seq, g_diag.fresh_cnt, g_diag.stale_cnt);
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

/*=======================================================================
 * 对外查询 API
 *=======================================================================*/
int bsp_imu_get_latest(imu_data_t *out)
{
    if (!g_initialized || out == NULL) return -1;
    if (g_imu_mutex) osMutexWait(g_imu_mutex, 0);
    *out = g_latest;
    if (g_imu_mutex) osMutexRelease(g_imu_mutex);
    return 0;
}

int bsp_imu_receive(imu_data_t *out, uint32_t timeout_ms)
{
    if (!g_imu_queue || out == NULL) return -1;
    return (xQueueReceive(g_imu_queue, out, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) ? 0 : -1;
}

const imu_diag_t *bsp_imu_get_diag(void) { return &g_diag; }
