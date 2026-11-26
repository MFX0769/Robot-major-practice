/* 主机测试台的模拟硬件：时钟、按键、总线舵机、6020、轮组、片内 Flash。
   片内 Flash 直接映射在 0x08000000 这个地址上，所以固件里直接解引用 Flash 地址的代码原样能跑。 */
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include "sim.h"
#include "arm_task.h"
#include "prot_servo.h"
#include "prot_xb_servo.h"
#include "prot_rm_motor.h"
#include "prot_8irtraking.h"
#include "prot_sbus.h"
#include "mode_switch_task.h"
#include "prot_vision.h"
#include "tfmini.h"
#include "stm32h7xx_hal_flash.h"
#include "stm32h7xx_hal_flash_ex.h"

#define FLASH_BASE   0x08000000UL
#define FLASH_BYTES  0x00100000UL
#define SECTOR_BYTES 0x00020000UL

uint32_t g_now_ms;
int      g_verbose = 1;
int      g_btn_pressed;
int16_t  g_pulse[2] = {1500, 1500};
uint8_t  g_released;
uint8_t  g_servo_online[2] = {1U, 1U};
uint16_t g_gripper_cmd = 1500U;
uint8_t  g_config_active;
int      g_yaw_online = 1;
int      g_yaw_wiring_dir = 1;
int16_t  g_yaw_tx;
float    g_yaw_angle = 3000.0f;
uint32_t g_yaw_frames;
cmd_log_t g_cmd_log[512];
uint32_t g_cmd_log_n;
uint32_t g_flash_erase_cnt;
uint32_t g_flash_program_cnt;
int      g_flag_flash_test;
int16_t  g_motor_tx[4];
uint32_t g_chassis_tx_frames;
uint32_t g_canraw_cnt;
uint32_t g_canraw_ch;
uint32_t g_canraw_id;
uint32_t g_canraw_n;
uint8_t  g_canraw_d[8];
sim_delayuntil_fn g_delayuntil_hook;
char     g_out[G_OUT_CAP];
uint32_t g_out_len;
int      g_capture;
/* 模拟的前向测距模块：g_tfmini_hz > 0 时每秒发这么多帧 */
int      g_tfmini_hz;
uint16_t g_tfmini_dist;
uint16_t g_tfmini_strength;
int      g_wheel_lock;              /* 1 = 轮子卡住不转（测超时用） */

/* 固件里声明的全局对象，在这里给出定义 */
servo_param_t servo_param[SERVO_NUM];
vision_fruit_msg_t vision_fruit;
volatile uint32_t vision_uart_rx_bytes;
volatile uint32_t vision_uart_rx_frames;
volatile uint32_t vision_uart_last_tick;
ctrl_mode_e ctrl_mode;
uint32_t g_sim_primask;

/* chassis_2006[4]、arm_yaw_motor、dji_send_current、wbus_rc 都来自真实固件文件
   （prot_rm_motor.c / prot_sbus.c），所以电机怎么打包进 CAN 帧、遥控开关怎么映射，测的就是上板的那份代码。 */

static uint8_t  s_user_mode_valid;
static int16_t  s_target[2] = {1500, 1500};
static int16_t  s_start[2] = {1500, 1500};
static uint32_t s_t0[2];
static uint32_t s_dur[2] = {20U, 20U};
static uint8_t  s_moving[2];
static float    s_yaw_speed;
static uint8_t *s_flash;
static int32_t  s_wheel_tgt[4];
static prot_8irtraking_data_t s_line;

/* ---------------- 基础桩 ---------------- */
void Error_Handler(void)
{
    printf("\n!!! Error_Handler called !!!\n");
    exit(2);
}

uint32_t HAL_GetTick(void)
{
    return g_now_ms;
}

GPIO_PinState HAL_GPIO_ReadPin(void *port, uint16_t pin)
{
    (void)port;
    (void)pin;
    return g_btn_pressed ? GPIO_PIN_RESET : GPIO_PIN_SET;
}

void HAL_GPIO_Init(void *port, GPIO_InitTypeDef *g)
{
    (void)port;
    (void)g;
}

uint32_t osKernelSysTick(void)
{
    return g_now_ms;
}

void osDelay(uint32_t ms)
{
    sim_advance(ms);
}

int osDelayUntil(uint32_t *ticks, uint32_t ms)
{
    if (g_delayuntil_hook != NULL) return g_delayuntil_hook(ticks, ms);
    return 0;                           /* 机械臂测试不跑 arm_task() 的死循环 */
}

/* 与 mode_switch_task.c 里的实现一致 */
void mode_user_set(ctrl_mode_e mode)
{
    s_user_mode_valid = 1U;
    ctrl_mode = mode;
}

void mode_user_clear(void)
{
    s_user_mode_valid = 0U;
    ctrl_mode = PROTECT_MODE;
}

uint8_t mode_user_active(void)
{
    return s_user_mode_valid;
}

int dbg_print(const char *fmt, ...)
{
    char buf[512];
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return 0;
    if (strstr(buf, "flash test PASS") != NULL) g_flag_flash_test = 1;
    else if (strstr(buf, "flash test FAIL") != NULL) g_flag_flash_test = 2;
    if (g_capture != 0) {
        int room = (int)sizeof(g_out) - 1 - (int)g_out_len;
        if (room > 0) {
            int k = (n < room) ? n : room;
            memcpy(g_out + g_out_len, buf, (size_t)k);
            g_out_len += (uint32_t)k;
            g_out[g_out_len] = '\0';
        }
    }
    if (g_verbose) fputs(buf, stdout);
    return n;
}

void sim_out_clear(void)
{
    g_out_len = 0U;
    g_out[0] = '\0';
}

int sim_out_has(const char *s)
{
    return (strstr(g_out, s) != NULL) ? 1 : 0;
}

/* ---------------- 被控对象 ---------------- */
static void plant_1ms(void)
{
    uint8_t j;

    for (j = 0U; j < 2U; j++) {
        if (g_released == 0U && s_moving[j] != 0U) {
            uint32_t el = g_now_ms - s_t0[j];
            float r = (el >= s_dur[j]) ? 1.0f : (float)el / (float)s_dur[j];
            g_pulse[j] = (int16_t)((float)s_start[j] + (float)(s_target[j] - s_start[j]) * r);
        }
    }

    {
        /* 6020：满电压对应 400rpm，一阶响应时间常数约 20ms */
        float v = (float)g_yaw_tx / 25000.0f;
        float tgt = v * 400.0f * (float)g_yaw_wiring_dir;
        s_yaw_speed += (tgt - s_yaw_speed) * 0.05f;
        g_yaw_angle += s_yaw_speed * (8192.0f / 60000.0f);
        if (g_yaw_angle < 0.0f) g_yaw_angle += 8192.0f;
        if (g_yaw_angle >= 8192.0f) g_yaw_angle -= 8192.0f;
        if (g_yaw_online) {
            arm_yaw_motor.ecd = (uint16_t)g_yaw_angle;
            arm_yaw_motor.speed_rpm = (int16_t)s_yaw_speed;
            arm_yaw_motor.receive_cnt++;
            arm_yaw_motor.online = 1U;
        }
    }

    for (j = 0U; j < 4U; j++) {
        /* 2006：满电流对应约 3000rpm，时间常数约 50ms */
        if (g_wheel_lock != 0) {
            chassis_2006[j].speed_rpm = 0;
            continue;
        }
        float tgt = (float)s_wheel_tgt[j] * 0.3f;
        float spd = (float)chassis_2006[j].speed_rpm;
        spd += (tgt - spd) * 0.02f;
        chassis_2006[j].speed_rpm = (int16_t)spd;
    }
}

/* 往 UART9 的接收寄存器里灌原始字节（测校验、错位、重同步用） */
void sim_tfmini_bytes(const uint8_t *buf, uint32_t n)
{
    uint32_t i;

    for (i = 0U; i < n; i++) {
        sim_uart_rx_push(9, buf[i]);
        tfmini_rx_isr();
    }
}

/* 模拟模块发一帧：dist 是帧里的原始值（出厂单位 cm） */
void sim_tfmini_frame(uint16_t dist, uint16_t strength)
{
    uint8_t b[9];
    uint8_t i, sum = 0U;

    b[0] = 0x59U;
    b[1] = 0x59U;
    b[2] = (uint8_t)(dist & 0xFFU);
    b[3] = (uint8_t)(dist >> 8);
    b[4] = (uint8_t)(strength & 0xFFU);
    b[5] = (uint8_t)(strength >> 8);
    b[6] = 0x20U;
    b[7] = 0x01U;
    for (i = 0U; i < 8U; i++) sum = (uint8_t)(sum + b[i]);
    b[8] = sum;
    sim_tfmini_bytes(b, 9U);
}

void sim_advance(uint32_t ms)
{
    uint32_t i;
    static uint32_t ms_cnt;

    for (i = 0U; i < ms; i++) {
        g_now_ms++;
        plant_1ms();
        ms_cnt++;
        if (g_tfmini_hz > 0 && (ms_cnt % (uint32_t)(1000 / g_tfmini_hz)) == 0U) {
            sim_tfmini_frame(g_tfmini_dist, g_tfmini_strength);
        }
    }
}

void sim_set_pulse(uint8_t joint, int16_t pulse)
{
    if (joint < 2U) {
        g_pulse[joint] = pulse;
        s_moving[joint] = 0U;          /* 手拖着走，取消未完成的运动 */
    }
}

static void log_cmd(uint8_t joint, uint16_t pulse, uint16_t time_ms)
{
    if (g_cmd_log_n < 512U) {
        cmd_log_t *c = &g_cmd_log[g_cmd_log_n++];
        c->t = g_now_ms;
        c->joint = joint;
        c->pulse = pulse;
        c->time_ms = time_ms;
    }
}

/* ---------------- 舵机接口桩 ---------------- */
uint8_t xb_servo_read_pos(uint8_t joint, uint16_t *pulse)
{
    if (joint >= 2U || pulse == NULL) return 1U;
    sim_advance(2U);                    /* 一次查询的时间 */
    if (g_servo_online[joint] == 0U) return 3U;
    *pulse = (uint16_t)g_pulse[joint];
    return 0U;
}

uint8_t xb_servo_set_pos(uint8_t joint, uint16_t pulse, uint16_t time_ms)
{
    if (joint >= 2U) return 1U;
    if (pulse < XB_PULSE_MIN || pulse > XB_PULSE_MAX) return 1U;
    s_target[joint] = (int16_t)pulse;
    s_start[joint] = g_pulse[joint];
    s_t0[joint] = g_now_ms;
    s_dur[joint] = (time_ms < 20U) ? 20U : time_ms;
    s_moving[joint] = 1U;
    g_released = 0U;                    /* 位置命令会让舵机重新出力 */
    log_cmd(joint, pulse, time_ms);
    return 0U;
}

void servo_set_pulse_time(uint8_t id, uint16_t pulse_us, uint16_t time_ms)
{
    if (id >= SERVO_NUM) return;
    if (pulse_us < SERVO_PULSE_MIN) pulse_us = SERVO_PULSE_MIN;
    if (pulse_us > SERVO_PULSE_MAX) pulse_us = SERVO_PULSE_MAX;

    if (id < 2U) {
        (void)xb_servo_set_pos(id, pulse_us, time_ms);
    } else {
        g_gripper_cmd = pulse_us;
        log_cmd(2U, pulse_us, 0U);
    }
    servo_param[id].current_pulse = pulse_us;
}

void servo_set_pulse(uint8_t id, uint16_t pulse_us)
{
    servo_set_pulse_time(id, pulse_us, 20U);
}

uint16_t servo_get_current_pulse(uint8_t id)
{
    return (id < 2U) ? (uint16_t)g_pulse[id] : g_gripper_cmd;
}

uint8_t servo_online(uint8_t id)
{
    return (id < 2U) ? g_servo_online[id] : 1U;
}

uint8_t servo_teach_begin(void)
{
    g_released = 1U;
    return 0U;
}

void servo_teach_end(void)
{
    uint8_t j;
    for (j = 0U; j < 2U; j++) {
        s_target[j] = g_pulse[j];
        s_start[j] = g_pulse[j];
        s_moving[j] = 0U;
    }
    g_released = 0U;
}

void servo_init(void) {}
void servo_poll(void) {}
void servo_set_enabled(uint8_t enabled) { (void)enabled; }
uint8_t servo_config_active(void) { return g_config_active; }

/* ---------------- CAN / 视觉桩 ---------------- */
void vision_uart_init(void) {}

/* 与 Comm/can_comm.c 里的 chassis_can_send 是同一条语句；帧的打包走真实的 dji_send_current */
void chassis_can_send(void)
{
    uint8_t i;

    dji_send_current(&chassis_2006[0], &chassis_2006[1], &chassis_2006[2], &chassis_2006[3]);
    for (i = 0U; i < 4U; i++) {
        g_motor_tx[i] = chassis_2006[i].tx_current;
        s_wheel_tgt[i] = chassis_2006[i].tx_current;
    }
    g_chassis_tx_frames++;
}

void prot_8irtraking_init(void) {}
void prot_8ir_uart_diag(void) {}
const prot_8irtraking_data_t *prot_8irtraking_get_data(void) { return &s_line; }

void sim_wheel_set_rpm(int idx, int rpm)
{
    if (idx >= 0 && idx < 4) chassis_2006[idx].speed_rpm = (int16_t)rpm;
}

void sim_line_set(uint8_t state, int16_t position)
{
    s_line.state = state;
    s_line.position = position;
}

/* ---------------- 片内 Flash 桩 ---------------- */
HAL_StatusTypeDef HAL_FLASH_Unlock(void) { return HAL_OK; }
HAL_StatusTypeDef HAL_FLASH_Lock(void) { return HAL_OK; }

HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *e, uint32_t *sector_error)
{
    uint32_t addr = FLASH_BASE + e->Sector * SECTOR_BYTES;

    if (s_flash == NULL || e->Sector > 7U) return HAL_ERROR;
    memset((void *)(uintptr_t)addr, 0xFF, SECTOR_BYTES);
    *sector_error = 0xFFFFFFFFU;
    g_flash_erase_cnt++;
    g_now_ms += 1000U;                  /* 擦除时 CPU 取指被停住：只拨时钟，被控对象照常走 */
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t addr, uint32_t data_addr)
{
    if (type != FLASH_TYPEPROGRAM_FLASHWORD || (addr & 31U) != 0U) return HAL_ERROR;
    if (addr < FLASH_BASE || addr >= FLASH_BASE + FLASH_BYTES) return HAL_ERROR;
    memcpy((void *)(uintptr_t)addr, (const void *)(uintptr_t)data_addr, 32U);
    g_flash_program_cnt++;
    g_now_ms += 1U;
    return HAL_OK;
}

void sim_reset(void)
{
    s_flash = (uint8_t *)VirtualAlloc((LPVOID)FLASH_BASE, FLASH_BYTES, MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE);
    if (s_flash != (uint8_t *)FLASH_BASE) {
        printf("sim: cannot map flash at 0x%08lX (got %p)\n", FLASH_BASE, (void *)s_flash);
        exit(3);
    }
    memset(s_flash, 0xFF, FLASH_BYTES);
    g_now_ms = 1000U;
    g_btn_pressed = 0;
    g_pulse[0] = 1500;
    g_pulse[1] = 1500;
    g_released = 0U;
    g_gripper_cmd = 1500U;
    g_yaw_tx = 0;
    g_yaw_angle = 3000.0f;
    s_yaw_speed = 0.0f;
    g_cmd_log_n = 0U;
    g_flash_erase_cnt = 0U;
    g_flash_program_cnt = 0U;
    g_flag_flash_test = 0;
    g_motor_tx[0] = g_motor_tx[1] = g_motor_tx[2] = g_motor_tx[3] = 0;
    g_chassis_tx_frames = 0U;
    g_canraw_cnt = 0U;
    g_canraw_n = 0U;
    memset(&s_wheel_tgt, 0, sizeof(s_wheel_tgt));
    memset(&s_line, 0, sizeof(s_line));
    memset(&chassis_2006, 0, sizeof(chassis_2006));
    memset(&arm_yaw_motor, 0, sizeof(arm_yaw_motor));
    ctrl_mode = PROTECT_MODE;
    g_sim_primask = 0U;
    g_tfmini_hz = 0;
    g_tfmini_dist = 0U;
    g_tfmini_strength = 800U;
    g_wheel_lock = 0;
    huart9.uart = 9;

    /* 与 Comm/can_comm.c 的 comm_can_init 相同：can_id 决定电机落在 0x200/0x1FF 帧的哪个槽位 */
    dji_motor_init(&arm_yaw_motor, DJI_6020_MOTOR, PROT_CAN_CHANNEL_2, 0x208U, 1.0f);
    dji_motor_init(&chassis_2006[0], DJI_2006_MOTOR, PROT_CAN_CHANNEL_1, CHA_LEFT_FRONT_2006, DJI_2006_ORIGINAL_REDUCTION_RATIO);
    dji_motor_init(&chassis_2006[1], DJI_2006_MOTOR, PROT_CAN_CHANNEL_1, CHA_LEFT_BACK_2006, DJI_2006_ORIGINAL_REDUCTION_RATIO);
    dji_motor_init(&chassis_2006[2], DJI_2006_MOTOR, PROT_CAN_CHANNEL_1, CHA_RIGHT_FRONT_2006, DJI_2006_ORIGINAL_REDUCTION_RATIO);
    dji_motor_init(&chassis_2006[3], DJI_2006_MOTOR, PROT_CAN_CHANNEL_1, CHA_RIGHT_BACK_2006, DJI_2006_ORIGINAL_REDUCTION_RATIO);
}
