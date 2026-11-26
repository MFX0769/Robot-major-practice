/**
  * @file    chassis_task.c
  * @brief   搴曠洏鎺у埗浠诲姟 (楹﹀厠绾冲锟?
  * @note    鍙傦拷?RoboticMajor_Pratice_code
  */

#include "chassis_task.h"
#include "pid.h"
#include "mode_switch_task.h"
#include "control_def.h"
#include "prot_rm_motor.h"
#include "can_comm.h"
#include "cmsis_os.h"
#include "config.h"
#include "prot_sbus.h"
#include "prot_8irtraking.h"
#include "arm_task.h"
#include "tfmini.h"
#include <stdlib.h>
#include <string.h>

/* 澶栭儴鐢垫満鍙橀噺 */
extern dji_motor_t chassis_2006[4];

/* 搴曠洏鐢垫満缁撴瀯锟?*/
chassis_motor_t chassis_motor;
extern volatile uint8_t arm_busy;

/* 搴曠洏鐢垫満閫熷害鐜疨ID锛屾寜chassis_2006[]鐨勪笅锟?宸﹀墠/宸﹀悗/鍙冲墠/鍙冲悗) */
static pid_t chassis_motor_pid[4];

/* 绾夸綅璺熻釜妯″紡 - 杞悜PID */
static pid_t line_track_pid;

/**
  * @brief  搴曠洏鍙傛暟鍒濆锟?  */
static void chassis_init(void)
{
    memset(&chassis_motor, 0, sizeof(chassis_motor_t));
    
    /* 4涓數鏈洪�熷害鐜疨ID鍒濆锟?*/
    for(int i = 0; i < 4; i++)
    {
        PID_struct_init(&chassis_motor_pid[i], 8000, 1600.0f, 5.0f, 0.0f, 1.0f, 0, 0, 0, NONE_IMPROVE);
    }
    
    /* 绾夸綅璺熻釜PID鍒濆鍖栵細maxout=1500璺熼仴鎺ф弧杞噺绾ф帴锟?ch4婊″亸脳CHASSIS_SCALE_WZ锟?640)锟?       kp=-15閰嶅悎position鏈�澶э拷?00锛宬p*error姝ｅソ鎵撴弧maxout锛涜礋鍙锋槸瀹炴祴纭杩囩殑绾犲亸鏂瑰悜锟?       鍘熸潵澶栭儴涔樼殑-150绯绘暟宸茬粡鍘绘帀锛屾柟鍚戜俊鎭尓鍒拌繖锟?*/
    /* 2026-11-26: kp -50 -> -12. wz = 50*position meant a quarter-scale error already steered as
       hard as the base speed (research 20261126_调研结果_绕障回线.md section 8); -12 gives
       wz = base speed only at full deflection. 'chassis pid line' tunes it live. */
    PID_struct_init(&line_track_pid, 150000, 150.0f, -12.0f, 0.0f, 0.0f, 0, 0, 0, NONE_IMPROVE);
}

/**
  * @brief  楹﹀厠绾冲杞繍鍔ㄥ瑙ｇ畻
  * @param  vx: 鍓嶈繘閫熷害
  * @param  vy: 宸﹀彸閫熷害
  * @param  wz: 鏃嬭浆瑙掗�熷害
  * @param  L: 搴曠洏杞磋窛绯绘暟
  */
static void chassis_mecanum_resolve(float vx, float vy, float wz, float L)
{
    /*
    搴曠洏鍧愭爣绯伙細
    FL --- FR
    |       |
    BL --- BR
    
    楹﹀厠绾冲杞В绠楋細
    */
    chassis_motor.motor[0] = (int16_t)(vx + vy + L * wz);    /* 鍓嶅乏 FL */
    chassis_motor.motor[1] = (int16_t)(vx - vy - L * wz);    /* 鍓嶅彸 FR */
    chassis_motor.motor[2] = (int16_t)(vx - vy + L * wz);    /* 鍚庡乏 BL */
    chassis_motor.motor[3] = (int16_t)(vx + vy - L * wz);    /* 鍚庡彸 BR */
}

/**
  * @brief  搴曠洏鎺у埗浠诲姟
  */
void chassis_task(void const *argu)
{
    uint32_t mode_wake_time = osKernelSysTick();
    chassis_init();

    /* 宸＄嚎妯″潡鏀圭敤绾疓PIO璇绘暟瀛楅噺锛屼笉鍐嶈蛋UART鍗忚 */
    prot_8irtraking_init();
    tfmini_init();              /* front rangefinder on UART9; nothing connected = stays offline */
    chassis_av_init();

    for(;;)
    {
        mode_wake_time = osKernelSysTick();
        prot_8ir_uart_diag();

        float vx_ref = 0.0f;
        float vy_ref = 0.0f;
        float wz_ref = 0.0f;
        const prot_8irtraking_data_t *line_data = prot_8irtraking_get_data();

        /* 搴曠洏涓撶敤妯″紡锛氱敱绗簩涓嫧鏉唖w2(ch7)鐩存帴椹卞姩锛岃窡鍏ㄥ眬ctrl_mode鏄嫭绔嬬殑涓や釜姒傚康 */
        static chassis_mode_e prev_mode = CHASSIS_STOP;
        chassis_mode_e chassis_mode;
        switch (wbus_rc.data.sw2)
        {
            case 2:  chassis_mode = CHASSIS_REMOTER;    break;
            case 0:  chassis_mode = CHASSIS_LINE_TRACK; break;
            default: chassis_mode = CHASSIS_STOP;       break;
        }
        if (chassis_mode != prev_mode) {
            prev_mode = chassis_mode;
        }
        if (chassis_mode != CHASSIS_LINE_TRACK) chassis_av_cancel();

        taskENTER_CRITICAL();
        chassis_odo_step();

        switch(chassis_mode)
        {
            case CHASSIS_STOP:
            {
                /* 闈欐 - 缁欐満姊拌噦璋冭瘯鐢紝涓嶉渶瑕佸簳鐩樺姩 */
                vx_ref = 0.0f;
                vy_ref = 0.0f;
                wz_ref = 0.0f;
            }break;

            case CHASSIS_REMOTER:
            {
                if (arm_busy) {
                    vx_ref = vy_ref = wz_ref = 0.0f;
                } else if (wbus_rc.data.sw1 == 0 && wbus_rc.data.sw3) {
                    vx_ref = vy_ref = wz_ref = 0.0f;
                } else {
                    /* DBUS/DR16 解码（prot_dr16.c）：ch1 右横(左=-) ch2 右竖(上=+) ch3 左横(左=-) ch4 左竖(上=+)。
                       右摇杆 = 平移，左摇杆横 = 旋转。装车后哪个方向反了用 'chassis map vx|vy|wz -1' 翻。 */
                    vx_ref = wbus_rc.data.ch2 * CHASSIS_SCALE_VX * (float)g_chassis_map_sign[0];
                    vy_ref = wbus_rc.data.ch1 * CHASSIS_SCALE_VY * (float)g_chassis_map_sign[1];
                    wz_ref = wbus_rc.data.ch3 * CHASSIS_SCALE_WZ * (float)g_chassis_map_sign[2];
                }
            }break;

            case CHASSIS_LINE_TRACK:
            {
                /* 2026-11-23 模拟量巡线: position=8路灰度加权质心(±100)
                   - 正常: PID 纠偏 + 偏差越大越慢(弯道减速)
                   - 全压线(十字): 直行冲过
                   - 丢线: position 保持记忆值 → 按记忆方向给固定转向量,
                     解决"转弯只出现半秒就冲过去、再无纠正量"; 超时停车保护 */
                static uint16_t lost_ms = 0;
                static uint16_t track_time = 0;
                if (track_time < 0xFFFF) track_time++;
                vx_ref = 1200.0f;

                if (line_data != NULL && track_time > 1000)
                {
                    if (line_data->state == LINE_STATE_OK)
                    {
                        lost_ms = 0;
                        float pos_abs = (float)line_data->position;
                        if (pos_abs < 0.0f) pos_abs = -pos_abs;
                        vx_ref = 1200.0f - pos_abs * 5.0f;   /* |pos|=100 → 700 */
                        if (vx_ref < 400.0f) vx_ref = 400.0f;
                        wz_ref = pid_calc(&line_track_pid, (float)line_data->position, 0.0f);
                    }
                    else if (line_data->state == LINE_STATE_ALLON)
                    {
                        lost_ms = 0;                          /* 十字: 直行冲过 */
                    }
                    else   /* LINE_STATE_LOST: 记忆转向 */
                    {
                        if (lost_ms < 700U) lost_ms += CHASSIS_PERIOD;
                        if (lost_ms < 700U) {
                            /* 朝线最后出现的同侧找：纠偏是 pos>0 -> 正 wz（kp<0，err=ref-fdb），
                               丢线也用同号；dir 跟随 kp 符号，'chassis pid line' 改了它自动跟着翻 */
                            float dir = (line_track_pid.p < 0.0f) ? 1.0f : -1.0f;
                            vx_ref = 350.0f;
                            wz_ref = dir * ((line_data->position >= 0) ? 900.0f : -900.0f);
                        } else {                              /* 丢线超时: 停 */
                            vx_ref = 0.0f;
                            wz_ref = 0.0f;
                        }
                    }
                }
            /* 神之一手: 手动随时可救场（左摇杆横） */
            wz_ref += wbus_rc.data.ch3 * CHASSIS_SCALE_WZ * (float)g_chassis_map_sign[2];
            chassis_av_tick(&vx_ref, &vy_ref, &wz_ref, line_data);   /* obstacle sidestep overrides the refs */
        }break;
            default:break;
        }

        /* 巡线中左摇杆竖向拨过 ±300（边沿 + 1.2s 防抖）= 触发/停止表演动作 */
        {
            static int16_t s_trig_prev = 0;
            static uint32_t s_trig_last = 0U;
            int16_t ch4 = wbus_rc.data.ch4;
            int16_t a4 = (ch4 < 0) ? (int16_t)(-ch4) : ch4;
            int16_t ap = (s_trig_prev < 0) ? (int16_t)(-s_trig_prev) : s_trig_prev;
            uint32_t nowt = osKernelSysTick();

            if (chassis_mode == CHASSIS_LINE_TRACK && a4 >= 300 && ap < 300 &&
                (int32_t)(nowt - s_trig_last) >= 1200) {
                s_trig_last = nowt;
                arm_pick_remote_trig();
            }
            s_trig_prev = ch4;
        }

        /* arm_busy锟斤拷锟斤拷锟斤拷锟斤拷模式 */
        /* zero the reference itself whenever the output is forced to zero (stop / PROTECT / arm busy).
           Otherwise the wheel PIDs keep a phantom 1200rpm reference while in PROTECT and emit a
           one-loop 1200-count D kick the moment the car leaves PROTECT. */
        if (arm_busy || chassis_mode == CHASSIS_STOP || ctrl_mode == PROTECT_MODE) vx_ref = vy_ref = wz_ref = 0.0f;

        /* 杩愬姩瀛﹁В锟?*/
        chassis_mecanum_resolve(vx_ref, vy_ref, wz_ref, MECANUM_L);

        /* chassis_2006[]涓嬫爣(宸﹀墠/宸﹀悗/鍙冲墠/鍙冲悗) 瀵瑰簲鐨勮繍鍔ㄥ鐩爣
           鍙宠竟涓や釜杞瓙(鍙冲墠/鍙冲悗)闇�瑕佸弽锟?*/
        float target_rpm[4] = {
            (float)chassis_motor.motor[0],   /* 宸﹀墠 */
            (float)chassis_motor.motor[2],   /* 宸﹀悗 */
            -(float)chassis_motor.motor[1],  /* 鍙冲墠 (鍙嶅悜) */
            -(float)chassis_motor.motor[3],  /* 鍙冲悗 (鍙嶅悜) */
        };

        /* 閫熷害鐜疨ID闂幆锛屾渶鍚庡己鍒朵繚鎶わ細搴曠洏闈欐鎴栧叏灞�淇濇姢妯″紡锛屾棤鏉′欢锟?锛屼笉绠ID绠楀嚭浠�锟?*/
        uint8_t force_zero = (chassis_mode == CHASSIS_STOP) || (ctrl_mode == PROTECT_MODE);
        for (int i = 0; i < 4; i++)
        {
            float pid_out = pid_calc(&chassis_motor_pid[i], chassis_2006[i].speed_rpm, target_rpm[i]);
            if (force_zero)
            {
                chassis_2006[i].tx_current = 0;
            }
            else
            {
                chassis_2006[i].tx_current = (int16_t)pid_out;
            }
        }

        chassis_dbg_jog_apply(chassis_2006);
        taskEXIT_CRITICAL();
        
        /* 閫氳繃闃熷垪鍙戦�佺數鏈虹數锟?*/
        chassis_can_send();   /* 2ms period -> 500Hz, every loop */
        
        
        osDelayUntil(&mode_wake_time, CHASSIS_PERIOD);
    }
}

/* ==================== console debug hooks (2026-11-26) ==================== */
/* 摇杆方向微调：DBUS 装车后某个方向反了用 'chassis map vx|vy|wz -1' 翻，
   值存在机械臂标定块 reserved[0..2]（'pick save' 后掉电保留）。 */
int8_t g_chassis_map_sign[3] = {1, 1, 1};

void chassis_map_sign_set(uint8_t idx, int8_t sign)
{
    if (idx > 2U || (sign != 1 && sign != -1)) return;
    g_chassis_map_sign[idx] = sign;
}

/* Open-loop jog for ONE motor: console 'chassis motor <idx> <cur>'. Auto timeout. */
static int8_t   s_jog_idx = -1;
static int16_t  s_jog_cur = 0;
static uint32_t s_jog_until = 0U;
static uint32_t s_jog_applied = 0U;

/* called from the control loop inside its critical section */
void chassis_dbg_jog_apply(dji_motor_t *m)
{
    int i;

    if (s_jog_idx < 0) return;
    if ((int32_t)(osKernelSysTick() - s_jog_until) >= 0) {
        s_jog_idx = -1;
        return;
    }
    for (i = 0; i < 4; i++) {
        m[i].tx_current = (i == s_jog_idx) ? s_jog_cur : 0;
    }
    s_jog_applied++;
}

int chassis_dbg_jog_start(int32_t idx, int32_t cur)
{
    if (idx < 0 || idx > 3 || cur == 0) return -1;
    if (cur > (int32_t)CHASSIS_JOG_MAX_CUR) cur = (int32_t)CHASSIS_JOG_MAX_CUR;
    if (cur < -(int32_t)CHASSIS_JOG_MAX_CUR) cur = -(int32_t)CHASSIS_JOG_MAX_CUR;
    taskENTER_CRITICAL();
    s_jog_idx = (int8_t)idx;
    s_jog_cur = (int16_t)cur;
    s_jog_until = osKernelSysTick() + CHASSIS_JOG_MS;
    s_jog_applied = 0U;
    taskEXIT_CRITICAL();
    return 0;
}

void chassis_dbg_jog_stop(void)
{
    taskENTER_CRITICAL();
    s_jog_idx = -1;
    taskEXIT_CRITICAL();
}

uint8_t chassis_dbg_jog_active(void)
{
    return (s_jog_idx >= 0) ? 1U : 0U;
}

pid_t *chassis_dbg_pid_get(uint8_t i)
{
    return (i < 4U) ? &chassis_motor_pid[i] : 0;
}

pid_t *chassis_dbg_line_pid_get(void)
{
    return &line_track_pid;
}

/* ==================== front rangefinder avoidance (2026-11-26) ====================
   One TF-mini on UART9 (PD14) pointing forward. While line following:
   obstacle nearer than app_mm for 3 frames -> stop 300ms -> strafe to the chosen side by
   shift_mm -> forward fwd_mm -> strafe back -> pick the line up again.
   Distances come from integrating the wheel speeds, so mm-per-rpm has to be calibrated once:
   'avoid odo', drive a known distance straight, 'avoid calib <measured mm>'.
   Open-source references: Robot-major-practice/Develop_Log/20261126_调研结果_绕障回线.md */

typedef enum {
    AV_IDLE = 0,      /* watching the rangefinder */
    AV_STOP,          /* brake before the sidestep */
    AV_OUT,           /* strafe away from the line */
    AV_AHEAD,         /* pass the obstacle */
    AV_BACK,          /* strafe back toward the line */
    AV_RELINE,        /* line not seen yet: keep easing back, up to a limit */
    AV_ABORT          /* gave up: stand still until line-follow mode is left or 'avoid run' */
} av_state_e;

#define AV_DEBOUNCE_FRAMES    3U
#define AV_STOP_MS            300U
#define AV_LEG_TIMEOUT_MS     8000U
#define AV_RELINE_EXTRA_MM    150U
#define AV_COOLDOWN_MS        1500U
#define AV_VY_RPM             600.0f
#define AV_VX_RPM             600.0f
#define AV_SEARCH_VY_RPM      250.0f
#define AV_DFLT_KODO_X1000    221U     /* mm per (rotor rpm * s): pi * 152mm / 36 / 60 */
#define AV_DFLT_SHIFT_MM      300U
#define AV_DFLT_FWD_MM        450U
#define AV_DFLT_APP_MM        300U
#define AV_DIST_MAX_MM        12000U

typedef struct {
    uint8_t  disabled;          /* 'avoid off' */
    uint8_t  force;             /* 'avoid run': start on the next line-follow loop */
    uint8_t  state;
    uint8_t  unit_mm;           /* rangefinder reports mm (0 = cm, the TF-mini default) */
    uint8_t  near_cnt;
    int8_t   side;              /* +1 = shift toward the stick-left side, -1 = the other side */
    uint32_t last_frame;
    uint32_t state_tick;
    uint32_t cool_tick;
    int32_t  leg_x_um;          /* signed distance covered in the current leg */
    int32_t  leg_y_um;
    int32_t  cal_rpms_x100;     /* running integral of forward speed, rpm*s * 100 */
    uint16_t kodo_x1000;        /* mm per (rpm * s) * 1000 */
    uint16_t shift_mm;
    uint16_t fwd_mm;
    uint16_t app_mm;
} av_t;

static av_t s_av;

static const char *av_state_name(uint8_t st)
{
    switch (st) {
        case AV_STOP:   return "STOP";
        case AV_OUT:    return "STRAFE_OUT";
        case AV_AHEAD:  return "AHEAD";
        case AV_BACK:   return "STRAFE_BACK";
        case AV_RELINE: return "RELINE";
        case AV_ABORT:  return "ABORT";
        default:        return "IDLE";
    }
}

static uint16_t av_range(uint16_t v, uint16_t lo, uint16_t hi, uint16_t dflt)
{
    return (v >= lo && v <= hi) ? v : dflt;
}

/* 由机械臂任务的 cal_apply 调用：标定块里的值，0 或越界 = 用默认值 */
void chassis_av_apply(uint16_t kodo_x1000, uint16_t shift_mm, uint16_t fwd_mm, int8_t side)
{
    s_av.kodo_x1000 = av_range(kodo_x1000, 20U, 2000U, AV_DFLT_KODO_X1000);
    s_av.shift_mm = av_range(shift_mm, 50U, 800U, AV_DFLT_SHIFT_MM);
    s_av.fwd_mm = av_range(fwd_mm, 50U, 1500U, AV_DFLT_FWD_MM);
    if (side == 1 || side == -1) s_av.side = side;
    if (s_av.app_mm == 0U) s_av.app_mm = AV_DFLT_APP_MM;
}

void chassis_av_init(void)
{
    /* 只在没人（cal_apply）给过值时填默认值，不覆盖已经从 Flash 恢复的标定 */
    if (s_av.kodo_x1000 == 0U) chassis_av_apply(0U, 0U, 0U, 1);
}

/* 每 2ms 一圈：把四轮转速折算成车体前向/横向位移。
   下标 0=左前 1=左后 2=右前 3=右后（右侧电机的正转就是车体后退，和 target_rpm 同号）。
   vx = (FL + BL - FR - BR) / 4，vy = (FL - BL + FR - BR) / 4，和 chassis_mecanum_resolve 互逆。 */
void chassis_odo_step(void)
{
    int32_t fl = chassis_2006[0].speed_rpm, bl = chassis_2006[1].speed_rpm;
    int32_t fr = chassis_2006[2].speed_rpm, br = chassis_2006[3].speed_rpm;
    float vx = (float)(fl + bl - fr - br) * 0.25f;    /* 前向为正 */
    float vy = (float)(fl - bl + fr - br) * 0.25f;    /* 右摇杆右推的那一侧为正 */
    float k = (float)s_av.kodo_x1000 * 0.001f * (float)CHASSIS_PERIOD * 0.001f;   /* mm per rpm per loop */

    s_av.leg_x_um += (int32_t)(vx * k * 1000.0f);
    s_av.leg_y_um += (int32_t)(vy * k * 1000.0f);
    s_av.cal_rpms_x100 += (int32_t)(vx * (float)CHASSIS_PERIOD * 0.1f);
}

/* 传感器读数换算成毫米。返回 0 = 离线或这一帧不可信 */
static uint8_t av_sensor_mm(uint32_t *d_mm, uint32_t *frames)
{
    uint16_t raw, str;

    if (tfmini_online(0U) == 0U) return 0U;
    raw = tfmini[0].dist_raw;
    str = tfmini[0].strength;
    if (raw == 0U || raw == 0xFFFFU || str < TFMINI_STRENGTH_MIN) return 0U;
    *d_mm = (s_av.unit_mm != 0U) ? (uint32_t)raw : (uint32_t)raw * 10U;
    if (*d_mm > AV_DIST_MAX_MM) return 0U;
    *frames = tfmini[0].rx_frames;
    return 1U;
}

static void av_enter(av_state_e st, const char *why)
{
    s_av.state = (uint8_t)st;
    s_av.state_tick = osKernelSysTick();
    s_av.leg_x_um = 0;
    s_av.leg_y_um = 0;
    if (st != AV_IDLE) dbg_print("[AVOID] %s -> %s\r\n", why, av_state_name((uint8_t)st));
}

static void av_finish(const char *why)
{
    av_enter(AV_IDLE, why);
    s_av.cool_tick = osKernelSysTick();
    s_av.near_cnt = 0U;
    dbg_print("[AVOID] %s, line following resumes\r\n", why);
}

static void av_abort(const char *why)
{
    s_av.state = AV_ABORT;
    dbg_print("[AVOID] aborted (%s): standing still. Leave line-follow mode, or 'avoid run' to try again\r\n", why);
}

void chassis_av_cancel(void)
{
    if (s_av.state == AV_IDLE) return;
    s_av.force = 0U;
    av_finish("cancelled");
}

void chassis_av_tick(float *vx, float *vy, float *wz, const prot_8irtraking_data_t *line)
{
    uint32_t now = osKernelSysTick(), d_mm, frames;
    int8_t side = (s_av.side < 0) ? -1 : 1;
    /* "左" = 右摇杆左推时车走的那一侧（'chassis map vy' 校准后二者一致） */
    float left_vy = -(float)g_chassis_map_sign[1] * AV_VY_RPM;
    int32_t shift_um = (int32_t)s_av.shift_mm * 1000;
    int8_t line_ok = (line != NULL && line->state == LINE_STATE_OK) ? 1 : 0;

    /* 空闲时不碰给定：巡线给的 vx/wz 原样往下走 */
    if (s_av.state == AV_IDLE || s_av.state == AV_ABORT) {
        uint8_t go = 0U;

        if (s_av.force != 0U) {
            s_av.force = 0U;
            go = 1U;
        } else if (s_av.disabled == 0U && (uint32_t)(now - s_av.cool_tick) >= AV_COOLDOWN_MS &&
                   av_sensor_mm(&d_mm, &frames) != 0U) {
            if (frames != s_av.last_frame) {              /* 每个新帧只数一次 */
                s_av.last_frame = frames;
                if (d_mm < (uint32_t)s_av.app_mm) {
                    if (s_av.near_cnt < 255U) s_av.near_cnt++;
                } else {
                    s_av.near_cnt = 0U;
                }
            }
            if (s_av.near_cnt >= AV_DEBOUNCE_FRAMES) go = 1U;
        } else if (s_av.state == AV_IDLE) {
            s_av.near_cnt = 0U;
        }
        if (go == 0U) {
            if (s_av.state == AV_ABORT) { *vx = 0.0f; *vy = 0.0f; *wz = 0.0f; }
            return;
        }
        av_enter(AV_STOP, "obstacle ahead");
    }

    *vx = 0.0f;
    *vy = 0.0f;
    *wz = 0.0f;

    if ((uint32_t)(now - s_av.state_tick) >= AV_LEG_TIMEOUT_MS) {
        av_abort("leg timed out");
        return;
    }

    switch (s_av.state) {
        case AV_STOP:
            if ((uint32_t)(now - s_av.state_tick) >= AV_STOP_MS) av_enter(AV_OUT, "strafe out");
            break;
        case AV_OUT:
            *vy = (float)side * left_vy;
            if (-(int32_t)side * s_av.leg_y_um >= shift_um) av_enter(AV_AHEAD, "ahead");
            break;
        case AV_AHEAD:
            *vx = AV_VX_RPM;
            if (s_av.leg_x_um >= (int32_t)s_av.fwd_mm * 1000) av_enter(AV_BACK, "strafe back");
            break;
        case AV_BACK:
            *vy = -(float)side * left_vy;
            /* 必须已经回移大部分，才允许“看见线就结束”。宽障碍/传感器仍看得到线时，
               刚离开 AHEAD 那一刻线可能还可见，不能因此零距离跳回巡线。 */
            if (line_ok != 0 && (int32_t)side * s_av.leg_y_um >= shift_um * 3 / 4) {
                av_finish("back on the line");
                break;
            }
            if ((int32_t)side * s_av.leg_y_um >= shift_um) av_enter(AV_RELINE, "reline");
            break;
        case AV_RELINE:
            if (line_ok != 0) { av_finish("back on the line"); break; }
            *vy = -(float)side * left_vy * (AV_SEARCH_VY_RPM / AV_VY_RPM);
            if ((int32_t)side * s_av.leg_y_um >= (int32_t)AV_RELINE_EXTRA_MM * 1000) av_abort("line not found");
            break;
        default:
            break;
    }
}

/* ---- 控制台 avoid ---- */
static int32_t av_num(const char *s, int32_t lo, int32_t hi, uint8_t *ok)
{
    char *end = NULL;
    long v;

    *ok = 0U;
    if (s == NULL) return 0;
    v = strtol(s, &end, 10);
    if (end == s || *end != '\0' || v < (long)lo || v > (long)hi) return 0;
    *ok = 1U;
    return (int32_t)v;
}

static void av_kodo_set(uint16_t kodo_x1000)
{
    if (kodo_x1000 < 20U || kodo_x1000 > 2000U) return;
    s_av.kodo_x1000 = kodo_x1000;
    arm_ctrl.cal.reserved[3] = kodo_x1000;
}

void chassis_av_cmd(const char *sub, const char *arg)
{
    uint8_t ok = 0U;
    int32_t v;
    uint32_t d_mm, frames;

    if (sub == NULL) goto usage;

    if (strcmp(sub, "status") == 0) {
        if (arg != NULL) goto usage;
        (void)av_sensor_mm(&d_mm, &frames);
        dbg_print("[AVOID] range raw=%u%s strength=%u valid=%u online=%u age=%lums frames=%lu crc_err=%lu\r\n",
                  (unsigned)tfmini[0].dist_raw, (s_av.unit_mm != 0U) ? "mm" : "cm",
                  (unsigned)tfmini[0].strength, (unsigned)tfmini_frame_valid(0U), (unsigned)tfmini_online(0U),
                  (unsigned long)(tfmini[0].last_tick != 0U ? osKernelSysTick() - tfmini[0].last_tick : 0xFFFFFFFFU),
                  (unsigned long)tfmini[0].rx_frames, (unsigned long)tfmini[0].crc_errors);
        dbg_print("[AVOID] state=%s on=%u app=%umm shift=%umm fwd=%umm side=%s unit=%s kodo=%u/1000 mm per rpm*s\r\n",
                  av_state_name(s_av.state), (unsigned)(s_av.disabled == 0U), (unsigned)s_av.app_mm,
                  (unsigned)s_av.shift_mm, (unsigned)s_av.fwd_mm, (s_av.side < 0) ? "right" : "left",
                  (s_av.unit_mm != 0U) ? "mm" : "cm", (unsigned)s_av.kodo_x1000);
        dbg_print("[AVOID] this leg dx=%ldum dy=%ldum. 'avoid run' starts on the next line-follow loop\r\n",
                  (long)s_av.leg_x_um, (long)s_av.leg_y_um);
        dbg_print("[AVOID] odo forward integral=%ld rpm*s/100. 'avoid odo' resets it; drive a measured distance; 'avoid calib <mm>'\r\n",
                  (long)s_av.cal_rpms_x100);
        return;
    }
    if (strcmp(sub, "on") == 0 || strcmp(sub, "off") == 0) {
        if (arg != NULL) goto usage;
        s_av.disabled = (sub[1] == 'f') ? 1U : 0U;
        chassis_av_cancel();
        dbg_print("[AVOID] %s\r\n", (s_av.disabled != 0U) ? "off" : "on");
        return;
    }
    if (strcmp(sub, "run") == 0) {
        if (arg != NULL) goto usage;
        s_av.force = 1U;
        dbg_print("[AVOID] starts on the next line-follow loop (no rangefinder needed)\r\n");
        return;
    }
    if (strcmp(sub, "app") == 0) {
        v = av_num(arg, 50, 2000, &ok);
        if (ok == 0U) goto usage;
        s_av.app_mm = (uint16_t)v;
        dbg_print("[AVOID] trigger distance=%umm\r\n", (unsigned)v);
        return;
    }
    if (strcmp(sub, "shift") == 0 || strcmp(sub, "fwd") == 0) {
        v = av_num(arg, 50, (sub[0] == 's') ? 800 : 1500, &ok);
        if (ok == 0U) goto usage;
        if (sub[0] == 's') { s_av.shift_mm = (uint16_t)v; arm_ctrl.cal.reserved[4] = (uint16_t)v; }
        else { s_av.fwd_mm = (uint16_t)v; arm_ctrl.cal.reserved[5] = (uint16_t)v; }
        dbg_print("[AVOID] %s=%umm ('pick save' to keep)\r\n", sub, (unsigned)v);
        return;
    }
    if (strcmp(sub, "side") == 0) {
        if (arg == NULL) goto usage;
        if (strcmp(arg, "left") == 0) s_av.side = 1;
        else if (strcmp(arg, "right") == 0) s_av.side = -1;
        else goto usage;
        arm_ctrl.cal.flags = (s_av.side < 0) ? (uint16_t)(arm_ctrl.cal.flags | ARM_CAL_FLAG_AVOID_RIGHT)
                                             : (uint16_t)(arm_ctrl.cal.flags & (uint16_t)~ARM_CAL_FLAG_AVOID_RIGHT);
        dbg_print("[AVOID] sidestep to the %s ('pick save' to keep)\r\n", (s_av.side < 0) ? "right" : "left");
        return;
    }
    if (strcmp(sub, "unit") == 0) {
        if (arg == NULL) goto usage;
        if (strcmp(arg, "cm") == 0) s_av.unit_mm = 0U;
        else if (strcmp(arg, "mm") == 0) s_av.unit_mm = 1U;
        else goto usage;
        dbg_print("[AVOID] rangefinder unit=%s\r\n", arg);
        return;
    }
    if (strcmp(sub, "wheel") == 0) {
        v = av_num(arg, 20, 300, &ok);
        if (ok == 0U) goto usage;
        /* 转子转速 / 36 减速比：轮子转一圈走 pi*D，所以 mm per (rpm*s) = pi*D/36/60 */
        av_kodo_set((uint16_t)((float)v * 3.14159265f * 1000.0f / 2160.0f + 0.5f));
        dbg_print("[AVOID] wheel %ldmm -> kodo=%u/1000 mm per rpm*s ('pick save' to keep)\r\n",
                  (long)v, (unsigned)s_av.kodo_x1000);
        return;
    }
    if (strcmp(sub, "odo") == 0) {
        if (arg != NULL) goto usage;
        dbg_print("[AVOID] odo reset (was %ld rpm*s/100). Drive straight a measured distance, then 'avoid calib <mm>'\r\n",
                  (long)s_av.cal_rpms_x100);
        s_av.cal_rpms_x100 = 0;
        return;
    }
    if (strcmp(sub, "calib") == 0) {
        v = av_num(arg, 50, 10000, &ok);
        if (ok == 0U) goto usage;
        if (s_av.cal_rpms_x100 < 1000L) {
            dbg_print("[AVOID] not enough distance driven since 'avoid odo' (%ld rpm*s/100). Drive at least a metre first\r\n",
                      (long)s_av.cal_rpms_x100);
            return;
        }
        av_kodo_set((uint16_t)(((int64_t)v * 100000L + s_av.cal_rpms_x100 / 2) / s_av.cal_rpms_x100));
        dbg_print("[AVOID] calib: %ldmm over %ld rpm*s -> kodo=%u/1000 mm per rpm*s ('pick save' to keep)\r\n",
                  (long)v, (long)(s_av.cal_rpms_x100 / 100), (unsigned)s_av.kodo_x1000);
        return;
    }
usage:
    dbg_print("[AVOID] usage: avoid status|on|off|run|app <mm>|shift <mm>|fwd <mm>|side left|right|unit cm|mm|wheel <mm>|odo|calib <mm>\r\n");
}
