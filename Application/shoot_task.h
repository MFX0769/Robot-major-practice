#ifndef __SHOOT_TASK_H__
#define __SHOOT_TASK_H__

#ifdef  __SHOOT_TASK_GLOBALS
#define __SHOOT_TASK_EXT
#else
#define __SHOOT_TASK_EXT extern
#endif

#include "stm32h7xx.h"
#include "prot_FricMotor.h"
#include "prot_TriggerMotor.h"
/* 拨盘闭环控制变量结构 */
typedef struct
{
    /* position ecd loop */
    float trigger_ecd_ref;
    float trigger_ecd_fdb;
    float trigger_ecd_error;
    /* speed loop */
    float trigger_spd_ref;
    float trigger_spd_fdb;
} trigger_pid_t;

/* 摩擦轮模式 */
typedef enum
{
    FIRC_MODE_STOP = 0,
    FIRC_MODE_RUN = 1
} shoot_firc_mode_e;

/*摩擦轮pid*/
typedef struct
{
    int16_t   spd_ref;
    int16_t   spd_fdb;
		int16_t   spd_error;
		int16_t   current; 
	  int16_t   filter_fdb;
} fric_pid_t;

/* 拨盘模式 */
typedef enum
{
//    STIR_MODE_STOP,
    STIR_MODE_PROTECT=0,
    STIR_MODE_SINGLE=1,
    STIR_MODE_SERIES=2  //连发
} shoot_stir_mode_e;

/* 弹舱模式 */
//////typedef enum
//////{
//////    HOUSE_MODE_OPEN = 0,
//////    HOUSE_MODE_CLOSE = 1,
//////    HOUSE_MODE_PROTECT = 2
//////} shoot_house_mode_e;

/* 枪管数据结构  关我飞机什么事*/
typedef struct
{
    trigger_pid_t pid;
    int16_t     current;            //发射器电流值
    float       heat;               //当前热量
    int32_t     heat_remain;        //剩余热量
    uint16_t    heat_max;			//最大热量	裁判系统读出
    uint16_t    cooling_rate;	    //冷却速率  裁判系统读出
    uint32_t    shoot_period;       //射击周期
} barrel_param_t;

/* 发射器控制中心结构 */
typedef struct
{
    shoot_firc_mode_e   firc_mode;
    shoot_stir_mode_e   stir_mode;
	  uint8_t             SHOT_FLAG;//2025年4月3日 用于观测当前是否控制发射  0否 1射
//    shoot_house_mode_e  house_mode;
    barrel_param_t      barrel;
		trigger_pid_t       pid;
		int16_t             current;
    uint16_t            shoot_speed;		//射速
    float               trigger_period;	//拨盘拨出一颗子弹的周期，体现射频
	   uint16_t frequency_cnt;	//射击周期计算
    uint8_t             shoot_speed_vision;//发给视觉的射速档位
	
	uint8_t series_shoot_enable_FLAG;
	uint8_t single_shoot_enable_FLAG;
	
	/*新遥控相关开始*/
	uint8_t FN2;
	uint8_t last_FN2;
	uint8_t shoot_mode_CNT;//右上角相机反转键（FN2）连续按5次，就进入连发模式，再按一次退出。用一个定时器，当模式是“shoot_protect"（即等待射击解锁指令）的时候，每隔3秒清零一次。若已经解锁了射击，则不清零，此时一旦相机反转键（FN2）被按下，马上进入shoot_protect
	/*新遥控相关结束*/
} shoot_t;

void shoot_task(void const *argu);
void shoot_init(void);

__SHOOT_TASK_EXT shoot_t shoot;    //发射器控制中心结构

extern fric_pid_t fric[2];
#endif
