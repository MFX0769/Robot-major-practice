#include "stdint.h"

#include "prot_vision.h"
#include "prot_can_send_interface.h"


#ifndef __BSP_TRIIGERMOTOR_H
#define __BSP_TRIIGERMOTOR_H

#define TRIGGER_MOTOR_ECD   36859.5f  //拨盘一颗子弹转过的编码值 8191 * 36 / 8 = 36859.5f

#define MIN_HEAT		    40

typedef enum
{
	T_ENABLE = 1,
	T_DISABLE = 0
}trigger_enable_e;

//2025年4月21日 先把enable加进来，只是用在单发复位功能上，后续再慢慢统一
typedef struct
{
//	trigger_pid_t 		trigger_pid;
//	trigger_init_e		init_flag;
//	shoot_stir_mode_e 	stir_mode;
//	int16_t				trigger_current;
	uint8_t 			trigger_enable;
//	float				trigger_period;
} trigger_t;

extern trigger_t trigger;


void TriggerMotor_init(void);
void TriggerMotor_control(void);

uint8_t single_shoot_reset(void);

uint8_t single_shoot_enable(void);

uint8_t series_shoot_enable(void);
void single_ammo_stuck_judge(void);
extern 	uint8_t 	SHOOT_PERIOD_CHANGE_FLAG;
extern 	uint8_t 	SHOOT_PERIOD_CHANGE_CNT;

extern uint16_t test_Trigger;
extern uint8_t  STIR_SERIES_SHOT_FLAG;
#endif

