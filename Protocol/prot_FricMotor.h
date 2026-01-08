#ifndef __DRIVER_FRICMOTOR
#define __DRIVER_FRICMOTOR

#include "stm32h7xx.h"
// #include "shoot_task.h"
#include "prot_can_send_interface.h"

typedef enum
{
	FRIC_LEFT 	= 0,
	FRIC_RIGHT 	= 1
}fric_e;

void FricMotor_init(void);
void FricMotor_Control(void);
#endif
