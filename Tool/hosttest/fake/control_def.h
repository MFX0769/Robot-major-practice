#ifndef FAKE_CONTROL_DEF_H
#define FAKE_CONTROL_DEF_H
/* 只保留底盘任务用到的几个数值，和真文件里的当前值一致（真文件编码损坏，主机编译器读不好） */
#define CHASSIS_PERIOD       2
#define CHASSIS_SCALE_VX     3.0f
#define CHASSIS_SCALE_VY     3.0f
#define CHASSIS_SCALE_WZ    15.0f
#define MECANUM_L            1.0f
#endif
