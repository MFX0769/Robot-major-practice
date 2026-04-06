/**
  * @file bsp_judge.h
  * @version 1.0
  * @date  2025.12.5
	*
  * @brief 裁判系统串口协议
	*
  *	@author  空中组
  *
  */
#ifndef PROT_RM_JUDGE_H
#define PROT_RM_JUDGE_H
#include "stdint.h"
#include "crc.h"
#include "mode_switch_task.h"


#define 	JUDGESEND_PERIOD 	10  //发送间隔，理论上越短越快
#define		JUDGESEND_TIME		10

#define   LEN_HEADER    5       //帧头长
#define   LEN_CMDID     2       //命令码长度
#define   LEN_TAIL      2	      //帧尾CRC16

//起始字节,协议固定为0xA5
#define    JUDGE_FRAME_HEADER         (0xA5)



typedef enum 
{
	FRAME_HEADER         = 0,
	CMD_ID               = 5,
	DATA                 = 7,
}JudgeFrameOffset;

//5字节帧头,偏移位置
typedef enum
{
	SOF          = 0,//起始位
	DATA_LENGTH  = 1,//帧内数据长度,根据这个来获取数据长度
	SEQ          = 3,//包序号
	CRC8         = 4 //CRC8
}FrameHeaderOffset;

typedef enum
{
	ID_game_state		  =		0x0001,		//比赛状态				1Hz
	ID_game_result		=		0x0002,		//比赛结果				结束时发送
	ID_robot_HP				=		0x0003,		//机器人血量			1Hz
//	ID_darts_status		=		0x0004,		//飞镖发射状态		飞镖发射后发送
	
	ID_event_data			=		0x0101,		//场地事件数据		事件改变后发送
	ID_supply_action	=		0x0102,		//补给站动作标识	动作发生后发送
	ID_judge_warning	=		0x0104,		//裁判警告数据		警告发生后发送
	ID_dart_remaining_time =0x0105,	//飞镖发射倒计时	1Hz
	
	ID_robot_status		=		0x0201,		//机器人状态数据	10Hz
	ID_robot_power		=		0x0202,		//实时功率热量		50Hz
	ID_robot_position =		0x0203,		//机器人位置数据	10Hz
	ID_robot_buff			=		0x0204,		//机器人增益数据	增益状态改变后发送
//	ID_AerialRobotEnergy =	0x0205, //无人机能量			10Hz,只发送空中
	ID_robot_hurt			=		0x0206,		//伤害状态数据		伤害发生后发送
	ID_shoot_data			=		0x0207,		//实时射击数据		子弹发射后发送
	ID_bullet_remaining = 0x0208,   //弹丸剩余发射数	1Hz，空中/哨兵
	ID_RFID_status		=		0x0209,		//机器人RFID状态	1Hz
	ID_dart_client    =   0x020A,		//飞镖机器人客户端指令数据 10Hz
	
	ID_robot_interact=		0x0301,		//机器人间交互数据	发送方触发发送,上限10Hz
	ID_custom_robot_data            = 0x0302, //自定义控制器与机器人交互数据，发送方触发发送，频率上限为30Hz 图传链路
	ID_map_command			= 	0x0303,	//客户端小地图交互数据	触发发送
	ID_map_robot_data               = 0x0305, //选手端小地图接收雷达数据，频率上限为5Hz
    ID_custom_client_data           = 0x0306, //自定义控制器与选手端交互数据，发送方触发发送，频率上限为30Hz
    ID_map_data                     = 0x0307, //选手端小地图接收路径数据，频率上限为1Hz
    ID_custom_info                  = 0x0308, //选手端小地图接收机器人数据，频率上限为3Hz
	ID_robot_custom_data			= 0x0309, //自定义控制器接收机器人数据，频率上限为10Hz 图传链路
}cmd_ID;

//数据段长度
typedef enum
{
	LEN_game_state           = 11,   // 无变化
    LEN_game_result          = 1,    // 无变化
    LEN_robot_HP             = 16,   // ★旧 32→新 16（只拷己方 8 机器人）
//    LEN_darts_status         = 3,    // 无变化
    LEN_event_data           = 4,    // 无变化
    LEN_supply_action        = 4,    // 无变化
    LEN_judge_warning        = 3,    // ★旧 2→新 3（新增 count 字节）
    LEN_darts_remaining_tim  = 3,    // ★旧 1→新 3（新增 dart_info 字端）
    LEN_robot_status         = 13,   // ★旧 27→新 13（精简结构）
    LEN_robot_power          = 14,   // ★旧 16→新 14（去掉 2B 保留）
    LEN_robot_position       = 16,   
    LEN_robot_buff           = 8,    // ★旧 1→新 8（扩展增益字段）
//    LEN_AerialRobotEnergy    = 2,    // 无变化
    LEN_robot_hurt           = 1,    // 无变化
    LEN_shoot_data           = 7,    // 无变化
    LEN_bullet_remaining     = 6,    
    LEN_RFID_status          = 5,    // ★旧 4→新 5（新增 rfid_status_2）
    LEN_dart_client          = 6,    // ★旧 12→新 6（精简指令结构）
    LEN_robot_interract      = 118,  // ★旧 20（用户数据）
	LEN_map_command                 = 12,//手册有矛盾 有15和12
    LEN_map_robot_data              = 24,
}cmd_LEN;


//以下为具体数据结构体

/* 帧头 */
typedef struct
{
	uint8_t  SOF;					//起始字节
	uint16_t DataLength;	//数据长度
	uint8_t  Seq;					//包序号
	uint8_t  CRC8;				//crc8校验
}__attribute__((packed)) frame_header;

/* 比赛状态数据：0x0001。发送频率：1Hz，发送范围：所有机器人。*/
typedef struct 
{ 
	uint8_t game_type : 4;				//1机甲大师赛，2机甲大师单项赛，4RoboMaster 联盟赛3V3，5RoboMaster 联盟赛1V1
	uint8_t game_progress : 4;		//0未开始比赛，1准备阶段，2自检阶段，3五秒倒计时，4对战中，5比赛解算中
	uint16_t stage_remain_time;		//当前阶段剩余时间，单位秒
	uint64_t SyncTimeStamp;				//机器人接收到该指令的精确时间，当机载端收到有效的服务器授时后生效
}__attribute__((packed)) game_status_t; 

/* 比赛结果数据：0x0002。发送频率：比赛结束后发送，发送范围：所有机器人。*/
typedef struct 
{ 
	uint8_t winner;								//0 平局 1 红方胜利 2 蓝方胜利
}__attribute__((packed)) game_result_t;; 

/* 机器人血量数据：0x0003。发送频率：1Hz，发送范围：所有机器人。*/
typedef struct 
{ 
uint16_t ally_1_robot_HP; 
uint16_t ally_2_robot_HP; 
uint16_t ally_3_robot_HP; 
uint16_t ally_4_robot_HP; 
uint16_t reserved; 
uint16_t ally_7_robot_HP; 
uint16_t ally_outpost_HP; 
uint16_t ally_base_HP; 
}__attribute__((packed)) game_robot_HP_t;

/* 飞镖发射状态：0x0004。发送频率：飞镖发射后发送，发送范围：所有机器人。*/
typedef struct
{
	uint8_t dart_belong;							//1红方飞镖，2蓝方飞镖 
	uint16_t stage_remaining_time; 		//发射时的剩余比赛时间，单位秒
}__attribute__((packed)) dart_status_t;

/* 场地事件数据：0x0101。发送频率：事件改变后发送，发送范围：己方机器人。*/
typedef struct
{
	uint32_t event_data;
	/*
	bit 0-2：
  bit0 己方补给站 1号补血点占领状态 1为已占领；
	bit1 己方补给站 2号补血点占领状态 1为已占领
	bit2 己方补给站 3号补血点占领状态 1为已占领；	
	bit 3-5：己方能量机关状态：
	bit 3 为打击点占领状态，1为占领；
	bit 4 为小能量机关激活状态，1为已激活；
	bit 5 为大能量机关激活状态，1为已激活；
	
	bit 6：己方R2环形高地占领状态1为已占领；
	bit 7：己方R3梯形高地占领状态 1为已占领；
	bit 8：己方R4梯形高地占领状态 1为已占领；
	bit 9：己方基地护盾状态：
		1 为基地有虚拟护盾血量；
		0为基地无虚拟护盾血量；
	bit 10：己方前哨战状态：
		1为前哨战存活；
		0为前哨战被击毁；
	bit 10 -31: 保留	
	*/
}__attribute__((packed)) event_data_t;

/* 补给站动作标识：0x0102。发送频率：动作触发后发送，发送范围：己方机器人。*/
typedef struct
{
	uint8_t supply_projectile_id; 		//补给站口 ID：1 ，2
	uint8_t supply_robot_id; 					//补弹机器人 ID：0 为当前无机器人补弹，1 为红方英雄机器人补弹，2 为红方工程机器人补弹，3/4/5 为红方步兵机器人补弹，蓝方为101/102/103/104/105
	uint8_t supply_projectile_step; 	//出弹口开闭状态：0 为关闭，1 为子弹准备中，2 为子弹下落
	uint8_t supply_projectile_num;		//补弹数量：50 ，100 ，150 ，200
}__attribute__((packed)) supply_projectile_action_t;

/* 裁判警告信息：cmd_id (0x0104)。发送频率：警告发生后发送，发送范围：己方机器人。*/
typedef struct
{
	uint8_t level; 						//警告等级
	uint8_t offending_robot_id;		//犯规机器人ID：判负时，机器人ID为0,黄牌、红牌时，机器人ID为犯规机器人ID
	uint8_t count;                 	//己方最后一次受到判罚的违规机器人对应判罚等级的违规次数。
}__attribute__((packed)) referee_warning_t;

/* 飞镖发射口倒计时：cmd_id (0x0105)。发送频率：1Hz 周期发送，发送范围：己方机器人*/
typedef struct
{ 
	uint8_t dart_remaining_time;
	uint16_t dart_info;
}__attribute__((packed)) dart_info_t;

/* 比赛机器人状态：0x0201。发送频率：10Hz，发送范围：单一机器人。*/
typedef struct
{ 
	/*本机器人ID*/
    uint8_t robot_id;
	/*机器人等级*/
    uint8_t robot_level;
	/*机器人当前血量*/
    uint16_t current_HP;
	/*机器人血量上限*/
    uint16_t maximum_HP;
	/*机器人射击热量每秒冷却值*/
    uint16_t shooter_barrel_cooling_value;
	/*机器人射击热量上限*/
    uint16_t shooter_barrel_heat_limit;
	/*机器人底盘功率上限*/
    uint16_t chassis_power_limit;
	/* 电源管理模块的输出情况： 
	? bit 0：gimbal口输出，0为无输出，1为 24V输出 
	? bit 1：chassis口输出，0为无输出，1为24V输出 
	? bit 2：shooter口输出，0为无输出，1为24V输出*/
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
}__attribute__((packed)) robot_status_t;

/* 实时功率热量数据：0x0202。发送频率：50Hz，发送范围：单一机器人。*/
typedef struct
{
	uint16_t reserved_1; 
	uint16_t reserved_2; 
    float reserved_3;
	/*缓冲能量（单位：J）*/
    uint16_t buffer_energy;
	/*17mm发射机构的射击热量 */
    uint16_t shooter_17mm_barrel_heat;
	/*42mm发射机构的射击热量 */
    uint16_t shooter_42mm_barrel_heat;
}__attribute__((packed)) power_heat_data_t;

/* 机器人位置：0x0203。发送频率：10Hz，发送范围：单一机器人。*/
typedef struct
{
	float x;		//单位米
	float y;
	float angle;	//枪口位置，单位度
}__attribute__((packed)) robot_pos_t;

/* 机器人增益：0x0204。发送频率：1Hz 周期发送，发送范围：单一机器人。*/
typedef struct
{
 uint8_t recovery_buff; 
 uint16_t cooling_buff; 
 uint8_t defence_buff; 
 uint8_t vulnerability_buff; 
 uint16_t attack_buff; 
 uint8_t remaining_energy;
	/* 
	bit 0：机器人血量补血状态
	bit 1：枪口热量冷却加成
	bit 2：机器人防御加成
	bit 3：机器人攻击加成
	*/
}__attribute__((packed)) buff_t;

///* 空中机器人能量状态：0x0205。发送频率：10Hz，发送范围：单一机器人。*/
//typedef __packed struct
//{ 
//	uint8_t attack_time;		//可攻击时间，单位 s，30s 递减至 0	
//}ext_aerial_robot_energy_t;

/* 伤害状态：0x0206。发送频率：伤害发生后发送，发送范围：单一机器人。*/
typedef struct
{
	uint8_t armor_id : 4;
	uint8_t hurt_type : 4;
	/*
	bit 0-3：当血量变化类型为装甲伤害，代表装甲 ID，其中数值为 0-4 号代表机器人的五个装甲片，其他血量变化类型，该变量数值为 0。
	bit 4-7：血量变化类型:
	         0x0 装甲伤害扣血；
	         0x1 模块掉线扣血；
	         0x2 超射速扣血；
	         0x3 超枪口热量扣血；
	         0x4 超底盘功率扣血；
	         0x5 装甲撞击扣血
	*/
}__attribute__((packed)) hurt_data_t;

/* 实时射击信息：0x0207。发送频率：射击后发送，发送范围：单一机器人。*/
typedef struct
{
  uint8_t bullet_type;		//子弹类型: 1：17mm 弹丸 2：42mm 弹丸
	uint8_t shooter_id;			//发射机构ID： 1:1号17mm发射机构	2:2号17mm发射机构		3：42mm 发射机构
  uint8_t bullet_freq;		//子弹射频 单位 Hz
  float bullet_speed;			//子弹射速 单位 m/s
}__attribute__((packed)) shoot_data_t;

/* 子弹剩余发射数：0x0208。发送频率：10Hz 周期发送，所有机器人，*/

typedef struct 
{ 
	uint16_t bullet_remaining_num_17mm;
	/*
	17mm子弹剩余发射数目 
	步兵机器人/英雄机器人:全队步兵与英雄剩余可发射17mm弹丸总量(联盟赛)		
	空中机器人/哨兵机器人:该机器人剩余可发射17mm弹丸总量(联盟赛/对抗赛)
	*/
	uint16_t bullet_remaining_num_42mm;	//42mm子弹剩余发射数目
	uint16_t coin_remaining_num; 				//剩余金币数量
	uint16_t projectile_allowance_fortress; //堡垒增益点提供的发弹量
}__attribute__((packed)) bullet_remaining_t;

/* 机器人 RFID 状态：0x0209。发送频率：1Hz，发送范围：单一机器人。*/
typedef struct
{
  uint32_t rfid_status;
	/*
	bit 0：基地增益点 RFID 状态；
  bit 1：高地增益点 RFID 状态；
  bit 2：能量机关激活点 RFID 状态；
  bit 3：飞坡增益点 RFID 状态；
  bit 4：前哨岗增益点 RFID 状态；
  bit 5：资源岛增益点 RFID 状态；
  bit 6：补血点增益点 RFID 状态；
  bit 7：工程机器人补血卡 RFID 状态；
  bit 8-31：保留
  RFID 状态不完全代表对应的增益或处罚状态，例如敌方已占领的高地增益点，不能获取对应的增益效果.
	
	*/
	uint8_t rfid_status_2;/*bit 0：对方地形跨越增益点（隧道）（靠近对方梯形高地较低处）
? bit 1：对方地形跨越增益点（隧道）（靠近对方梯形高地较高处）*/
}__attribute__((packed)) rfid_status_t;

//typedef struct
//{
//	float target_position_x; 
//	float target_position_y; 
//	uint8_t cmd_keyboard; 
//	uint8_t target_robot_id; 
//	uint16_t cmd_source;
//}__attribute__((packed)) map_command_t;

//typedef struct
//{
//    uint16_t hero_position_x; 
//	uint16_t hero_position_y; 
//	uint16_t engineer_position_x; 
//	uint16_t engineer_position_y; 
//	uint16_t infantry_3_position_x; 
//	uint16_t infantry_3_position_y; 
//	uint16_t infantry_4_position_x; 
//	uint16_t infantry_4_position_y; 
//	uint16_t reserved; 
//	uint16_t reserved_1; 
//	uint16_t sentry_position_x; 
//	uint16_t sentry_position_y;
//}__attribute__((packed)) map_robot_data_t;

/* 飞镖机器人客户端指令数据：0x020A。发送频率：10Hz，发送范围：单一机器人。*/
typedef struct
{
	uint8_t dart_launch_opening_status;
	uint8_t dart_attack_target;
	uint16_t target_change_time;
  uint16_t operate_launch_cmd_time;
}__attribute__((packed)) dart_client_cmd_t;

/* 机器人间交互数据：0x0200~0x02ff。发送范围：单一机器人。*/
typedef struct 
{ 
uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[112];
}__attribute__((packed)) robot_interactive_receivedata_t;//机器人间交互数据
typedef struct
{
	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
	int8_t left_button_down;
	int8_t right_button_down;
	uint16_t keyboard_value;
	uint16_t reserved;
}__attribute__((packed)) remote_control_t;

int Judge_Read_Data(uint8_t *ReadFromUsart);

extern robot_status_t robot_status;
extern shoot_data_t Shoot_Data;
extern game_status_t Game_status;
extern remote_control_t Remote_Control;
extern power_heat_data_t Power_Heat_Data;//(2025年2月27日 仔细测试各个电机的功率，给硬件组的大佬做降压！
#endif //版本定义
