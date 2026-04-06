/**
  * @file bsp_judgesend.h
  * @version 1.1
  * @date  2022.3.14
	*
  * @brief 绘制函数的各种协议以及函数的声明，浮点数那个改了
	*
  *	@author  空中组抄强哥的
  *
  */
#ifndef PROT_JUDGE_SEND_H
#define PROT_JUDGE_SEND_H

#include "prot_judge_send.h"
#include "stdint.h"
#include "shoot_task.h"
#include "prot_rm_judge.h"
#include "mode_switch_task.h"
typedef enum 
{
	BLUE,
	RED
}color_e;

typedef __packed struct
{
	color_e	 color;				//阵营
	uint16_t robot_ID;		//自身id
	uint16_t client_ID;		//对应客户端id
	uint16_t self_HP;			//自身血量
}self_inf_t;

typedef enum 
{
	DRAW_IMAGINE1=30,
	DRAW_IMAGINE2=45,
	DRAW_IMAGINE5=90,
	DRAW_IMAGINE7=120,
	PRINTF_CODE	=60,
	JUDGE_DELETE=17,
//	ROBOT_COMM=(15+LEN_robot_interract),
}Judgesend_length_e;//自定义数据发送总长度


/* 裁判系统数据帧帧头+交互数据接收信息帧头 */
typedef __packed struct
{
	frame_header   									txFrameHeader;//帧头5
	cmd_ID		 											CmdID;//命令码2
}SendClient_header_t;

typedef enum
{
	CLIENT_INTERACTIVE_CMD_ID_TEST    = 0x0200 , //交互数据，可在0x0200~0x02ff选取，具体ID含义由参赛队自定义
	CLIENT_DELETE_GRAPH_CMD_ID     		= 0x0100 , //客户端删除图形
	CLIENT_DRAW_1_GRAPHS_CMD_ID     	= 0x0101 , //客户端绘制1个图形
	CLIENT_DRAW_2_GRAPHS_CMD_ID    		= 0x0102 , //客户端绘制2个图形
	CLIENT_DRAW_5_GRAPHS_CMD_ID    		= 0x0103 , //客户端绘制5个图形
	CLIENT_DRAW_7_GRAPHS_CMD_ID    		= 0x0104 , //客户端绘制7个图形
	CLIENT_WRITE_STRINGS_CMD_ID    		= 0x0110 , //客户端绘制字符串图形
}client_data_cmd_e;//数据段的内容 ID

/* 交互数据接收信息帧头 */
typedef __packed struct 
{ 
	client_data_cmd_e data_cmd_id;    
	uint16_t 					send_ID;    
	uint16_t 					receiver_ID; 
} judgesend_custom_header_t;

typedef __packed struct
{
	frame_header   							txFrameHeader;//帧头5
	cmd_ID		 									CmdID;//命令码2;
	judgesend_custom_header_t		data_header;//数据段帧头 6
	//此处为各种数据段
	uint16_t		 								FrameTail;//帧尾 2
}judgesend_frame_t;


/* 图形删除 */
typedef enum
{
	type_delete_nop   = 0,
	type_delete_layer = 1,
	type_delete_all   = 2,
}type_graphic_delete_e;//图形删除类型

/*客户端删除图形*/
typedef __packed struct
{ 
	type_graphic_delete_e operate_tpye;
	uint8_t layer;
} judgesend_delete_t;

/* 图形数据*/
typedef __packed struct 
{ 
	uint8_t graphic_name[3]; 
	uint16_t operate_tpye:3; 
	uint16_t graphic_tpye:3;
	uint16_t layer:4;
	uint16_t color:4; 
	uint16_t start_angle:9;
	uint16_t end_angle:9;
	uint16_t width:10;
	uint16_t start_x:11;
	uint16_t start_y:11;
	uint16_t radius:10; 
	uint16_t end_x:11;
	uint16_t end_y:11;
} graphic_data_struct_t;

/* 浮点数据*/
typedef __packed struct 
{ 
	uint8_t graphic_name[3]; 
	uint16_t operate_tpye:3; 
	uint16_t graphic_tpye:3;
	uint16_t layer:4;
	uint16_t color:4; 
	uint16_t start_angle:9;
	uint16_t end_angle:9;
	uint16_t width:10;
	uint16_t start_x:11;
	uint16_t start_y:11;
	int32_t num;
} float_data_struct_t;

/* 整型数据*/
typedef __packed struct 
{ 
	uint8_t graphic_name[3]; 
	uint16_t operate_tpye:3; 
	uint16_t graphic_tpye:3;
	uint16_t layer:4;
	uint16_t color:4; 
	uint16_t start_angle:9;
	uint16_t end_angle:9;
	uint16_t width:10;
	uint16_t start_x:11;
	uint16_t start_y:11;
	int32_t num;
} int_data_struct_t;

/* 绘制字符 */
typedef __packed struct 
{
	graphic_data_struct_t grapic_data_struct; 
	uint8_t data[30];
} judgesend_code_t;

/*小地图交互信息*/
typedef __packed struct
{
	float target_position_x;		//目标x位置坐标，单位m		当发送目标机器人ID时，该项为0
	float target_position_y;		//目标y位置坐标，单位m 		当发送目标机器人ID时，该项为0
	float target_position_z;		//目标z位置坐标，单位m 		当发送目标机器人ID时，该项为0
	uint8_t commd_keyboard;			//发送指令时，云台手按下的键盘信息 		无按键按下则为0
	uint16_t target_robot_ID;		//要作用的目标机器人ID 		当发送位置信息时，该项为0
}judgesend_map_t;

void judge_get_basedata(void);

void ui_data_packed_line(
			graphic_data_struct_t *line_data,
			uint8_t line_name[3],
			uint16_t operate_tpye,
			uint16_t layer,
			uint16_t color,
			uint16_t line_width,
			uint16_t line_startx,
			uint16_t line_starty,
			uint16_t line_endx,
			uint16_t line_endy);
void ui_data_packed_rectangle(
			graphic_data_struct_t *rect_data,
			uint8_t rect_name[3],
			uint16_t operate_tpye,
			uint16_t layer,
			uint16_t color,
			uint16_t rect_width,
			uint16_t rect_startx,
			uint16_t rect_starty,
			uint16_t rect_endx,
			uint16_t rect_endy);
void ui_data_packed_round (
			graphic_data_struct_t *round_data,
			uint8_t round_name[3],
			uint16_t operate_tpye,
			uint16_t layer,
			uint16_t color,
			uint16_t round_width,
			uint16_t round_centerx,
			uint16_t round_centery,
			uint16_t round_radius);
void ui_data_packed_oval (
			graphic_data_struct_t *oval_data,
			uint8_t oval_name[3],
			uint16_t operate_tpye,
			uint16_t layer,
			uint16_t color,
			uint16_t oval_width,
			uint16_t oval_centerx,
			uint16_t oval_centery,
			uint16_t oval_lengthx,
			uint16_t oval_lengthy);
void ui_data_packed_arc (
			graphic_data_struct_t *arc_data,
			uint8_t arc_name[3],
			uint16_t operate_tpye,
			uint16_t layer,
			uint16_t color,
			uint16_t arc_width,
			uint16_t arc_startangle,
			uint16_t arc_endangle,
			uint16_t arc_centerx,
			uint16_t arc_centery,
			uint16_t arc_lengthx,
			uint16_t arc_lengthy);
void ui_data_packed_float (
			float_data_struct_t *float_data,
			uint8_t float_name[3],
			uint16_t operate_tpye,
			uint16_t layer,
			uint16_t color,
			uint16_t float_size,
			uint16_t float_precision,
			uint16_t float_width,
			uint16_t float_startx,
			uint16_t float_starty,
			int32_t num);
void ui_data_packed_int (
			int_data_struct_t *int_data,
			uint8_t int_name[3],
			uint16_t operate_tpye,
			uint16_t layer,
			uint16_t color,
			uint16_t int_size,
			uint16_t int_width,
			uint16_t int_startx,
			uint16_t int_starty,
			int32_t num);


void ui_delete(type_graphic_delete_e type, uint8_t layer);
void ui_sendcode(
			uint8_t code_name[3],
			uint16_t operate_tpye,
			uint16_t layer,
			uint16_t color,
			uint16_t code_size,
			uint16_t code_width,
			uint16_t code_startx,
			uint16_t code_starty,
			uint16_t code_length,
			uint8_t* CODE);
void ui_sendgraph(Judgesend_length_e data_length, uint8_t* data_locate, client_data_cmd_e date_type);
void ui_sendmessage(uint16_t Robot_Target_ID, client_data_cmd_e CLIENT_INTERACTIVE_CMD_ID, uint8_t* comm_date);
void data_pack_map_position(float target_position_x, float target_position_y, float target_position_z, uint8_t commd_keyboard, uint16_t target_robot_ID);

void judge_send_task(void const *argu);
void ui_init(void);

void ui_fric_mode_display(shoot_firc_mode_e function);
void ui_stir_mode_display(shoot_stir_mode_e function);
void ui_ctrl_mode_display(ctrl_mode_e function);
void ui_gimbal_mode_display(_gimbal_ctrl_ gimbal_ctrl_mode);
void gimbal_control_sentry(void);
void ui_robot_HP_init(void);
void ui_robot_HP_display(void);
void warning_len_display(uint16_t len_x,uint16_t len_start_y,uint16_t len_end_y,uint16_t color,uint16_t operate);
void import_point_display(void);
void infantry_type_init(void);
void infantry_type_dispaly(void);
void imu_display(void);
void attacked_display( void );

#endif

