///**
//  * @file bsp_judgesend.c
//  * @version 1.1
//  * @date  2022.5.22
//	*
//  * @brief UI的底层函数
//	*
//  *	@author  空中组
//  *
//  */

//#include "prot_judge_send.h"
//#include "usart_comm.h"
//#include "cmsis_os.h"
//#include "string.h"

//extern robot_status_t robot_status;
//extern game_robot_HP_t      RobotHP;

//self_inf_t self_inf;
//uint8_t ui_seq;

///**
//  * @brief  判断自身ID，选择客户端ID及阵营
//  * @param  void
//  * @attention
//  */
//void judge_get_basedata(void)
//{
//	self_inf.robot_ID = robot_status.robot_id;//读取当前机器人ID
//	if(self_inf.robot_ID > 100)
//	{
//		self_inf.color = BLUE;
//		self_inf.client_ID = 0x0164 + (self_inf.robot_ID-100);//计算客户端ID
//		self_inf.self_HP = RobotHP.ally_2_robot_HP;
//	}else
//	{
//		self_inf.color = RED;
//		self_inf.client_ID = 0x0100 + self_inf.robot_ID;
//		self_inf.self_HP = RobotHP.ally_2_robot_HP;
//	}
//}

///**
//  * @brief  客户端删除图形
//  * @param  类型选择：0: 空操作；
//	*										1: 删除图层；
//	*										2: 删除所有；
//  * @param  图层选择：0~9
//  * @attention
//  */
//	uint8_t CliendTxBuffer_delete[JUDGE_DELETE];

//void ui_delete(type_graphic_delete_e type, uint8_t layer)
//{
//	judgesend_frame_t delete_frame;
//	judgesend_delete_t delete_data;
//	
//	delete_frame.txFrameHeader.SOF = 0xA5;			//数据帧头
//	delete_frame.txFrameHeader.DataLength = sizeof(delete_frame.data_header) + sizeof(delete_data);	
//	delete_frame.txFrameHeader.Seq = ui_seq;
////	ui_seq++;

////	delete_frame.CmdID = ID_robot_interact;
//	delete_frame.data_header.data_cmd_id = CLIENT_DELETE_GRAPH_CMD_ID;
//	delete_frame.data_header.send_ID = self_inf.robot_ID;
//	delete_frame.data_header.receiver_ID = self_inf.client_ID;//客户端的ID，只能为发送者机器人对应的客户端
//	
//	delete_data.operate_tpye = type;
//	delete_data.layer = layer;

//	memcpy(CliendTxBuffer_delete, &delete_frame.txFrameHeader, sizeof(frame_header));//写入帧头数据
//	Append_CRC8_Check_Sum((uint8_t *)CliendTxBuffer_delete, sizeof(frame_header));//写入帧头CRC8校验码	
//	memcpy(CliendTxBuffer_delete + 5, (uint8_t*)&delete_frame.CmdID, 8);
//	memcpy(CliendTxBuffer_delete + 13, (uint8_t*)&delete_data, 2);
//	Append_CRC16_Check_Sum(CliendTxBuffer_delete, JUDGE_DELETE); //写入数据段CRC16校验码
//	HAL_UART_Transmit_DMA(&JUDGE_HUART, CliendTxBuffer_delete, JUDGE_DELETE);
//}

///**
//  * @brief  客户端绘制字符
//  * @param  图像名(不可重复)
//  * @param  图像操作
//  * @param  图层
//  * @param  字体颜色
//  * @param  字体大小
//  * @param  字符长度
//  * @param  线条宽度
//  * @param  起点x坐标
//  * @param  起点y坐标
//  * @param  字符
//  * @attention
//  */
//	uint8_t CliendTxBuffer_code[PRINTF_CODE];

//void ui_sendcode(
//			uint8_t code_name[3],
//			uint16_t operate_tpye,
//			uint16_t layer,
//			uint16_t color,
//			uint16_t code_size,
//			uint16_t code_width,
//			uint16_t code_startx,
//			uint16_t code_starty,
//			uint16_t code_length,
//			uint8_t* CODE)
//{
//	judgesend_code_t 	code_data;
//	judgesend_frame_t code_frame;

//	code_frame.txFrameHeader.SOF = 0xA5;			//数据帧头
//	code_frame.txFrameHeader.DataLength = sizeof(code_frame.data_header) + sizeof(code_data);	
//	code_frame.txFrameHeader.Seq = ui_seq;
////	ui_seq++;

////	code_frame.CmdID = ID_robot_interact;
//	code_frame.data_header.data_cmd_id = CLIENT_WRITE_STRINGS_CMD_ID;
//	code_frame.data_header.send_ID = self_inf.robot_ID;
//	code_frame.data_header.receiver_ID = self_inf.client_ID;//客户端的ID，只能为发送者机器人对应的客户端	
//	
//	memcpy(&code_data.grapic_data_struct.graphic_name, code_name,3);
//	code_data.grapic_data_struct.operate_tpye = operate_tpye;
//	code_data.grapic_data_struct.graphic_tpye = 7;
//	code_data.grapic_data_struct.layer = layer;
//	code_data.grapic_data_struct.color = color;
//	code_data.grapic_data_struct.start_angle = code_size;
//	code_data.grapic_data_struct.end_angle = code_length;
//	code_data.grapic_data_struct.width = code_width;
//	code_data.grapic_data_struct.start_x = code_startx;
//	code_data.grapic_data_struct.start_y = code_starty;	

//	//打包写入数据段
//	memcpy(CliendTxBuffer_code, &code_frame.txFrameHeader, sizeof(frame_header));//写入帧头数据	
//	Append_CRC8_Check_Sum((uint8_t *)CliendTxBuffer_code, sizeof(frame_header));//写入帧头CRC8校验码
//	memcpy(CliendTxBuffer_code + 5, (uint8_t*)&code_frame.CmdID, 8);
//	memcpy(CliendTxBuffer_code + 13, (uint8_t*)&code_data.grapic_data_struct, 15);
//	memcpy(CliendTxBuffer_code + 28, (uint8_t*)CODE, 30);
//	Append_CRC16_Check_Sum(CliendTxBuffer_code, PRINTF_CODE); //写入数据段CRC16校验码
//	HAL_UART_Transmit_DMA(&JUDGE_HUART, CliendTxBuffer_code, PRINTF_CODE);
//}

///**
//	* @brief  客户端绘图
//  * @param  Judgesend_length_e	 	data_length  数据总长
//	*		uint8_t* 			    data_locate  数据段指针
//	*		client_data_cmd_e 		date_type
//  * @retval void
//  * @attention  数据打包,打包完成后通过串口发送到裁判系统
//  */
//uint8_t CliendTxBuffer_graph[200];

//void ui_sendgraph(Judgesend_length_e data_length, uint8_t* data_locate, client_data_cmd_e date_type)
//{
//	judgesend_frame_t graph_frame;

//	graph_frame.txFrameHeader.SOF = 0xA5;			//数据帧头
//	graph_frame.txFrameHeader.DataLength = data_length - 9;
//	graph_frame.txFrameHeader.Seq = ui_seq;
////	ui_seq++;
//	
////	graph_frame.CmdID = ID_robot_interact;
//	graph_frame.data_header.data_cmd_id = date_type;         //子内容ID
//	graph_frame.data_header.send_ID = self_inf.robot_ID;     //发送者ID
//	graph_frame.data_header.receiver_ID = self_inf.client_ID;//客户端的ID，只能为发送者机器人对应的客户端	


//	//打包写入数据段
////	memcpy(CliendTxBuffer_graph, &graph_frame.txFrameHeader, sizeof(frame_header));//写入帧头数据
////	Append_CRC8_Check_Sum((uint8_t *)CliendTxBuffer_graph, sizeof(frame_header));//写入帧头CRC8校验码
////	memcpy(CliendTxBuffer_graph + 5, (uint8_t*)&graph_frame.CmdID, 8);
////	memcpy(CliendTxBuffer_graph + 13, (uint8_t*)data_locate, data_length - 15);
////	Append_CRC16_Check_Sum(CliendTxBuffer_graph, data_length); //写入数据段CRC16校验码
////	HAL_UART_Transmit_DMA(&JUDGE_HUART, CliendTxBuffer_graph, data_length);
//}

///**
//	* @brief  直线数据封装
//  * @attention  数据打包
//  */
//void ui_data_packed_line(
//			graphic_data_struct_t *line_data,
//			uint8_t line_name[3],
//			uint16_t operate_tpye,
//			uint16_t layer,
//			uint16_t color,
//			uint16_t line_width,
//			uint16_t line_startx,
//			uint16_t line_starty,
//			uint16_t line_endx,
//			uint16_t line_endy)
//{
//	memcpy(line_data->graphic_name,line_name,3);
//	line_data->operate_tpye = operate_tpye;
//	line_data->graphic_tpye = 0;
//	line_data->layer = layer;
//	line_data->color = color;
//	line_data->width = line_width;
//	line_data->start_x = line_startx;
//	line_data->start_y = line_starty;
//	line_data->end_x = line_endx;
//	line_data->end_y = line_endy;		
//}

///**
//	* @brief  矩形数据封装
//  * @attention  数据打包
//  */
//void ui_data_packed_rectangle(
//			graphic_data_struct_t *rect_data,
//			uint8_t rect_name[3],
//			uint16_t operate_tpye,
//			uint16_t layer,
//			uint16_t color,
//			uint16_t rect_width,
//			uint16_t rect_startx,
//			uint16_t rect_starty,
//			uint16_t rect_endx,
//			uint16_t rect_endy)
//{
//	memcpy(rect_data->graphic_name,rect_name,3);
//	rect_data->operate_tpye = operate_tpye;
//	rect_data->graphic_tpye = 1;
//	rect_data->layer = layer;
//	rect_data->color = color;
//	rect_data->width = rect_width;
//	rect_data->start_x = rect_startx;
//	rect_data->start_y = rect_starty;
//	rect_data->end_x = rect_endx;
//	rect_data->end_y = rect_endy;		
//}

///**
//	* @brief  圆形数据封装
//  * @attention  数据打包
//  */
//void ui_data_packed_round (
//			graphic_data_struct_t *round_data,
//			uint8_t round_name[3],
//			uint16_t operate_tpye,
//			uint16_t layer,
//			uint16_t color,
//			uint16_t round_width,
//			uint16_t round_centerx,
//			uint16_t round_centery,
//			uint16_t round_radius)
//{
//	memcpy(round_data->graphic_name,round_name,3);
//	round_data->operate_tpye = operate_tpye;
//	round_data->graphic_tpye = 2;
//	round_data->layer = layer;
//	round_data->color = color;
//	round_data->width = round_width;
//	round_data->start_x = round_centerx;
//	round_data->start_y = round_centery;
//	round_data->radius = round_radius;	
//}

///**
//	* @brief  椭圆数据封装
//  * @attention  数据打包
//  */
//void ui_data_packed_oval (
//			graphic_data_struct_t *oval_data,
//			uint8_t oval_name[3],
//			uint16_t operate_tpye,
//			uint16_t layer,
//			uint16_t color,
//			uint16_t oval_width,
//			uint16_t oval_centerx,
//			uint16_t oval_centery,
//			uint16_t oval_lengthx,
//			uint16_t oval_lengthy)
//{
//	memcpy(oval_data->graphic_name,oval_name,3);
//	oval_data->operate_tpye = operate_tpye;
//	oval_data->graphic_tpye = 3;
//	oval_data->layer = layer;
//	oval_data->color = color;
//	oval_data->width = oval_width;
//	oval_data->start_x = oval_centerx;
//	oval_data->start_y = oval_centery;
//	oval_data->end_x = oval_lengthx;
//	oval_data->end_y = oval_lengthy;		
//}

///**
//	* @brief  圆弧数据封装
//  * @attention  数据打包
//  */
//void ui_data_packed_arc (
//			graphic_data_struct_t *arc_data,
//			uint8_t arc_name[3],
//			uint16_t operate_tpye,
//			uint16_t layer,
//			uint16_t color,
//			uint16_t arc_width,
//			uint16_t arc_startangle,
//			uint16_t arc_endangle,
//			uint16_t arc_centerx,
//			uint16_t arc_centery,
//			uint16_t arc_lengthx,
//			uint16_t arc_lengthy)
//{
//	memcpy(arc_data->graphic_name,arc_name,3);
//	arc_data->operate_tpye = operate_tpye;
//	arc_data->graphic_tpye = 4;
//	arc_data->layer = layer;
//	arc_data->color = color;
//	arc_data->start_angle = arc_startangle;
//	arc_data->end_angle = arc_endangle;
//	arc_data->width = arc_width;
//	arc_data->start_x = arc_centerx;
//	arc_data->start_y = arc_centery;
//	arc_data->end_x = arc_lengthx;
//	arc_data->end_y = arc_lengthy;		
//}

///**
//	* @brief  浮点数数据封装
//  * @attention  数据打包
//  */
//void ui_data_packed_float (
//			float_data_struct_t *float_data,
//			uint8_t float_name[3],
//			uint16_t operate_tpye,
//			uint16_t layer,
//			uint16_t color,
//			uint16_t float_size,
//			uint16_t float_precision,
//			uint16_t float_width,
//			uint16_t float_startx,
//			uint16_t float_starty,
//			int32_t num)
//{
//	memcpy(float_data->graphic_name,float_name,3);
//	float_data->operate_tpye = operate_tpye;
//	float_data->graphic_tpye = 5;
//	float_data->layer = layer;
//	float_data->color = color;
//	float_data->start_angle = float_size;
//	float_data->end_angle = float_precision;
//	float_data->width = float_width;
//	float_data->start_x = float_startx;
//	float_data->start_y = float_starty;
//	float_data->num = num;
//}

///**
//	* @brief  整型数数据封装
//  * @attention  数据打包
//  */
//void ui_data_packed_int (
//			int_data_struct_t *int_data,
//			uint8_t int_name[3],
//			uint16_t operate_tpye,
//			uint16_t layer,
//			uint16_t color,
//			uint16_t int_size,
//			uint16_t int_width,
//			uint16_t int_startx,
//			uint16_t int_starty,
//			int32_t num)
//{
//	memcpy(int_data->graphic_name,int_name,3);
//	int_data->operate_tpye = operate_tpye;
//	int_data->graphic_tpye = 6;
//	int_data->layer = layer;
//	int_data->color = color;
//	int_data->start_angle = int_size;
//	int_data->width = int_width;
//	int_data->start_x = int_startx;
//	int_data->start_y = int_starty;
//	int_data->num = num;
//}

///**
//  * @brief  机器人交互数据
//  * @param  目标机器人id
//  * @param  交互数据id，可在0x0200~0x02ff选取，具体ID含义由参赛队自定义
//  * @param  交互数据?
//  * @attention
//  */
////	uint8_t CliendTxBuffer_message[ROBOT_COMM];

//void ui_sendmessage(uint16_t Robot_Target_ID, client_data_cmd_e CLIENT_INTERACTIVE_CMD_ID, uint8_t* comm_date)
//{
//	judgesend_frame_t message_frame;
//	
//	message_frame.txFrameHeader.SOF = 0xA5;			//数据帧头
////	message_frame.txFrameHeader.DataLength = sizeof(message_frame.data_header) + LEN_robot_interract;
//	message_frame.txFrameHeader.Seq = ui_seq;
////	ui_seq++;

////	message_frame.CmdID = ID_robot_interact;
//	message_frame.data_header.data_cmd_id = CLIENT_INTERACTIVE_CMD_ID_TEST;
//	message_frame.data_header.send_ID = self_inf.robot_ID;
//	message_frame.data_header.receiver_ID = Robot_Target_ID;//目标机器人ID
//	
//	//打包写入数据段
////	memcpy(CliendTxBuffer_message, &message_frame.txFrameHeader, sizeof(frame_header));//写入帧头数据
////	Append_CRC8_Check_Sum((uint8_t *)CliendTxBuffer_message, sizeof(frame_header));//写入帧头CRC8校验码
////	memcpy(CliendTxBuffer_message + 5, (uint8_t*)&message_frame.data_header, 8);
//////	memcpy(CliendTxBuffer_message + 13, (uint8_t*)comm_date, LEN_robot_interract);
////	Append_CRC16_Check_Sum(CliendTxBuffer_message, ROBOT_COMM); //写入数据段CRC16校验码
////	HAL_UART_Transmit_DMA(&JUDGE_HUART, CliendTxBuffer_message, ROBOT_COMM);
//}

///**
//  * @brief  小地图互动数据
//  * @param	机器人xyz坐标
//  * @param	云台手所按下按键
//  * @param  目标机器人id
//  * @attention
//  */
//	uint8_t CliendTxBuffer_map[29];

//void data_pack_map_position(float target_position_x, float target_position_y, float target_position_z, uint8_t commd_keyboard, uint16_t target_robot_ID)
//{
//	judgesend_frame_t map_frame;
//	judgesend_map_t map_data;

//	map_frame.txFrameHeader.SOF = 0xA5;			//数据帧头
//	map_frame.txFrameHeader.DataLength = sizeof(map_data);
//	map_frame.txFrameHeader.Seq = ui_seq;
////	ui_seq++;
//	
////	map_frame.CmdID = ID_client_map;
//	
//	map_data.target_position_x = target_position_x;
//	map_data.target_position_y = target_position_y;
//	map_data.target_position_z = target_position_z;
//	map_data.commd_keyboard = commd_keyboard;
//	map_data.target_robot_ID = target_robot_ID;

//	//打包写入数据段
//	memcpy(CliendTxBuffer_map, &map_frame.txFrameHeader, sizeof(frame_header));//写入帧头数据
//	Append_CRC8_Check_Sum((uint8_t *)CliendTxBuffer_map, sizeof(frame_header));//写入帧头CRC8校验码
//	memcpy(CliendTxBuffer_map + 5, (uint8_t*)&map_frame.CmdID, 2);
//	memcpy(CliendTxBuffer_map + 7, (uint8_t*)&map_data.target_position_x, 15);
////	Append_CRC16_Check_Sum(CliendTxBuffer_map, ROBOT_COMM); //写入数据段CRC16校验码
//	HAL_UART_Transmit_DMA(&JUDGE_HUART, CliendTxBuffer_map, 29);	
//}
