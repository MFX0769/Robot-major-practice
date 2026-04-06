/**
  * @file    prot_rm_judge.c
  * @version 2.0
  * @date    2025-12-01
  * @brief   RoboMaster 2026 协议 V1.0.0 解析（带中文注释）
  */

#include "prot_rm_judge.h"
#include "string.h"

/*****************数据结构体定义**********************/
frame_header											FrameHeader;		//帧头

game_status_t									Game_status;
game_result_t									Game_Result;
game_robot_HP_t               RobotHP;
dart_status_t									Dart_Status;
event_data_t									Event_Data;
supply_projectile_action_t		Supply_Projectile_Action;
referee_warning_t							Referee_Warning;
dart_info_t					dart_info;
robot_status_t						robot_status;
power_heat_data_t							Power_Heat_Data;
robot_pos_t							robot_pos;
buff_t												Buff;
//aerial_robot_energy_t					Aerial_Robot_Energy;
hurt_data_t									hurt_data;
shoot_data_t									Shoot_Data;
bullet_remaining_t						Bullet_Remaining;
rfid_status_t									RFID_Status;
dart_client_cmd_t							Dart_Client;
robot_interactive_receivedata_t		Comm_getdata;
//小地图交互接收
//map_command_t                       map_command;                    //选手端小地图交互数据，选手端触发发送
//map_robot_data_t                    map_robot_data;
/******************************************************/



uint16_t Robot_Self_HP;//自身血量

/**
  * @brief  读取裁判数据函数，串口中断函数中直接调用进行读取
  * @param  对应串口的缓存区数据
  * @retval 是否对正误判断做处理
  * @attention  在此判断帧头和CRC校验,无误再写入数据，不重复判断帧头
  */
int Judge_Read_Data(uint8_t *ReadFromUsart)
{
    uint8_t  retval_tf = 0;
    uint16_t judge_length;
    uint16_t CmdID;

    if (ReadFromUsart == NULL) return -1;

    /* 1. 取帧头 */
    memcpy(&FrameHeader, ReadFromUsart, LEN_HEADER);

    /* 2. 帧头 SOF 判断 */
    if (FrameHeader.SOF != JUDGE_FRAME_HEADER) return 0;

    /* 3. CRC8 校验 */
    if (Verify_CRC8_Check_Sum(ReadFromUsart, LEN_HEADER) != TRUE) return 0;

    /* 4. 整帧长度 + CRC16 校验 */
    judge_length = LEN_HEADER + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL;
    if (Verify_CRC16_Check_Sum(ReadFromUsart, judge_length) != TRUE) return 0;

    retval_tf = 1;

    /* 5. 取命令码 */
    CmdID = (ReadFromUsart[CMD_ID + 1] << 8) | ReadFromUsart[CMD_ID];

    /* 6. 根据 CmdID 解析数据 */
    switch (CmdID)
    {
        case ID_game_state:
            memcpy(&Game_status,  ReadFromUsart + DATA, LEN_game_state);
            break;

        case ID_game_result:
            memcpy(&Game_Result, ReadFromUsart + DATA, LEN_game_result);
            break;

        case ID_robot_HP:
            memcpy(&RobotHP, ReadFromUsart + DATA, LEN_robot_HP);
            break;

        /* case ID_darts_status:   // .h 中已删除
            memcpy(&Dart_Status, ReadFromUsart + DATA, LEN_darts_status);
            break; */

        case ID_event_data:
            memcpy(&Event_Data, ReadFromUsart + DATA, LEN_event_data);
            break;

        case ID_supply_action:
            memcpy(&Supply_Projectile_Action, ReadFromUsart + DATA, LEN_supply_action);
            break;

        case ID_judge_warning:
            memcpy(&Referee_Warning, ReadFromUsart + DATA, LEN_judge_warning);
            break;

        case ID_dart_remaining_time:
            memcpy(&dart_info, ReadFromUsart + DATA, LEN_darts_remaining_tim);
            break;

        case ID_robot_status:
            memcpy(&robot_status, ReadFromUsart + DATA, LEN_robot_status);
            break;

        case ID_robot_power:
            memcpy(&Power_Heat_Data, ReadFromUsart + DATA, LEN_robot_power);
            break;

        case ID_robot_position:
            memcpy(&robot_pos, ReadFromUsart + DATA, LEN_robot_position);
            break;

        case ID_robot_buff:
            memcpy(&Buff, ReadFromUsart + DATA, LEN_robot_buff);
            break;

        /* case ID_AerialRobotEnergy:  // .h 中已删除
            memcpy(&Aerial_Robot_Energy, ReadFromUsart + DATA, LEN_AerialRobotEnergy);
            break; */

        case ID_robot_hurt:
            memcpy(&hurt_data, ReadFromUsart + DATA, LEN_robot_hurt);
            break;

        case ID_shoot_data:
            memcpy(&Shoot_Data, ReadFromUsart + DATA, LEN_shoot_data);
            break;

        case ID_bullet_remaining:
            memcpy(&Bullet_Remaining, ReadFromUsart + DATA, LEN_bullet_remaining);
            break;

        case ID_RFID_status:
            memcpy(&RFID_Status, ReadFromUsart + DATA, LEN_RFID_status);
            break;

        case ID_dart_client:
            memcpy(&Dart_Client, ReadFromUsart + DATA, LEN_dart_client);
            break;

        case ID_robot_interact:
            memcpy(&Comm_getdata, ReadFromUsart + DATA, LEN_robot_interract);
            break;
//		case ID_map_command                  : memcpy(&map_command                 , ReadFromUsart + DATA, LEN_map_command                 );break;
//        case ID_map_robot_data               : memcpy(&map_robot_data              , ReadFromUsart + DATA, LEN_map_robot_data              );break;

        default:
            break;
    }

    /* 7. 如果本帧后面紧跟着又一帧 0xA5，则递归处理 */
    if (*(ReadFromUsart + judge_length) == JUDGE_FRAME_HEADER)
        Judge_Read_Data(ReadFromUsart + judge_length);

    return retval_tf;
}
