//#include "log_task.h"
//#include "data_log.h"
//#include "cmsis_os.h"
//#include "prot_T_imu.h"
//#include "prot_rm_judge.h"
//#include "mode_switch_task.h"
//#include "prot_vision.h"
//#include "prot_FricMotor.h"
//#include "prot_TriggerMotor.h"
//#include "gimbal_task.h"
//#include "prot_dm_motor.h"
//#include "shoot_task.h"
//#include <stdio.h>


///*****************全局变量定义**********************/
////robot_state_e robot_current_state = ROBOT_STATE_INIT;

///*****************数据结构体定义**********************/
//extern pid_t pid_trigger_ecd;
//extern pid_t pid_trigger_spd;


///**
//  * @brief  初始化空中机器人日志系统
//  * @param  huart: 使用的串口句柄
//  */
//void AirRobot_Log_Init(void)
//{
//    // 初始化日志模块，缓冲区长度200，发送周期50ms
//    Log_Init(&huart3, 200, 10);

//	//HAL_UART_Transmit(&huart5, (uint8_t*)"TEST\n", 5, 1000);
//}

///**
//  * @brief  日志任务，定期记录机器人状态信息
//  * @note   纯发送，不做任何数据拷贝
//  */
//void AirRobot_Log_Task(const void* argu)
//{
//    uint32_t thread_wake_time = osKernelSysTick();
//    
//    for (;;)
//    {
//        if (log_info.log_status == LOG_READY || log_info.log_status == LOG_NOTICE)
//        {
////            Log_Send("[time][%lu] ID=%d HP=%d/%d Lv=%d Camp=%s Bullet=%d@%dcm/s\r\n", 
////                osKernelSysTick(),
////                robot_status.robot_id,
////                robot_status.remain_HP,
////                robot_status.max_HP,
////                robot_status.robot_level,
////                (robot_status.robot_id > 100) ? "Blue" : "Red",
////                Bullet_Remaining.bullet_remaining_num_17mm,
////                (uint16_t)(Shoot_Data.bullet_speed * 100));

//            Log_Send("[MODE] Sys=%d Vis=%d Gimb=%d Lock=%d\r\n",
//                ctrl_mode, vision_mode, gimbal_ctrl_mode, lock_flag);

//            Log_Send("IMU: pitch=%.2f yaw=%.2f  wy=%.2f wz=%.2f\r\n",
//                imu_data.pitch, imu_data.yaw,
//                imu_data.wy, imu_data.wz);

//             /* ========== YAW电机（RM电机） ========== */
//            Log_Send("[YAW] TX=%d RX=%d A_REF%.2f Cur=%d\r\n",
//                yaw_motor.send_cnt,
//                yaw_motor.receive_cnt,
//                gimbal.yaw_angle.ref,           // 或者 moto_yaw.ecd，看你实际定义
//                yaw_motor.tx_current);    // 或者 moto_yaw.given_current

//            /* ========== PITCH电机（DM4310） ========== */
//            Log_Send("[PITCH] TX=%d RX=%d T_rotor=%.2f T_ref=%.2f Err=%d\r\n",
//                dm3507_motor.send_cnt,      // DM3507发送计数
//                dm3507_motor.receive_cnt,   // DM3507接收计数
//                gimbal.pit_angle.ref,       // 目标角度
//                dm3507_motor.t_ref,         // 目标转矩
//                dm3507_motor.err_code);     // 错误码

//            /* ========== 摩擦轮（RM电机数组） ========== */
//            Log_Send("FRIC: Mode=%d L_ref=%.0f L_fdb=%.0f L_rpm=%d R_ref=%.0f R_fdb=%.0f R_rpm=%d\r\n",
//                shoot.firc_mode,
//                fric[0].spd_ref,
//                fric[0].spd_fdb,
//                fric_motor[0].speed_rpm,    // RM电机实际转速
//                fric[1].spd_ref,
//                fric[1].spd_fdb,
//                fric_motor[1].speed_rpm);   // RM电机实际转速

//            /* ========== 拨盘电机 ========== */
//            Log_Send("TRIG: Mode=%d ECD_ref=%.0f ECD_fdb=%.0f SPD_ref=%.0f SPD_fdb=%.0f Cur=%d Flag=%d Freq=%d\r\n", 
//                shoot.stir_mode,
//                shoot.pid.trigger_ecd_ref,
//                shoot.pid.trigger_ecd_fdb,
//                shoot.pid.trigger_spd_ref,
//                shoot.pid.trigger_spd_fdb,
//                trigger_motor.tx_current,   // 或者 motor_trigger.given_current
//                shoot.SHOT_FLAG,
//                shoot.frequency_cnt);

//            Log_Send("----------------------------------------\r\n");
//        }
//        
//        osDelayUntil(&thread_wake_time, 10);
//    }
//}

/////**
////  * @brief  日志任务，定期记录机器人状态信息
////  */
////void AirRobot_Log_Task(const void* argu)
////{
//////#if INCLUDE_vTaskDelayUntil
//////    uint32_t thread_wake_time = osKernelSysTick();
//////#endif
////    uint32_t thread_wake_time = osKernelSysTick();
////    for (;;)
////    {
////		// 强制设置日志状态为可记录
//////        log_info.log_status = LOG_READY;
////        // 只记录系统就绪或注意状态下的日志
////        if (log_info.log_status == LOG_READY || log_info.log_status == LOG_NOTICE)
////        {
//////        // === 模拟裁判系统数据 ===
//////        log_data.referee.robot_id = 7;  // 模拟机器人ID
//////        log_data.referee.robot_hp = 1000;
//////        log_data.referee.max_hp = 2000;
//////        log_data.referee.chassis_power = 50.5f;
//////        log_data.referee.bullet_speed = 1500; // cm/s
//////        log_data.referee.bullet_remaining = 42;
//////        log_data.referee.camp = 0; // 红方
//////        robot_current_state = ROBOT_STATE_NORMAL;

//////        // === 模拟IMU数据 ===
//////        static float mock_yaw = 0.0f;
//////        mock_yaw += 5.0f;  // 每次任务循环增加5度
//////        if(mock_yaw > 360.0f) mock_yaw = 0.0f;
//////        
//////        log_data.imu.yaw = mock_yaw;
//////        log_data.imu.pitch = 10.0f;
//////        log_data.imu.roll = -2.5f;

//////        // === 模拟摩擦轮数据 ===
//////        log_data.fric_wheel.fric_mode = 1;
//////        log_data.fric_wheel.spd_ref[0] = 8000; 
//////        log_data.fric_wheel.spd_fdb[0] = 7900;
//////        log_data.fric_wheel.current[0] = 1000;
//////        log_data.fric_wheel.spd_ref[1] = 8000;
//////        log_data.fric_wheel.spd_fdb[1] = 8100;
//////        log_data.fric_wheel.current[1] = 1050;

//////        // === 模拟拨盘数据 ===
//////        log_data.trigger.stir_mode = 2;
//////        log_data.trigger.ecd_ref = 1800;
//////        log_data.trigger.ecd_fdb = 1790;
//////        log_data.trigger.spd_ref = 200;
//////        log_data.trigger.spd_fdb = 195;
//////        log_data.trigger.current = 3000;
//////        log_data.trigger.shot_flag = 1;
//////        log_data.trigger.frequency_cnt = 3;
////			Log_Update_Mode_Status();
////			AirRobot_Log_UpdateFromReferee();
////            // 记录裁判系统数据

////         Log_Send("[time][%lu] ", osKernelSysTick());
////         Log_Send("ID=%d HP=%d/%d Camp=%s  Bullet=%d@%dcm/s\r\n", 
////                    log_data.referee.robot_id,
////                    log_data.referee.robot_hp,
////                    log_data.referee.max_hp,
//////					log_data.referee.robot_level,
////					log_data.referee.camp ? "Blue" : "Red",
//////					log_data.referee.chassis_power,
////					log_data.referee.bullet_remaining,
////                    log_data.referee.bullet_speed
////					);


////				
////			//记录模式
////			Log_Send("[MODE] Sys=%d Vis=%d Gimb=%d Lock=%d\r\n",
////                    log_data.referee.system_mode,
////                    log_data.referee.vision_submode,
////                    log_data.referee.gimbal_distance_mode,
////                    log_data.referee.lock_status);
////										
////										
////            // 记录IMU数据
////			AirRobot_Log_UpdateIMUData();
////            Log_Send("IMU Data: pitch=%.2f°, yaw=%.2f°, wy=%.2f° , wz=%.2f\r\n",
////                imu_data.pitch,
////                imu_data.yaw,
////								imu_data.wy,
////								imu_data.wz);


////			//yaw电机在线数据
////      Log_Send("[YAW_Online] can_send=%d can_receive=%d ",
////                    can_send_cnt,
////                    yaw_can_receive_cnt);
////										
////			//pitch电机在线数据
////      Log_Send("[PITCH_Online] 4310_can_send=%d 4310_can_receive=%d ",
////                    motor_dm4310_pitch.can_send_cnt  ,
////                    motor_dm4310_pitch.can_receive_cnt);
////					
////					
////			//记录摩擦轮电机数据
////			AirRobot_Log_UpdateFricData();
////            Log_Send("FricMotor: Mode=%d Left(Ref/Fdb/Cur)=%.0f/%.0f/%.0f Right(Ref/Fdb/Cur)=%.0f/%.0f/%.0f\r\n",
////			log_data.fric_wheel.fric_mode,
////			log_data.fric_wheel.spd_ref[0], 
////            log_data.fric_wheel.spd_fdb[0],
////            log_data.fric_wheel.current[0],
////			log_data.fric_wheel.spd_ref[1], 
////            log_data.fric_wheel.spd_fdb[1],
////            log_data.fric_wheel.current[1]);

////			
////			//记录拨盘电机数据
////			AirRobot_Log_UpdateTriggerData();
////            Log_Send("Trigger: Mode=%d ECD(Ref/Fdb)=%.0f/%.0f SPD(Ref/Fdb)=%.0f/%.0f Cur=%.0f Flag=%d Freq=%d\r\n", 
////			log_data.trigger.stir_mode,
////			log_data.trigger.ecd_ref, 
////            log_data.trigger.ecd_fdb,
////			log_data.trigger.spd_ref,
////            log_data.trigger.spd_fdb,		
////			log_data.trigger.current,
////			log_data.trigger.shot_flag,
////             log_data.trigger.frequency_cnt);


////            Log_Send("----------------------------------------\r\n");
////        }
////		osDelayUntil(&thread_wake_time, 10);
//////#if INCLUDE_vTaskDelayUntil
//////        osDelayUntil(&thread_wake_time, log_info.log_term);
//////#else
//////        osDelay(log_info.log_term);
//////#endif
////    }
////}

////void AirRobot_Log_Task(const void* argu) {
////    uint32_t wake_time = osKernelSysTick();
////    for (;;) {
////		HAL_UART_Transmit(&huart5, (uint8_t*)"Log Task Started!\r\n", 18, 100); // 替换为实际串口
//////        // 1. 模拟数据（替代硬件依赖）
//////        log_data.referee.robot_id = 7;
//////        log_data.referee.robot_hp = 1000;
//////        log_data.imu.yaw = 45.0f;

//////        // 2. 发送日志（带调试标记）
//////        Log_Send("[LOG] ID=%d, HP=%d, Yaw=%.1f\r\n", 
//////                log_data.referee.robot_id,
//////                log_data.referee.robot_hp,
//////                log_data.imu.yaw);
//////		printf("HELLO,WORLD");

////        // 3. 打印任务存活信息（直接阻塞发送，绕过Log_Send）
////// HAL_UART_Transmit(&huart5, "[DBG] Log Task Alive\r\n", 20, 100);
////Log_Send("Hello World!");
////        // 4. 精确延时
////        osDelayUntil(&wake_time, 5);
////    }
////}
