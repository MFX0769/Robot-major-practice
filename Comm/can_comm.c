/**
  ******************************************************************************
  * @file    can_comm.c
  * @author  ZZY 1950828677@qq.com
  * @brief   CAN 应用层：过滤表 + 电机实例注册 + 收发
  * @date    2025-10
  * @version 1.2
  * @note    2026-11-25 本车只保留 CAN1：4 个底盘 2006（0x201~0x204）+ 1 个 6020（0x208）。
  *           原 yaw 6020 / trigger 2006 / 摩擦轮 / 达妙电机，以及 CAN2、CAN3 的收发路径已删除。

  * -----------------------------------CAN过滤器规则-----------------------------------
  * 过滤器初始化规则存放在 bsp_can1_filt_tbl，count 是规则条数（每条占 data 的 3 个 u16）。
  * 每条规则的格式为 type;id_high;id_low，Type 可选：
            * FLT_RANGE = 0x0000U
            * FLT_DUAL  = 0x0001U
            * FLT_MASK  = 0x0002U
  * count 必须与 Core/Src/fdcan.c 的 hfdcan1.Init.StdFiltersNbr 一致。
  *   2026-11-25 修正：StdFiltersNbr 原为 1（原厂模板值），硬件只认第 1 条规则，
  *   电机反馈 0x201~0x204/0x208 全部被丢弃；现与 4 条规则一致。

*/

/* Includes ------------------------------------------------------------------*/
#include "can_comm.h"
#include "dbg_print.h"
#include "config.h"

/* Private define ------------------------------------------------------------*/
#define TX_RETRY_MAX       5      // CAN等邮箱：总等待 ≈5ms
#define TX_RETRY_DELAY_MS  1      // CAN等邮箱：每次检查间隔 1ms
/* External variables ---------------------------------------------------------*/
extern dji_motor_t chassis_2006[4];
extern dji_motor_t arm_yaw_motor;
uint32_t can2_rx_count;

/* Private function declarations ---------------------------------------------*/
extern QueueHandle_t CAN1_Tx_queue;
extern QueueHandle_t CAN2_Tx_queue;

/*-----------------------------CAN初始化开始-----------------------------------*/
/**
* @brief  初始化CAN：过滤表 + 电机实例
* @retval None
*/
void comm_can_init(void)
{
    /*-----------用户在此修改过滤器的规则，最多10条规则-----------*/
    bsp_can1_filt_tbl.count = 4;
    bsp_can1_filt_tbl.data[0] = FLT_RANGE;
    bsp_can1_filt_tbl.data[1] = 0x017U;  // IMU 0x017~0x019
    bsp_can1_filt_tbl.data[2] = 0x019U;
    bsp_can1_filt_tbl.data[3] = FLT_RANGE;
    bsp_can1_filt_tbl.data[4] = 0x201U;  // 4 个底盘 2006 + 6020
    bsp_can1_filt_tbl.data[5] = 0x204U;
    bsp_can1_filt_tbl.data[6] = FLT_RANGE;
    bsp_can1_filt_tbl.data[7] = 0x208U;  // 备用：6020 若改接 CAN1
    bsp_can1_filt_tbl.data[8] = 0x208U;
    bsp_can1_filt_tbl.data[9] = FLT_RANGE;
    bsp_can1_filt_tbl.data[10] = 0x205U; // 空闲槽位 0x205~0x207，供 USB-CAN 诊断
    bsp_can1_filt_tbl.data[11] = 0x207U;

    /* CAN2：仅 6020，ID4 → 反馈 0x208。 */
    bsp_can2_filt_tbl.count = 1;
    bsp_can2_filt_tbl.data[0] = FLT_RANGE;
    bsp_can2_filt_tbl.data[1] = CAN_ARM_YAW_MOTOR_ID;
    bsp_can2_filt_tbl.data[2] = CAN_ARM_YAW_MOTOR_ID;

    /*调用函数完成初始化*/
    bsp_can_init(&bsp_user_can1, User_CAN1_RxCpltCallback);
    bsp_can_init(&bsp_user_can2, User_CAN2_RxCpltCallback);

    /* 6020 实际接在板上 FDCAN2（PB5/PB6）。 */
    dji_motor_init(&arm_yaw_motor, DJI_6020_MOTOR, PROT_CAN_CHANNEL_2, CAN_ARM_YAW_MOTOR_ID, 1.0f);

    /* 底盘2006电机初始化（全在 CAN1） */
    dji_motor_init(&chassis_2006[0], DJI_2006_MOTOR, PROT_CAN_CHANNEL_1, CHA_LEFT_FRONT_2006, DJI_2006_ORIGINAL_REDUCTION_RATIO);
    dji_motor_init(&chassis_2006[1], DJI_2006_MOTOR, PROT_CAN_CHANNEL_1, CHA_LEFT_BACK_2006, DJI_2006_ORIGINAL_REDUCTION_RATIO);
    dji_motor_init(&chassis_2006[2], DJI_2006_MOTOR, PROT_CAN_CHANNEL_1, CHA_RIGHT_FRONT_2006, DJI_2006_ORIGINAL_REDUCTION_RATIO);
    dji_motor_init(&chassis_2006[3], DJI_2006_MOTOR, PROT_CAN_CHANNEL_1, CHA_RIGHT_BACK_2006, DJI_2006_ORIGINAL_REDUCTION_RATIO);
}

/*-----------------------------CAN初始化结束-----------------------------------*/

/**
* @brief  Callback function in CAN Interrupt
* @return None.
*/
void User_CAN1_RxCpltCallback(CAN_RxBuffer_t* buf)
{
    switch (bsp_user_can1.CAN_RxBuffer.rxHeader.Identifier)
    {
        case TIMU_PALSTANCE_ID:
        {
            T_imu_calcu(bsp_user_can1.CAN_RxBuffer.rxHeader.Identifier, bsp_user_can1.CAN_RxBuffer.rx_data);
            break;
        }
        case TIMU_ANGLE_ID:
        {
            T_imu_calcu(bsp_user_can1.CAN_RxBuffer.rxHeader.Identifier, bsp_user_can1.CAN_RxBuffer.rx_data);
            break;
        }
        case TIMU_ROL_ID:
        {
            T_imu_calcu(bsp_user_can1.CAN_RxBuffer.rxHeader.Identifier, bsp_user_can1.CAN_RxBuffer.rx_data);
            break;
        }
        case CHA_LEFT_FRONT_2006:
        case CHA_LEFT_BACK_2006:
        case CHA_RIGHT_FRONT_2006:
        case CHA_RIGHT_BACK_2006:
        {
            uint32_t id = bsp_user_can1.CAN_RxBuffer.rxHeader.Identifier;
            uint8_t idx;
            if      (id == CHA_LEFT_FRONT_2006) idx = 0;
            else if (id == CHA_LEFT_BACK_2006)  idx = 1;
            else if (id == CHA_RIGHT_FRONT_2006) idx = 2;
            else if (id == CHA_RIGHT_BACK_2006)  idx = 3;
            else break;  /* 未知 CAN ID，不应到达 */
            dji_motor_get_single_data(&chassis_2006[idx], bsp_user_can1.CAN_RxBuffer.rx_data);
            break;
        }
        default:break;
    }
}

void User_CAN2_RxCpltCallback(CAN_RxBuffer_t* buf)
{
    (void)buf;
    if (bsp_user_can2.CAN_RxBuffer.rxHeader.Identifier == CAN_ARM_YAW_MOTOR_ID)
    {
        dji_motor_get_single_data(&arm_yaw_motor, bsp_user_can2.CAN_RxBuffer.rx_data);
        can2_rx_count++;
    }
}

/*-----------------------------底盘电机发送函数-----------------------------------*/
void chassis_can_send(void)
{
    /* 4 个底盘电机按 ID 打包到 0x200；6020 当前仅注册反馈，控制入口后续接入。 */
    dji_send_current(&chassis_2006[0], &chassis_2006[1], &chassis_2006[2], &chassis_2006[3]);
}


void Prot_CAN1_Send_Frame(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    CanFrame_t msg = {0};

    msg.ID   = id;
    msg.DLC  = (dlc > 8) ? 8 : dlc;

    if (data != NULL)
    {
        memcpy(msg.Data, data, msg.DLC);
    }

    BaseType_t ret = xQueueSend(CAN1_Tx_queue, &msg, 0);

    if (ret != pdPASS)
    {
#if DEBUG_TEXT_ENABLE
        dbg_print("[CAN1TX] queue full, dropped id=0x%03X\r\n", (unsigned int)id);
#endif
    }
}

void Prot_CAN2_Send_Frame(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    CanFrame_t msg = {0};
    msg.ID = id;
    msg.DLC = (dlc > 8) ? 8 : dlc;
    if (data != NULL) memcpy(msg.Data, data, msg.DLC);
    (void)xQueueSend(CAN2_Tx_queue, &msg, 0);
}

/*-----------------------------CAN发送任务 CAN Send task (queue receive)-----------------------------------*/

/**
* @brief  CAN发送任务
* @return None.
*/
void Task_CAN1Transmit(void const *arg)
{
  CanFrame_t msg;
  (void)arg;
  for (;;)
  {
      if (xQueueReceive(CAN1_Tx_queue, &msg, portMAX_DELAY) == pdPASS)
      {
          uint32_t retry = 0;
          while (retry < TX_RETRY_MAX &&
                 bsp_can_std_transmit(CAN_CHANNEL_1, msg.ID, msg.Data) != BSP_CAN_OK)
          {
              if (++retry < TX_RETRY_MAX) vTaskDelay(pdMS_TO_TICKS(TX_RETRY_DELAY_MS));
          }
      }
  }
}

/**
* @brief  CAN2发送任务（6020 零电流帧）
* @return None.
*/
void Task_CAN2Transmit(void const *arg)
{
  CanFrame_t msg;
  (void)arg;
  for (;;)
  {
      if (xQueueReceive(CAN2_Tx_queue, &msg, portMAX_DELAY) == pdPASS)
      {
          uint32_t retry = 0;
          while (retry < TX_RETRY_MAX &&
                 bsp_can_std_transmit(CAN_CHANNEL_2, msg.ID, msg.Data) != BSP_CAN_OK)
          {
              if (++retry < TX_RETRY_MAX) vTaskDelay(pdMS_TO_TICKS(TX_RETRY_DELAY_MS));
          }
      }
  }
}
