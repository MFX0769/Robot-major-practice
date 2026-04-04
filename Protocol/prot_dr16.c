/**
  * @file     remote_msg.c
  * @version  v3.1
  * @date     July,22 2019
  *
  * @brief    ???????????????
  *         
  *	@author   ??ZJ????????
              ????? 2025??6?????????????????

  *
  */


/* Includes ------------------------------------------------------------------*/


/* Includes ------------------------------------------------------------------*/
#include "prot_dr16.h"
#include "prot_remote_fusion.h"

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/


rc_info_t rc;

/* 灵敏度数据 */
scale_t scale;

/* 键盘按键状态标志位 */
uint8_t kb_status[12]= {0};
/* Private type --------------------------------------------------------------*/
/*dr16断联计数器*/
uint16_t dr16_online_cnt = 0;
/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/


/**
 * @brief 从DR16原始数据融合
 * @note  在此处完成方向转换：向上为正，向左为正
 */
void FuseFromDR16(void)
{
    // 鼠标
    remote_fusion.mouse.x = rc.mouse.x;
    remote_fusion.mouse.y = rc.mouse.y;
    remote_fusion.mouse.z = rc.mouse.z;
    remote_fusion.mouse.left = rc.mouse.l;
    remote_fusion.mouse.right = rc.mouse.r;
    
    // 键盘
    remote_fusion.keyboard.key_code = rc.kb.key_code;
    
    // 摇杆 - 从原始值转换为带符号值
    // 方向约定：向上为正，向左为正（以 remote_fusion 定义为准：ch3水平/ch4垂直）
    // 原始值范围: 364~1684, 中值1024
    remote_fusion.channel.ch1 = (int16_t)(1024 - rc.ch1);   // 右摇杆水平 (向左为正)
    remote_fusion.channel.ch2 = (int16_t)(rc.ch2 - 1024);   // 右摇杆垂直 (向上为正)
    remote_fusion.channel.ch3 = (int16_t)(1024 - rc.ch3);   // remote_fusion.ch3: 左摇杆水平 (向左为正)
    remote_fusion.channel.ch4 = (int16_t)(rc.ch4 - 1024);   // remote_fusion.ch4: 左摇杆垂直 (向上为正)
    remote_fusion.channel.dial = (int16_t)(rc.ch5 - 1024); // 拨轮
    
    // 死区处理
    if (remote_fusion.channel.ch1 > -RC_DEADZONE && remote_fusion.channel.ch1 < RC_DEADZONE) remote_fusion.channel.ch1 = 0;
    if (remote_fusion.channel.ch2 > -RC_DEADZONE && remote_fusion.channel.ch2 < RC_DEADZONE) remote_fusion.channel.ch2 = 0;
    if (remote_fusion.channel.ch3 > -RC_DEADZONE && remote_fusion.channel.ch3 < RC_DEADZONE) remote_fusion.channel.ch3 = 0;
    if (remote_fusion.channel.ch4 > -RC_DEADZONE && remote_fusion.channel.ch4 < RC_DEADZONE) remote_fusion.channel.ch4 = 0;
    
    // 开关 - DR16 sw 已经是 1=上, 2=下, 3=中
    remote_fusion.sw.sw1 = rc.sw1;
    remote_fusion.sw.sw2 = rc.sw2;
    remote_fusion.sw.cns = rc.sw1;  // DR16 使用 sw1 作为 CNS
}

/**
 * @brief       handle received rc data
 * @param[out]  rc:   structure to save handled rc data
 * @param[in]   buff: the buff which saved raw rc data
 * @retval
 * @note 
 */

void rc_callback_handler(rc_info_t *rc, uint8_t *buff)
{
   /* ?????????? */
	  // remote_online.DT7_online_cnt = 0;
  //  rc_normal_flag = 1;
	dr16_online_cnt = 0;
   /* ??????? */
   rc->ch1 = (buff[0]      | buff[1]  << 8) & 0x07FF;
   rc->ch1 -= 1024;
   rc->ch2 = (buff[1] >> 3 | buff[2]  << 5) & 0x07FF;
   rc->ch2 -= 1024;
   rc->ch3 = (buff[2] >> 6 | buff[3]  << 2 | buff[4] << 10) & 0x07FF;
   rc->ch3 -= 1024;
   rc->ch4 = (buff[4] >> 1 | buff[5]  << 7) & 0x07FF;
   rc->ch4 -= 1024;
   rc->ch5=  (buff[16]     | buff[17] << 8) & 0x07FF;
   rc->ch5 -= 1024;
   rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
   rc->sw2 = (buff[5] >> 4) & 0x0003;

   if ((abs(rc->ch1) > 660) || \
       (abs(rc->ch2) > 660) || \
       (abs(rc->ch3) > 660) || \
       (abs(rc->ch4) > 660)    )
   {
       memset(rc, 0, sizeof(rc_info_t));
   }
   rc->mouse.x = buff[6] | (buff[7] << 8); // x axis
   rc->mouse.y = buff[8] | (buff[9] << 8);
   rc->mouse.z = buff[10] | (buff[11] << 8);
   
   
   rc->mouse.l = buff[12];
   rc->mouse.r = buff[13];

   rc->kb.key_code = buff[14] | buff[15] << 8; // key borad code
   
   FuseFromDR16();
}

/**
  * @brief       按键枚举常量映射函数
  * @param[out]  key: 所需按键当前状态
  * @param[in]   key_index: 需要检测的按键
  * @author      ZZJ
  */
static uint16_t key_map(key_index_e key_index)
{
    uint16_t key;
    switch( key_index )
    {
        case KB_Q:  key = rc.kb.bit.Q; break;
        case KB_E:  key = rc.kb.bit.E; break;
        case KB_R:  key = rc.kb.bit.R; break;
        case KB_F:  key = rc.kb.bit.F; break;
        case KB_G:  key = rc.kb.bit.G; break;
        case KB_Z:  key = rc.kb.bit.Z; break;
        case KB_X:  key = rc.kb.bit.X; break;
        case KB_C:  key = rc.kb.bit.C; break;
        case KB_V:  key = rc.kb.bit.V; break;
        case KB_B:  key = rc.kb.bit.B; break;
        case KB_CTRL:
                    key = rc.kb.bit.CTRL; break;
	   case KB_SHIFT:
		          key = rc.kb.bit.SHIFT;break;
        default:    break;
    }
    return key;
}

/**
  * @brief       键盘按键扫描
  * @param[out]  kb_status[key_index]: 按键按下并弹起后，状态标志位取反
  * @param[in]   key_index：	       被检测的按键序号
  * @note        按键按下后，给相应标志位取反,不支持长按
  * @author      ZZJ
  */
uint8_t keyboard_scan(key_index_e key_index)
{
    static uint8_t key_press_enable[11] = {0};
    uint16_t key = key_map(key_index);  //获取当前按键状态
    if( key && key_press_enable[key_index] == 0 )  //如果当前按键按下且之前没有按下
    {
        key_press_enable[key_index] = 1;  //记录上一次状态：防止重复进入
        kb_status[key_index] = !kb_status[key_index];  //全局状态标志位取反 0-1
    }
    else if( key == 0 )
    {
        key_press_enable[key_index] = 0;  //允许下一次进入
    }
		    return kb_status[key_index];

}

/**
  * @brief       键盘状态清零
  * @param[out]  NONE
  * @param[in]   NONE
  * @author      ZZJ
  */
void key_status_clear(key_index_e key_index)
{
    kb_status[key_index] = KEY_END;
}

/**
  * @brief       键盘按键扫描，并清除标志位
  * @param[out]  NONE
  * @param[in]   NONE
  * @note        按键按下后，给相应标志位取反,不支持长按
  * @author      ZZJ
  */
uint8_t key_scan_clear(key_index_e key_index)
{
    uint8_t res = 0;
    keyboard_scan(key_index);  //如果按下一次按键，则反转标志位
    if( kb_status[key_index] == KEY_RUN )
    {
        kb_status[key_index] = KEY_END;  //清除标志位
        res = 1;  //返回 TRUE
    }
    else if( kb_status[key_index] == KEY_END )
        res = 0; 
    return res;
}
