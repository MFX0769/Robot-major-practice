/**
  * @file     remote_msg.c
  * @version  v3.1
  * @date     July,22 2019
  *
  * @brief    遥控器数据解算文件
  *         
  *	@author   ZZJ（原作者）
              满分笑 2025年6月添加多遥控兼容的代码

  *
  */
#include "remote_msg.h"
#include "stdlib.h"
#include "string.h"
#include "gimbal_task.h"
#include "modeswitch_task.h"
#include "bsp_judge.h"


/*新旧遥控切换*/
//#define NEW_RC 1 此宏定义写在头文件里面

/*遥控器失联处理&控制链路切换相关变量*/
uint8_t RC_FAIL_FLAG;//0为遥控器正常，1为遥控器失联，切图传链路


#if NEW_RC
  remote_online_t remote_online = {
     . DT7_online_flag=0,
	   . DT7_online_cnt=0,
	   . NEW_RC_online_flag=0,
	   . NEW_RC_online_cnt=0,
		 . NEW_OLD_RC_FLAG=1,//手动指定新遥控
   };

#else
		remote_online_t remote_online = {
     . DT7_online_flag=0,
	   . DT7_online_cnt=0,
	   . NEW_RC_online_flag=0,
	   . NEW_RC_online_cnt=0,
		 . NEW_OLD_RC_FLAG=0,//默认旧遥控
   };
#endif	
	 
/* 遥控器数据 */
rc_info_t rc;
new_rc_info_t new_rc;

/* 遥控器数据传输状态标志 */
uint8_t rc_normal_flag = 0;

/* 灵敏度数据 */
scale_t scale;

/* 键盘按键状态标志位 */
uint8_t kb_status[12]= {0};

/*静态函数声明*/
static uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16);



/*===========================新旧遥控切换=======================*/
void NEW_OLD_RC_Switch(void)
{
	static uint8_t NEW_OLD_RC_CNT;
	
	#if NEW_RC
	
	#else
	/*优先用旧遥控，如果没收到旧遥控数据，就自动切换成新遥控的数据*/

	remote_online.DT7_online_cnt++;
	
	if((remote_online.DT7_online_cnt) >= 5) //新旧遥控都是14ms发一次数据，也就是71.48，如果少了50，就判定为旧遥控bad了
	{
//		remote_online.NEW_OLD_RC_FLAG = 1;//切换成新遥控！！
	}
	
	#endif
	
	if(new_rc.fn_1 == 1 || new_rc.key_R == 1 )
	{
//		remote_online.NEW_OLD_RC_FLAG = 1;//切换成新遥控！！
	}
	

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
    /* 遥控器通信检测 */
	  remote_online.DT7_online_cnt = 0;
    rc_normal_flag = 1;
    /* 数据解算 */
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
//        memset(rc, 0, sizeof(rc_info_t));
    }
    rc->mouse.x = buff[6] | (buff[7] << 8); // x axis
    rc->mouse.y = buff[8] | (buff[9] << 8);
    rc->mouse.z = buff[10] | (buff[11] << 8);

    rc->mouse.l = buff[12];
    rc->mouse.r = buff[13];

    rc->kb.key_code = buff[14] | buff[15] << 8; // key borad code
}


//typedef __packed struct
//{
//	int16_t mouse_x;
//	int16_t mouse_y;
//	int16_t mouse_z;
//	int8_t left_button_down;
//	int8_t right_button_down;
//	uint16_t keyboard_value;
//	uint16_t reserved;
//}ext_remote_control_t;
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


/*=============================================新遥控接收代码开始=============================================*//**
  * @brief       新遥控（RC N3）的数据解算函数
  * @param[out]  NONE
  * @param[in]   NONE
  * @note        2025年7月1日 未测试
  * @author      
  */
void new_rc_callback_handler(new_rc_info_t *new_rc, uint8_t *buff)
{
    /* 遥控器通信检测 */
	  remote_online.NEW_RC_online_cnt++;
    rc_normal_flag = 1;
    /* 数据解算 */
    if(buff[0]==0xA9 && buff[1]==0x53)//帧头
    {
        if(verify_crc16_check_sum(buff,21)==true)// 新遥控传输的数据长度为21字节，减去2字节的校验结果为19字节
        {
            memcpy(new_rc,buff,21);//将dma数据直接复制到联合体里面
					
					 new_rc->ch_0 -= 1024;
					 new_rc->ch_1 -= 1024;
					 new_rc->ch_2 -= 1024;
					 new_rc->ch_3 -= 1024;
					 new_rc->wheel -= 1024;

        }
    }

    
}


/**
 * @brief Get the crc16 checksum
 *
 * @param p_msg Data to check
 * @param lenData length
 * @param crc16 Crc16 initialized checksum
 * @return crc16 Crc16 checksum
 */
static uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16)
{
    uint8_t data;

    if(p_msg == NULL)
    {
        return 0xffff;
    }

    while(len--)
    {
        data = *p_msg++;
        (crc16) = ((uint16_t)(crc16) >> 8) ^ crc16_tab[((uint16_t)(crc16) ^ (uint16_t)(data)) & 0x00ff];
    }

    return crc16;
}

/**
 * @brief crc16 verify function
 *
 * @param p_msg Data to verify
 * @param len Stream length=data+checksum
 * @return bool Crc16 check result
 */
bool verify_crc16_check_sum(uint8_t *p_msg, uint16_t len)
{
    uint16_t w_expected = 0;

    if((p_msg == NULL) || (len <= 2))
    {
        return false;
    }
    w_expected = get_crc16_check_sum(p_msg, len - 2, crc16_init);

    return ((w_expected & 0xff) == p_msg[len - 2] && ((w_expected >> 8) & 0xff) == p_msg[len - 1]);
}

/*=============================================新遥控接收代码结束=============================================*/
