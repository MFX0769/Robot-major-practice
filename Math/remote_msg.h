//#ifndef __REMOTE_MSG_H__
//#define __REMOTE_MSG_H__

//#include "stm32h7xx_hal.h"


//#include "stdbool.h"


////#define DBUS_MAX_LEN  50
////#define DBUS_BUFLEN   18
//#define NEW_RC        0      //2025年4月21日新增新遥控器，若此宏定义为1，则使用新遥控器，否则依然使用旧遥控器。只需要改这里，其他所有都不用改（通道，modeswitch等）


///*===================================遥控器在线检测相关开始==================================*/

//typedef struct
//{
//uint8_t DT7_online_flag;
//int32_t DT7_online_cnt;
//int32_t DT7_last_online_cnt;
//uint8_t NEW_RC_online_flag;
//int32_t NEW_RC_online_cnt;
//int32_t NEW_RC_last_online_cnt;

//uint8_t NEW_OLD_RC_FLAG;  //0为旧遥控（默认）  1为新遥控（旧遥控bad时）
//} remote_online_t;
//extern remote_online_t remote_online;
///*===================================遥控器在线检测相关结束==================================*/
///* channel sensitivity */
//typedef struct
//{
//float ch1;
//float ch2;
//float ch3;
//float ch4;
//float ch5;
//} scale_t;

///* remote control information structure */
//typedef __packed struct
//{
///* rocker channel information */
//int16_t ch1;
//int16_t ch2;
//int16_t ch3;
//int16_t ch4;
//int16_t ch5;
///* left and right lever information */
//uint8_t sw1;
//uint8_t sw2;

///* mouse movement and button information */
//__packed struct
//{
//	int16_t x;
//	int16_t y;
//	int16_t z;

//	uint8_t l;
//	uint8_t r;
//} mouse;
///* keyboard key information */
//__packed union
//{
//	uint16_t key_code;
//	__packed struct
//	{
//		uint16_t W:1;
//		uint16_t S:1;
//		uint16_t A:1;
//		uint16_t D:1;
//		uint16_t SHIFT:1;
//		uint16_t CTRL:1;
//		uint16_t Q:1;
//		uint16_t E:1;
//		uint16_t R:1;
//		uint16_t F:1;
//		uint16_t G:1;
//		uint16_t Z:1;
//		uint16_t X:1;
//		uint16_t C:1;
//		uint16_t V:1;
//		uint16_t B:1;
//	} bit;
//} kb;
//} rc_info_t;

///*=============================新遥控相关变量开始=============================*/
//typedef __packed struct
//{
//uint8_t sof_1:8;
//uint8_t sof_2:8;
//int64_t ch_0:11;
//int64_t ch_1:11;
//int64_t ch_2:11;
//int64_t ch_3:11;
//uint64_t mode_sw:2;
//uint64_t pause:1;
//uint64_t fn_1:1;
//uint64_t fn_2:1;
//int64_t wheel:11;
//uint64_t trigger:1;

//int16_t mouse_x:16;
//int16_t mouse_y:16;
//int16_t mouse_z:16;
//uint8_t mouse_left:2;
//uint8_t mouse_right:2;
//uint8_t mouse_middle:2;
//uint8_t reserve:2;

//	uint8_t key_W:1;
//	uint8_t key_S:1;
//	uint8_t key_A:1;
//	uint8_t key_D:1;
//	uint8_t key_SHIFT:1;
//	uint8_t key_CTRL:1;
//	uint8_t key_Q:1;
//	uint8_t key_E:1;
//	uint8_t key_R:1;
//	uint8_t key_F:1;
//	uint8_t key_G:1;
//	uint8_t key_Z:1;
//	uint8_t key_X:1;
//	uint8_t key_C:1;
//	uint8_t key_V:1;
//	uint8_t key_B:1;
//	
//uint16_t crc16;
//}new_rc_info_t;

//typedef enum
//{
//RC_MODE_C = 0,//保护
//RC_MODE_N = 1,//遥控
//RC_MODE_S = 2,//键鼠
//}new_rc_mode_sw_e;//新遥控挡位切换开关  C:0 N:1 S:2




//static uint16_t crc16_init = 0xffff;
//static const uint16_t crc16_tab[256] =
//{
//0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
//0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
//0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
//0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
//0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
//0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
//0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
//0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
//0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
//0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
//0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
//0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
//0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
//0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
//0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
//0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
//0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
//0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
//0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
//0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
//0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
//0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
//0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
//0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
//0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
//0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
//0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
//0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
//0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
//0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
//0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
//0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
//};
///*=============================新遥控相关变量结束=============================*/

///* PC_Key_Definition */
//typedef enum
//{
//KB_Q = 0,
//KB_E = 1,
//KB_R = 2,
//KB_F = 3,
//KB_G = 4,
//KB_Z = 5,
//KB_X = 6,
//KB_C = 7,
//KB_V = 8,
//KB_B = 9,
//KB_CTRL = 10,
//KB_SHIFT=11,
//} key_index_e;

//typedef enum
//{
//RC_UP = 1,
//RC_MI = 3,
//RC_DN = 2,
//} rc_sw_mode_e;




//typedef enum
//{
//KEY_RUN = 1,
//KEY_END = 0
//} rc_key_status_e;


///*重要！！！*/
//extern uint8_t RC_FAIL_FLAG;
///*重要！！！*/


//extern rc_info_t rc;
//extern new_rc_info_t new_rc;
//extern scale_t scale;
//extern uint8_t rc_normal_flag;
//extern uint8_t kb_status[12];

//void rc_callback_handler(rc_info_t *rc, uint8_t *buff);
//uint8_t keyboard_scan(key_index_e key_index);
//void key_status_clear(key_index_e key_index);
//uint8_t key_scan_clear(key_index_e key_index);
//void NEW_OLD_RC_Switch(void);
//void new_rc_callback_handler(new_rc_info_t *new_rc, uint8_t *buff);
//bool verify_crc16_check_sum(uint8_t *p_msg, uint16_t len);
//#endif
