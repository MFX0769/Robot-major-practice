/**
  * @file  prot_vision.h
  * @brief �Ӿ����ڽ��� + ��̨�ο��������С�棩
  */

#ifndef __PROT_VISION_H__
#define __PROT_VISION_H__
#include <stdint.h>
#include <math.h>

#define VISION_DATA_LEN  24
#define VISION_SP_RX_DATA_LEN 29
#define VISION_SP_TX_DATA_LEN 44
#define VISION_SP_FIRE_YAW_ERR 1.5f   // 度
#define VISION_SP_FIRE_PIT_ERR 1.5f   // 度

/* 采摘水果视觉协议帧定义 */
#define VISION_SOF       0xAA
#define VISION_EOF       0xBB
#define VISION_FRUIT_DATA_LEN  7
typedef enum {
    vMODE_AUTO        = 0,    /* ���� */
    vMODE_bENERGY     = 1,    /* ���������� */
    vMODE_sENERGY     = 2,    /* С�������� */
    vMODE_ANTISPIN    = 4,    /* ��Զ���뷴С����ģʽ */
} vision_mode_e;

/* �Ӿ�״̬ */
typedef enum {
    UNAIMING   = 0,
    AIMING     = 1,
    FIRST_LOST = 2,
} vision_aim_status_e;

typedef enum {
    NORMAL      = 0,
    TAIL_ERROR  = 1,
    REPEAT_ERROR= 2,
} vision_rx_status_e;

typedef struct
{
    /*收发计数*/
	uint32_t rx_cnt;
    uint32_t tx_cnt;
	
    /*收发帧率*/
	double rx_freq;
    float  tx_freq;
	
    vision_rx_status_e rx_status;
    vision_aim_status_e aim_status;
	uint32_t rx_repeat_cnt;
    uint32_t new_frame_flag;
	uint32_t last_rx_tick;
    float target_yaw_angle, target_pit_angle;
	float min_err;

    float yaw_min_err;
	float pit_min_err;
    uint32_t shoot_enable;
	uint8_t trace_id;
    uint8_t online;
    __packed union
    {
        uint8_t buff[VISION_DATA_LEN];
         __packed struct
        {
            float yaw;
            float pit;
			float yaw_vel;
			float yaw_acc;
			float pit_vel;
			float pit_acc;
            float dis;
//						float dis2;
            float fire;
            float pos;
            uint8_t empty;
            uint8_t cnt : 6;
            uint8_t ist_flag :1;
            uint8_t aim_flag :1;
						uint8_t trace_id;
            uint8_t eof;
        } data;
    } rx[2];
    
    __packed union
    {
        uint8_t buff[8];
         __packed struct
        {
            uint8_t vision_shoot_enable;
            uint8_t start_up_flag;
            uint8_t empty2;
            uint8_t empty3;
            uint8_t empty4;
            uint8_t empty5;
            uint8_t empty6;
            uint8_t empty7;            
        } data;
    } rxt;
    
} vision_t;

typedef union {
/*==============================2025��4��12�� �����Ӿ�tx��ʽʹ��Ͳ���ͳһ=============================*/
    uint8_t buff[25];//VISION_TX_BUFF_LEN
    
    __packed struct {
        uint8_t SOF;  //֡ͷ
        uint8_t imu_pit[4];
        uint8_t imu_yaw[4];  
        uint8_t imu_pit_spd[4];
        uint8_t imu_yaw_spd[4];
        
			uint8_t vacancy:1;
			uint8_t camp:1;
			uint8_t aiming_mode:3;
      uint8_t shooter_speed:3;
			
			uint8_t trace_id;
			uint8_t shoot_speed[4];
			uint8_t eof1;  //֡β1
			uint8_t eof2;  //֡β2

    } data;
		
/*==============================�´������=============================*/
}__attribute__((packed)) vision_tx_t;



/*---------------------新视觉结构体-------------------------------*/
typedef enum
{
    VISION_SP_MODE_IDLE = 0,
    VISION_SP_MODE_AUTO_AIM = 1,
    VISION_SP_MODE_SMALL_BUFF = 2,
    VISION_SP_MODE_BIG_BUFF = 3
} vision_sp_mode_e;

typedef enum {
	VISION_SP_RX_MODE_NO_CONTROL = 0,
    VISION_SP_RX_MODE_CONTROL        = 1,   
    VISION_SP_RX_MODE_FIRE     = 2,   
} VISION_SP_RX_MODE_e;

typedef struct
{
    uint8_t head[2];
    uint8_t mode;
    uint8_t yaw[4];        // rad
    uint8_t yaw_vel[4];    // rad/s
    uint8_t yaw_acc[4];    // rad/s^2
    uint8_t pitch[4];      // rad
    uint8_t pitch_vel[4];  // rad/s
    uint8_t pitch_acc[4];  // rad/s^2
    uint8_t crc16[2];
} vision_rx_msg_sp_t;

typedef struct
{
    uint8_t head[2];
    uint8_t mode;
    uint8_t q[4][4];        // w, x, y, z
    uint8_t yaw[4];         // rad
    uint8_t yaw_vel[4];     // rad/s
    uint8_t pitch[4];       // rad
    uint8_t pitch_vel[4];   // rad/s
    uint8_t bullet_speed[4];// m/s
    uint8_t bullet_count[2];
    uint8_t camp;
    uint8_t crc16[2];
} vision_tx_msg_sp_t;

/* 采摘水果视觉数据结构 */
typedef struct {
    uint8_t flag;          /* 目标检测标志 */
    int16_t x_dis;         /* X方向像素差值 (有符号) */
    int16_t y_dis;         /* Y方向像素差值 (有符号) */
    uint32_t rec_cnt;      /* 接收计数器 */
    uint8_t target_type;   /* 目标类型 (扩展用) */
} vision_fruit_msg_t;

/* ȫ�ֱ��� */
extern vision_t vision;
extern vision_tx_t vision_tx;
extern vision_mode_e vision_mode;
extern vision_fruit_msg_t vision_fruit;

/* ���� */
void vision_get_data_sp(uint8_t *data, uint32_t len);
void vision_data_handler(uint8_t *pVision_Data,uint32_t len);   // ���ڻص�
void vsn_gimbal_ref_calc(void);            // 1 ms ����
void vision_output_data(void);
void vision_output_data_sp(void);
void vision_fruit_getData(uint8_t *buff);

#endif 
