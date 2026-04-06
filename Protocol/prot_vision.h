/**
  * @file  prot_vision.h
  * @brief �Ӿ����ڽ��� + ��̨�ο��������С�棩
  */

#ifndef __PROT_VISION_H__
#define __PROT_VISION_H__
#include <stdint.h>
#include <math.h>

#define VISION_DATA_LEN  28

/* �Ӿ���ģʽ */
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
    float target_yaw_angle, target_pit_angle;
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
            float dis;
						float dis2;
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

/* ȫ�ֱ��� */
extern vision_t vision;
extern vision_tx_t vision_tx;
extern vision_mode_e vision_mode;
/* ���� */
void vision_data_handler(uint8_t *pVision_Data);   // ���ڻص�
void vsn_gimbal_ref_calc(void);            // 1 ms ����
void vision_output_data(void);

#endif 
