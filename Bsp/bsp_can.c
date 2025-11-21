
/**
  ******************************************************************************
  * @file    bsp_can.c
  * @author  ZZY 1950828677@qq.com
  * @brief   Code for FDCAN driver in STM32 series MCU
  * @date    2025-10
  * @version 1.0
  * @
  ==============================================================================
*/

/* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
FDCAN_TxHeaderTypeDef tx_message;
FDCAN_RxHeaderTypeDef rx_fifo0_message, rx_fifo1_message;

static void (*pCAN1_RxCpltCallback)(CAN_RxBuffer_t*);
static void (*pCAN2_RxCpltCallback)(CAN_RxBuffer_t*);
static void (*pCAN3_RxCpltCallback)(CAN_RxBuffer_t*);

bsp_user_can_t bsp_user_can1;
bsp_user_can_t bsp_user_can2;
bsp_user_can_t bsp_user_can3;

bsp_can_filt_tbl_t bsp_can1_filt_tbl;
bsp_can_filt_tbl_t bsp_can2_filt_tbl;
bsp_can_filt_tbl_t bsp_can3_filt_tbl;

//实例化结构体 初始化部分BSP_CAN成员
bsp_user_can_t bsp_user_can1 = 
{
    .hfdcan  = &hfdcan1,
    .CAN_TxBuffer = {0},
    .CAN_RxBuffer = {0}    
};

bsp_user_can_t bsp_user_can2 = 
{
    .hfdcan  = &hfdcan2,
    .CAN_TxBuffer = {0},
    .CAN_RxBuffer = {0}    
};

bsp_user_can_t bsp_user_can3 = 
{
    .hfdcan  = &hfdcan3,
    .CAN_TxBuffer = {0},
    .CAN_RxBuffer = {0}    
};
/* Private type --------------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
//static void bsp_user_can_rx_Callback(bsp_user_can_t  *p);
static void bsp_can_filter_init( bsp_user_can_t *canx);
static void bsp_can_tx_init(bsp_user_can_t *canx);
// static void Error_Handler(void);

/* function prototypes -------------------------------------------------------*/
/**
* @brief  Initialize uart device
* @param  bsp_user_can_t BSP层封装的CAN端口
* @note   FIFO的分配遵循平均的原则（平衡负担），由该函数自行决定放在哪个FIFO
* @note   过滤器Index也由该函数根据用户填写表格的数值来自增
*/
static void bsp_can_filter_init( bsp_user_can_t *canx)
{
    uint8_t              user_rules_num = 0;
    const uint16_t       *p = NULL;

    /*取过滤器初始化规则表中的内容*/
    if(canx->hfdcan->Instance==FDCAN1) 
    {
     user_rules_num = bsp_can1_filt_tbl.count ;                    //这个数由用户填写，用户要填多少条规则就填多少，用于后续的for循环 （关联can_filter.FilterIndex）
     p              = bsp_can1_filt_tbl.data;                              //取柔性数组首元素的地址
    }
        
    if(canx->hfdcan->Instance==FDCAN2) 
    {
     user_rules_num = bsp_can2_filt_tbl.count ;                       
     p              = bsp_can2_filt_tbl.data;                              
    }
    
    if(canx->hfdcan->Instance==FDCAN3) 
    {
     user_rules_num = bsp_can3_filt_tbl.count ;                          
     p              = bsp_can3_filt_tbl.data;                              
    }


    /*CAN过滤器的初始化*/
	 FDCAN_FilterTypeDef can_filter;  

    //一些基本不会变的设置
	 can_filter.IdType = FDCAN_STANDARD_ID;									//标准帧
	
    //过滤规则取用装填
   for(uint8_t i=0;i<user_rules_num;i++)                                    //有多少条规则初始化过滤器多少次
   {
        can_filter.FilterIndex = i;										    //规则索引
            
        switch (p[0])                                                       //数组固定格式：  type；high_id；low_id  第一个就是type
        {
        case FLT_RANGE:{
            can_filter.FilterType = FDCAN_FILTER_RANGE;
        }break;
        
        case FLT_DUAL:{
            can_filter.FilterType = FDCAN_FILTER_DUAL;
        }break;

        case FLT_MASK:{
            can_filter.FilterType = FDCAN_FILTER_MASK;
        }break;
        default:break;
        }

        uint16_t hight_id  = p[1]& 0x7FFU ;                                 //装填用户填写在tbl上的id   与上0x7FFU是因为过滤器需要的是11位数据
        uint16_t low_id    = p[2]& 0x7FFU ;                                       
        can_filter.FilterID1 = 		hight_id;					
        can_filter.FilterID2 = 		low_id;					
				
				
        if(i%2 == 0)                                                        //平均分配FIFO，缓解FIFO数据压力
        {
           can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;				//通过过滤器后给邮箱0
        }
        else if(i%2 == 1)
        {
           can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;				//通过过滤器后给邮箱1
        }

        	HAL_FDCAN_ConfigFilter(canx->hfdcan,&can_filter);				//传参		
   }

   	HAL_FDCAN_ConfigGlobalFilter(canx->hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ActivateNotification(canx->hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);//使能邮箱0新消息中断
    HAL_FDCAN_ActivateNotification(canx->hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);//使能邮箱1新消息中断

}

/**
* @brief  初始化CAN接收
* @param  bsp_user_can_t BSP层封装的CAN端口
* @retval None
*/
static void bsp_can_tx_init(bsp_user_can_t *canx)
{
    tx_message.IdType = FDCAN_STANDARD_ID;
    tx_message.TxFrameType = FDCAN_DATA_FRAME;
    tx_message.DataLength = FDCAN_DLC_BYTES_8;
    tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_message.BitRateSwitch = FDCAN_BRS_OFF;
    tx_message.FDFormat = FDCAN_CLASSIC_CAN;
    tx_message.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
    tx_message.MessageMarker = 0;
}


/**
* @brief  BSP开启CAN 
* @param  bsp_user_can_t BSP层封装的CAN端口
* @param  pFunc          指向中间层的用户数据处理函数
* @retval None
* @note   每个CAN都要调用一次（也就是H7用3个CAN，就调用3次）
*/
void bsp_can_init(bsp_user_can_t *canx, void (*pFunc)(CAN_RxBuffer_t*))
{
    bsp_can_filter_init(canx);
    bsp_can_tx_init(canx);

    if (canx->hfdcan->Instance==FDCAN1)		pCAN1_RxCpltCallback = pFunc;
    if (canx->hfdcan->Instance==FDCAN2)		pCAN2_RxCpltCallback = pFunc;
    if (canx->hfdcan->Instance==FDCAN3)		pCAN3_RxCpltCallback = pFunc;

   
    HAL_FDCAN_Start(canx->hfdcan);	

}







/** 
 * @brief     can发送标准数据统一接口，提供给其它文件调用，8字节数据长度
 * @param[in] can_periph: can通道
 * @param[in] id        : 帧id
 * @param[in] data      : 数据指针
 * @retval    HAL_StatusTypeDef
 */
HAL_StatusTypeDef can_std_transmit(can_channel_e can_periph, uint32_t id, uint8_t *data)
{
    tx_message.Identifier = id;
    if (can_periph == CAN_CHANNEL_1)
        return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_message, data);
    else if (can_periph == CAN_CHANNEL_2)
        return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx_message, data);
    else if (can_periph == CAN_CHANNEL_3)
		return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &tx_message, data);
	return HAL_ERROR;
}



/**
* @brief  FIFO0中断
* @note   由hal库中断自动调用
*/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{

            if(hfdcan->Instance == FDCAN1)
            {
                if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &bsp_user_can1.CAN_RxBuffer.rxHeader, bsp_user_can1.CAN_RxBuffer.rx_data) == HAL_OK)
                pCAN1_RxCpltCallback(&bsp_user_can1.CAN_RxBuffer);
            }
            else if(hfdcan->Instance == FDCAN2)
            {		
                if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &bsp_user_can2.CAN_RxBuffer.rxHeader, bsp_user_can2.CAN_RxBuffer.rx_data) == HAL_OK)
                pCAN2_RxCpltCallback(&bsp_user_can2.CAN_RxBuffer);
            }
            else if(hfdcan->Instance == FDCAN3)
            {
                if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &bsp_user_can3.CAN_RxBuffer.rxHeader, bsp_user_can3.CAN_RxBuffer.rx_data) == HAL_OK)
                pCAN3_RxCpltCallback(&bsp_user_can3.CAN_RxBuffer);
            }


        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    }
}



/**
* @brief  FIFO1中断
* @note   由hal库中断自动调用
*/
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
	{

            if(hfdcan->Instance == FDCAN1)
            {
                if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &bsp_user_can1.CAN_RxBuffer.rxHeader, bsp_user_can1.CAN_RxBuffer.rx_data) == HAL_OK)
                pCAN1_RxCpltCallback(&bsp_user_can1.CAN_RxBuffer);
            }
            else if(hfdcan->Instance == FDCAN2)
            {		
                if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &bsp_user_can2.CAN_RxBuffer.rxHeader, bsp_user_can2.CAN_RxBuffer.rx_data) == HAL_OK)
                pCAN2_RxCpltCallback(&bsp_user_can2.CAN_RxBuffer);
            }
            else if(hfdcan->Instance == FDCAN3)
            {
                if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &bsp_user_can3.CAN_RxBuffer.rxHeader, bsp_user_can3.CAN_RxBuffer.rx_data) == HAL_OK)
                pCAN3_RxCpltCallback(&bsp_user_can3.CAN_RxBuffer);
            }

	

        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
    }


}



