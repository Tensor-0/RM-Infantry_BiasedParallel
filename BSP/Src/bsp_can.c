/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_can.c
  * @brief          : bsp can functions 
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to enable the fdcan filter
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"
#include "bsp_can.h"
#include "Motor.h"
#include "Remote_Control.h"
#include "Image_Transmission.h"

/**
 * @brief The structure that contains the Information of FDCAN1 and FDCAN2 Receive.
 */
FDCAN_RxFrame_TypeDef FDCAN_RxFIFO0Frame;
FDCAN_RxFrame_TypeDef FDCAN_RxFIFO1Frame;

/**
 * @brief The structure that contains the Information of FDCAN1 Transmit(CLASSIC_CAN).
 */
FDCAN_TxFrame_TypeDef FDCAN1_TxFrame = {
	.hcan = &hfdcan1,
  .Header.IdType = FDCAN_STANDARD_ID, 
  .Header.TxFrameType = FDCAN_DATA_FRAME,
  .Header.DataLength = 8,
	.Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
  .Header.BitRateSwitch = FDCAN_BRS_OFF,
  .Header.FDFormat =  FDCAN_CLASSIC_CAN,           
  .Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,  
  .Header.MessageMarker = 0,
};

/**
 * @brief The structure that contains the Information of FDCAN2 Transmit(FDCAN).
 */
FDCAN_TxFrame_TypeDef FDCAN2_TxFrame = {
  .hcan = &hfdcan2,
  .Header.IdType = FDCAN_STANDARD_ID, 
  .Header.TxFrameType = FDCAN_DATA_FRAME,
  .Header.DataLength = 8,
	.Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
  .Header.BitRateSwitch = FDCAN_BRS_ON,
  .Header.FDFormat =  FDCAN_FD_CAN,           
  .Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,  
  .Header.MessageMarker = 0,
};

/**
 * @brief The structure that contains the Information of FDCAN3 Transmit(CLASSIC_CAN).
 */
FDCAN_TxFrame_TypeDef FDCAN3_TxFrame = {
  .hcan = &hfdcan3,
  .Header.IdType = FDCAN_STANDARD_ID, 
  .Header.TxFrameType = FDCAN_DATA_FRAME,
  .Header.DataLength = 8,
	.Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
  .Header.BitRateSwitch = FDCAN_BRS_OFF,
  .Header.FDFormat =  FDCAN_CLASSIC_CAN,           
  .Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,
	.Header.MessageMarker = 0,
};

/**
  * @brief  Configures the FDCAN Filter. 
            FDCAN1:CLASSIC_CAN  FDCAN2:FDCAN  FDCAN3:CLASSIC_CAN
  * @param  None
  * @retval None
  */
void BSP_FDCAN_Init(void){

  FDCAN_FilterTypeDef FDCAN1_FilterConfig;
	
	FDCAN1_FilterConfig.IdType = FDCAN_STANDARD_ID; // 閿熸枻鎷烽敓鏂ゆ嫹ID閿熸枻鎷烽敓鏂ゆ嫹閫夐敓鏂ゆ嫹 閿熸枻鎷峰噯ID
  FDCAN1_FilterConfig.FilterIndex = 0;           //閿熸枻鎷峰墠FDCAN閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹鐗涢敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷锋灇閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鍓胯鎷峰悓閿熸枻鎷稩D 閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹0閿熸枻鎷�1閿熸枻鎷�2....
  FDCAN1_FilterConfig.FilterType = FDCAN_FILTER_MASK; //閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷稭ask妯″紡 閿熸埅鐚存嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷稩D1閿熸枻鎷稩D2閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷�
  FDCAN1_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;//閫夐敓鏂ゆ嫹閿熶茎闈╂嫹FIFO閿熸枻鎷烽敓鏂ゆ嫹閿熺Ц锝忔嫹閿熸枻鎷烽敓鏂ゆ嫹CubeMX閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷稦IFO1閿熼叺鏀圭鎷稦DCAN_FILTER_TO_RXFIFO1
  FDCAN1_FilterConfig.FilterID1 = 0x00000000; // 閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷锋牎閿熻鐏扮嫪D2閿熸枻鎷烽敓鏂ゆ嫹0x00000000閿熼叺璇ф嫹閿熸枻鎷烽敓鏂ゆ嫹璇撮敓鏂ゆ嫹榄忛敓绲€D
  FDCAN1_FilterConfig.FilterID2 = 0x00000000; //閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹
  
  HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN1_FilterConfig); //閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熺煫纰夋嫹CAN1
		
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE); //閿熸枻鎷烽敓鏂ゆ嫹CAN1閿熸枻鎷峰叏閿熻鐧告嫹閿熷壙锝忔嫹閿熸枻鎷烽敓瑙掑尅鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷�
 
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);//閿熸枻鎷稦IFO0閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸嵎鏂ゆ嫹閿熸枻鎷烽敓鍙鎷�
  
  HAL_FDCAN_Start(&hfdcan1);//浣块敓鏂ゆ嫹CAN1
 	
	
	FDCAN_FilterTypeDef FDCAN2_FilterConfig;//FDCAN2鏉╁洦鎶ら崳銊х波閺嬪嫪缍�
	
  FDCAN2_FilterConfig.IdType = FDCAN_STANDARD_ID;//鏉╁洦鎶ら崳銊ㄧ箖濠婎棷D缁鐎�
  FDCAN2_FilterConfig.FilterIndex = 0;//鏉╁洦鎶ら崳銊х椽閸欙拷0
  FDCAN2_FilterConfig.FilterType = FDCAN_FILTER_MASK;//鏉╁洦鎶ら崳銊ц閸ㄥ绱扮紒蹇撳悁娴ｅ秴鐫嗛拕鍊熺箖濠娿倕娅�
  FDCAN2_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;//鏉╁洦鎶ら崥搴ょ箻閸忣檶IFO1閸栵拷
  FDCAN2_FilterConfig.FilterID1 = 0x00000000; //濞戝牊浼匢D鏉╁洦鎶ら崳锟�
  FDCAN2_FilterConfig.FilterID2 = 0x00000000; //鏉╁洦鎶ら崳銊ョ潌閽勶拷 濮ｅ繋閲滄担宥呮綆鐠佸墽鐤�0閿涘苯宓嗘稉宥堢箖濠娿倓鎹㈡担鏃綝
  
	HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN2_FilterConfig);//閺嶈宓佹潻鍥ㄦ姢閸ｃ劎绮ㄩ弸鍕秼娑擃厽瀵氱€规氨娈戦崣鍌涙殶闁板秶鐤咶DCAN閹恒儲鏁规潻鍥ㄦ姢閸ｏ拷

  //闁板秶鐤� FDCAN 閸忋劌鐪潻鍥ㄦ姢閸ｏ拷
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);//娴ｈ儻鍏樻稉顓熸焽閿涘瓗IFO1閺傜増绉烽幁顖欒厬閺傦拷

  HAL_FDCAN_EnableTxDelayCompensation(&hfdcan2);//娴ｈ儻鍏楩DCAN閸欐垿鈧礁娆㈤弮鎯八夐崑锟�
 
  HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan2,14,14);//FDCAN閸欐垿鈧礁娆㈤弮鎯八夐崑鎸庢闂傜顔曠純锟�

  HAL_FDCAN_Start(&hfdcan2);//FDCAN瀵偓婵浼愭担锟�
	
	
	FDCAN_FilterTypeDef FDCAN3_FilterConfig;
	
	FDCAN3_FilterConfig.IdType = FDCAN_STANDARD_ID;
  FDCAN3_FilterConfig.FilterIndex = 0;
  FDCAN3_FilterConfig.FilterType = FDCAN_FILTER_MASK;
  FDCAN3_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  FDCAN3_FilterConfig.FilterID1 = 0x00000000; 
  FDCAN3_FilterConfig.FilterID2 = 0x00000000; 
  
	HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN3_FilterConfig);

  HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

  HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  HAL_FDCAN_Start(&hfdcan3);
}

/**
  * @brief  Function to transmit the FDCAN message.
  * @param  *FDCAN_TxFrame :the structure that contains the Information of FDCAN
  * @retval None
  */
void USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame){

    HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame->hcan,&FDCAN_TxFrame->Header,FDCAN_TxFrame->Data);
 
}

/**
  * @brief  Function to converting the FDCAN1 received message to Fifo0.
	* @param  Identifier: Received the identifier.
	* @param  Data: Array that contains the received massage.
  * @retval None
  */
static void FDCAN1_RxFifo0RxHandler(uint32_t *Identifier,uint8_t Data[8])
{
   
//  if(*Identifier==0x302)
//	 {
//		remote_ctrl.rc_lost= ((Data[0] &0x80)>>7);
//	  remote_ctrl.rc.s[0]	= ((Data[0] &0x60) >> 5); 
//	  remote_ctrl.rc.s[1] = (Data[0] &0x18)>>3;
//		//Control_Info.Tigger.Vision_IF_Fire = ((Data[0] &0x04)>>2);
//		remote_ctrl.mouse.press_l = ((Data[0] &0x02)>>1);
//		remote_ctrl.mouse.press_r = ((Data[0] &0x01));
//	  remote_ctrl.rc.ch[3] = ((int16_t) Data[1] << 8  | Data[2]) ;
//		remote_ctrl.key.v = ((int16_t) Data[3] << 8  | Data[4]) ;
//	 	remote_ctrl.rc.ch[4] = ((int16_t) Data[6] << 8  | Data[7]) ;

//	 }else if(*Identifier==0x11){
//	 
//	 
//	  DM_Motor_Info_Update(Identifier,Data,&DM_Yaw_Motor);
//	 
//	 }


}

/**
  * @brief  Function to converting the FDCAN3 received message to Fifo0.
	* @param  Identifier: Received the identifier.
	* @param  Data: Array that contains the received massage.
  * @retval None
  */
static void FDCAN3_RxFifo0RxHandler(uint32_t *Identifier,uint8_t Data[8])
{

	DJI_Motor_Info_Update(Identifier,Data,&Chassis_Motor[0]);
  DJI_Motor_Info_Update(Identifier,Data,&Chassis_Motor[1]);


}


/**
  * @brief  Function to converting the FDCAN2 received message to Fifo1.
	* @param  Identifier: Received the identifier.
	* @param  Data: Array that contains the received massage.
  * @retval None
  */
static void FDCAN2_RxFifo1RxHandler(uint32_t *Identifier,uint8_t Data[8])
{
	
	DM_Motor_Info_Update(Identifier,Data,&DM_8009_Motor[0]);
  DM_Motor_Info_Update(Identifier,Data,&DM_8009_Motor[1]);
	DM_Motor_Info_Update(Identifier,Data,&DM_8009_Motor[2]);
	DM_Motor_Info_Update(Identifier,Data,&DM_8009_Motor[3]);
	

}

/**
  * @brief  Rx FIFO 0 callback.
  * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs indicates which Rx FIFO 0 interrupts are signaled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{ 
  
	HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN_RxFIFO0Frame.Header, FDCAN_RxFIFO0Frame.Data);
	
  if(hfdcan == &hfdcan1){	
	
   FDCAN1_RxFifo0RxHandler(&FDCAN_RxFIFO0Frame.Header.Identifier,FDCAN_RxFIFO0Frame.Data);
	 
	}

  if(hfdcan == &hfdcan3){
	
	 FDCAN3_RxFifo0RxHandler(&FDCAN_RxFIFO0Frame.Header.Identifier,FDCAN_RxFIFO0Frame.Data);
	
	}
	
}
	
/**
  * @brief  Rx FIFO 1 callback.
  * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo1ITs indicates which Rx FIFO 1 interrupts are signaled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo1_Interrupts.
  * @retval None
  */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{ 
  
	HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &FDCAN_RxFIFO1Frame.Header, FDCAN_RxFIFO1Frame.Data);
	
  FDCAN2_RxFifo1RxHandler(&FDCAN_RxFIFO1Frame.Header.Identifier,FDCAN_RxFIFO1Frame.Data);
	 
}