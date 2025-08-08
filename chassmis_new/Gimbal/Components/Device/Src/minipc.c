/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : minipc.c
  * @brief          : minipc interfaces functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : to be tested
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "minipc.h"
#include "usbd_cdc_if.h"
#include "crc.h"
#include "Control_Task.h"

/* Private variables ---------------------------------------------------------*/
/**
 * @brief Buffer of MiniPC data to be sent
 */
//uint8_t MiniPC_SendBuf[MINIPC_SENDLENGTH];

/**
 * @brief structure that contains the information for the MiniPC Receive Data.
 */
MiniPC_ReceivePacket_Typedef MiniPC_ReceivePacket = {
  .header = 0xA5,
};
/**
 * @brief structure that contains the information for the MiniPC Transmit Data.
 */
MiniPC_SendPacket_Typedef MiniPC_SendPacket = {
    .header = 0x5A,
	  .detect_color = 0,
};

/**
  * @brief  Send the MiniPC frame Information according the USB CDC
  * @param  SendPacket: pointer to MiniPC_SendPacket_Typedef structure that 
  *         contains the information for the MiniPC Transmit Data.
  * @retval none
  */
void MiniPC_SendFrameInfo(MiniPC_SendPacket_Typedef *SendPacket)
{
  /* calculate the crc16 */
  SendPacket->checksum = get_CRC16_check_sum((uint8_t *)SendPacket,MINIPC_SENDLENGTH-2,0xffff);

  /* store the MiniPC data to be sent */
//  memcpy(MiniPC_SendBuf,(uint8_t *)SendPacket,MINIPC_SENDLENGTH);

  /* USB Send data */
//  CDC_Transmit_FS(MiniPC_SendBuf,MINIPC_SENDLENGTH);
}
//------------------------------------------------------------------------------

/**
  * @brief  Receive the MiniPC frame Information according the USB CDC
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval none
  */
static float bit8TOfloat32(uint8_t change_info[4])
{
	union
	{
    float float32;
		uint8_t  byte[4];
	}u32val;

  u32val.byte[0] = change_info[0];
  u32val.byte[1] = change_info[1];
  u32val.byte[2] = change_info[2];
  u32val.byte[3] = change_info[3];

	return u32val.float32;
}

void MiniPC_RecvFrameInfo(uint8_t* Buf, const uint32_t *Len)
{

	Control_Info.Shoot.Vision_Fire_Flag = Buf[1];
	
	Control_Info.Gimbal.Vision.Pitch_int[0] = Buf[2];
	Control_Info.Gimbal.Vision.Pitch_int[1] = Buf[3];
	Control_Info.Gimbal.Vision.Pitch_int[2] = Buf[4];
	Control_Info.Gimbal.Vision.Pitch_int[3] = Buf[5];

	Control_Info.Gimbal.Vision.Yaw_int[0] = Buf[6];
	Control_Info.Gimbal.Vision.Yaw_int[1] = Buf[7];
	Control_Info.Gimbal.Vision.Yaw_int[2] = Buf[8];
	Control_Info.Gimbal.Vision.Yaw_int[3] = Buf[9];
	
	Control_Info.Gimbal.Vision.Distance_int[0] = Buf[10];
	Control_Info.Gimbal.Vision.Distance_int[1] = Buf[11];
	Control_Info.Gimbal.Vision.Distance_int[2] = Buf[12];
	Control_Info.Gimbal.Vision.Distance_int[3] = Buf[13];
	
	Control_Info.Gimbal.Vision.Distance = bit8TOfloat32(Control_Info.Gimbal.Vision.Distance_int);
	Control_Info.Gimbal.Vision.Pitch_Angle = bit8TOfloat32(Control_Info.Gimbal.Vision.Pitch_int);
	Control_Info.Gimbal.Vision.Yaw_Angle = bit8TOfloat32(Control_Info.Gimbal.Vision.Yaw_int);
	
 Control_Info.Gimbal.Vision.Flag = Buf[14];
}

//------------------------------------------------------------------------------

