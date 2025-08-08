/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Vision_Task.c
  * @brief          : Vision task
  * @author         : Yan Yuanbin
  * @date           : 2023/07/23
  * @version        : v2.1
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "Vision_Task.h"
#include "INS_Task.h"
#include "api_trajectory.h"
#include "usbd_cdc_if.h"

/* Private variables -----------------------------------------------------------*/
/**
 * @brief structure that contains the information for the solved trajectory.
 */
SolveTrajectory_Typedef SolveTrajectory={
  .Camera_Yaw_Vertical = -0.105f,//-0.1   0.025f
  .Camera_Yaw_Horizontal = 0.2f,//-0.1   0.1f
  .Time_Offset = 0.08f,
	.Armor_Yaw_Limit = 0.28f,
	.Armor_Yaw_Limit_Offset = 0.f,
	.bullet_speed =28.f,//60
	.Bias_Yaw= -1.f,
	.Bias_Pitch=-0.f, //+ = -
};

/**
 * @brief structure that contains the information for the Vision.
 */
Vision_Info_Typedef Vision_Info;
uint32_t Vision_Task_Cnt;
uint8_t MiniPC_SendBuf[16];

uint8_t *Yaw, *Pitch , *Roll;



/* USER CODE BEGIN Header_Vision_Task */
/**
* @brief Function implementing the StartVisionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Vision_Task */
void Vision_Task(void const * argument)
{
  /* USER CODE BEGIN Vision_Task */
  TickType_t systick = 0;

  /* Infinite loop */
  for(;;)
  {
    systick = osKernelSysTick();

//    /* received minipc tracking , Enable the vision aiming */
//    Vision_Info.IF_Aiming_Enable = (MiniPC_ReceivePacket.tracking == true);

//    /* update the transmit euler angle in radians */
//    MiniPC_SendPacket.pitch = -INS_Info.angle[2]+SolveTrajectory.Bias_Pitch*DegreesToRadians  ;
//    MiniPC_SendPacket.yaw   = INS_Info.angle[0]+SolveTrajectory.Bias_Yaw *DegreesToRadians ;
//    MiniPC_SendPacket.roll  = INS_Info.angle[1];
//   MiniPC_SendPacket.detect_color=0;
//    /* update the solve trajectory */
//		SolveTrajectory_Update(&SolveTrajectory,-MiniPC_SendPacket.pitch,MiniPC_SendPacket.yaw,SolveTrajectory.bullet_speed);

//    /* transform the solved trajetory */
//    SolveTrajectory_Transform(&MiniPC_SendPacket,&MiniPC_ReceivePacket,&SolveTrajectory);

//		/* update the gimbal target pose */
//    Vision_Info.target_Pitch = SolveTrajectory.armorlock_pitch * RadiansToDegrees ;
//    Vision_Info.target_Yaw = SolveTrajectory.armorlock_yaw * RadiansToDegrees ;
//		
//		/* update the fire control flag */
//		Vision_Info.IF_Fire_Accept = (SolveTrajectory.control_status == 2 && Vision_Info.IF_Aiming_Enable == true);

//    /* transmit the minipc frame data */
    MiniPC_SendFrameInfo(&MiniPC_SendPacket);

//		Vision_Task_Cnt++;

    Yaw = (uint8_t *)&INS_Info.yaw_angle;
    Pitch = (uint8_t *)&INS_Info.rol_angle;
    Roll = (uint8_t *)&INS_Info.pit_angle;



     MiniPC_SendBuf[0] = 0xFF;
		 MiniPC_SendBuf[1] = 0X00;
     MiniPC_SendBuf[2] = *Pitch;
		 MiniPC_SendBuf[3] = *(Pitch + 1);
     MiniPC_SendBuf[4] = *(Pitch + 2);
		 MiniPC_SendBuf[5] = *(Pitch + 3);
		 MiniPC_SendBuf[6] = *Roll;
		 MiniPC_SendBuf[7] = *(Roll + 1);	 
		 MiniPC_SendBuf[8] = *(Roll + 2);
		 MiniPC_SendBuf[9] = *(Roll + 3);
		 MiniPC_SendBuf[10] = *Yaw;
		 MiniPC_SendBuf[11] = *(Yaw + 1);	 
		 MiniPC_SendBuf[12] = *(Yaw + 2);
		 MiniPC_SendBuf[13] = *(Yaw + 3);
		 MiniPC_SendBuf[14] = 0x00;
		 MiniPC_SendBuf[15] = 0x0d;
     CDC_Transmit_FS(MiniPC_SendBuf,16);



    osDelayUntil(&systick,2);
  }
  /* USER CODE END Vision_Task */
}
//------------------------------------------------------------------------------

