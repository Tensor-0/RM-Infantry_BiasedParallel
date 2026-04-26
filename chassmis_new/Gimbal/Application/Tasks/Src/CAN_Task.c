/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : CAN_Task.c
  * @brief          : CAN task
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "CAN_Task.h"
#include "Control_Task.h"
#include "Vision_Task.h"
#include "Motor.h"
#include "bsp_can.h"
#include "remote_control.h"
#include "INS_Task.h"

bool Enable_Flag = 0;
uint8_t Motor_Enable_Cnt = 0;


static void Check_Motor_Online();
static void Check_Motor_Enable();


 TickType_t systick = 0;
/* USER CODE BEGIN Header_CAN_Task */
/**
* @brief Function implementing the StartCANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Task */
void CAN_Task(void const * argument)
{
  /* USER CODE BEGIN CAN_Task */
	 osDelay(600);

  /* Infinite loop */
  for(;;)
  {
	 systick = osKernelSysTick();
		

   Shoot_TxFrame.Data[0] = (uint8_t)(Control_Info.SendValue[0]>>8);
   Shoot_TxFrame.Data[1] = (uint8_t)(Control_Info.SendValue[0]);
	 Shoot_TxFrame.Data[2] = (uint8_t)(Control_Info.SendValue[1]>>8);
   Shoot_TxFrame.Data[3] = (uint8_t)(Control_Info.SendValue[1]);
	 Shoot_TxFrame.Data[4] = (uint8_t)(Control_Info.SendValue[2]>>8);
   Shoot_TxFrame.Data[5] = (uint8_t)(Control_Info.SendValue[2]);
   USER_CAN_TxMessage(&Shoot_TxFrame);

  Send2Chassis_TxFrame.Data[0] =  (uint8_t) ((Control_Info.Shoot.Mode)<<5 | (remote_ctrl.rc_lost)<<4 | (remote_ctrl.rc.s[0])<<2 | (remote_ctrl.rc.s[1]));
	Send2Chassis_TxFrame.Data[1] =  (uint8_t) (remote_ctrl.rc.ch[3]>> 8);
	Send2Chassis_TxFrame.Data[2] =  (uint8_t) (remote_ctrl.rc.ch[3] );
  Send2Chassis_TxFrame.Data[3] =  (uint8_t) ((uint8_t)Vision_Info.IF_Aiming_Enable);
	Send2Chassis_TxFrame.Data[4] = (uint8_t) (remote_ctrl.mouse.press_r);
	Send2Chassis_TxFrame.Data[5] = remote_ctrl.rc.ch[4]; 
	Send2Chassis_TxFrame.Data[6] = (uint8_t)(remote_ctrl.key.v >> 8);
	Send2Chassis_TxFrame.Data[7] = (uint8_t)(remote_ctrl.key.v);
  USER_CAN_TxMessage(&Send2Chassis_TxFrame);
  
		
	
	Check_Motor_Online();
		
	if(Enable_Flag == 1){
	
		  Damiao_Motor_CAN_Send(&Pitch_TxFrame,0,0,0,0, Control_Info.Gimbal.Output.Pitch);
			Damiao_Motor_CAN_Send(&Yaw_TxFrame,0,0,0,0, Control_Info.Gimbal.Output.Yaw);
	
	}else{
	
	   	Check_Motor_Enable();

	}
 	 
	


	
		
 
    osDelay(1);
  }
  /* USER CODE END CAN_Task */
}

static void Check_Motor_Online(){

	

	
	  if(Damiao_Yaw_Motor.Online_cnt>200){
	     Damiao_Yaw_Motor.Online_cnt--;
		   Damiao_Yaw_Motor.lost = 0;

			}else{
		   Damiao_Yaw_Motor.Online_cnt = 0;
		   Damiao_Yaw_Motor.lost = 1;
		  }
        
    if(Damiao_Pitch_Motor.Online_cnt>200){
	     Damiao_Pitch_Motor.Online_cnt--;
		   Damiao_Pitch_Motor.lost = 0;

			}else{
		   Damiao_Pitch_Motor.Online_cnt = 0;
		   Damiao_Pitch_Motor.lost = 1;
		  }
	if(Damiao_Yaw_Motor.lost + Damiao_Pitch_Motor.lost==0 ) Enable_Flag = 1;
	else Enable_Flag= 0;
	
	
	
}
static void Check_Motor_Enable(){

		if( Damiao_Yaw_Motor.lost == 1){
			   Motor_Enable_Cnt = 30;
			   Damiao_Motor_Enable(&Yaw_TxFrame);
			   while(Motor_Enable_Cnt > 0) Motor_Enable_Cnt--;
			 }
 
    if( Damiao_Pitch_Motor.lost == 1){
			   Motor_Enable_Cnt = 30;
			   Damiao_Motor_Enable(&Pitch_TxFrame);
			   while(Motor_Enable_Cnt > 0) Motor_Enable_Cnt--;
		}
		    
		
}

