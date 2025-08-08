#include "CAN_Task.h"
#include "bsp_can.h"
#include "cmsis_os.h"
#include "Motor.h"
#include "pid.h"
#include "usart.h"
#include "remote_control.h"
#include "math.h"
#include "arm_math.h"
#include "Control_Task.h"
#include "INS_Task.h"
#include "bsp_uart.h"
#include "Referee_System.h"

uint8_t DM_Motor_Enable_Cnt = 0;
uint8_t GIM_Motor_Enable_Cnt = 0;


static void Check_Motor_Online(void);
static void Check_Motor_Enable(void);
static void Referee_System_Info_Send_Gimbal(void);


 void CAN_Task(void const * argument)
{

 
  TickType_t systick = 0;
	
	
	  	MIT_Motor_Command(&FDCAN3_TxFrame,0x01,Motor_Enable);
	       osDelay(30);
	 	  MIT_Motor_Command(&FDCAN3_TxFrame,0x02,Motor_Enable);
	       osDelay(30);
	  	MIT_Motor_Command(&FDCAN2_TxFrame,0x01,Motor_Enable);
	       osDelay(30);
	 	  MIT_Motor_Command(&FDCAN2_TxFrame,0x02,Motor_Enable);
	       osDelay(30);
			MIT_Motor_Command(&FDCAN2_TxFrame,0x03,Motor_Enable);
         osDelay(30);
    	MIT_Motor_Command(&FDCAN2_TxFrame,0x04,Motor_Enable);
        osDelay(30);
	
//		MIT_Motor_Command(&FDCAN2_TxFrame,0x01,Motor_Save_Zero_Position);
//      	      osDelay(30);
//		MIT_Motor_Command(&FDCAN2_TxFrame,0x02,Motor_Save_Zero_Position);
//      	      osDelay(30);
//		MIT_Motor_Command(&FDCAN2_TxFrame,0x03,Motor_Save_Zero_Position);
//      	      osDelay(30);
//    MIT_Motor_Command(&FDCAN2_TxFrame,0x04,Motor_Save_Zero_Position);
//      	      osDelay(30);

	
	for(;;)
  {
		
  	Referee_System_Info_Send_Gimbal();
		

if(Control_Info.Init.Begin_Init == 1){
		
		   if(Control_Info.Init.Joint_Init.IF_Joint_Init == 0){
			
			  GIM_Motor_Motor_CAN_TxMessage(&FDCAN3_TxFrame,&GIM_8108_Motor[0],0,0,0,0,0);
	      GIM_Motor_Motor_CAN_TxMessage(&FDCAN3_TxFrame,&GIM_8108_Motor[1],0,0,0,0,0);
			  DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[0],0,0,10.f,2.f,0);		
        DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[1],0,0,10.f,2.f,0);
        DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[2],0,0,10.f,2.f,0);
		    DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[3],0,0,10.f,2.f,0);
   
			}else  if(Control_Info.Init.Joint_Init.IF_Joint_Init == 1){
				
				if(Control_Info.Init.Velocity_Init.IF_Velocity_Init == 0){
					
							GIM_Motor_Motor_CAN_TxMessage(&FDCAN3_TxFrame,&GIM_8108_Motor[0],0,0,0,1.f,0);
	            GIM_Motor_Motor_CAN_TxMessage(&FDCAN3_TxFrame,&GIM_8108_Motor[1],0,0,0,1.f,0);
							DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[0],0,0,10.f,2.f,0);		
							DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[1],0,0,10.f,2.f,0);
							DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[2],0,0,10.f,2.f,0);
							DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[3],0,0,10.f,2.f,0);
				
				}else if(Control_Info.Init.Velocity_Init.IF_Velocity_Init == 1){
			
				 if(Control_Info.Init.Yaw_Init.IF_Yaw_Init == 0){
				 
				    	GIM_Motor_Motor_CAN_TxMessage(&FDCAN3_TxFrame,&GIM_8108_Motor[0],0,0,0,0, -Control_Info.L_Leg_Info.Moment.T);
	            GIM_Motor_Motor_CAN_TxMessage(&FDCAN3_TxFrame,&GIM_8108_Motor[1],0,0,0,0,  Control_Info.R_Leg_Info.Moment.T);
							DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[0],0,0,10.f,2.f,0);		
							DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[1],0,0,10.f,2.f,0);
							DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[2],0,0,10.f,2.f,0);
							DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[3],0,0,10.f,2.f,0);
				 
				 
				 
				 }else if(Control_Info.Init.Yaw_Init.IF_Yaw_Init == 1){
				
				    if(Control_Info.Init.Balance_Init.IF_Balance_Init == 0){
				
							GIM_Motor_Motor_CAN_TxMessage(&FDCAN3_TxFrame,&GIM_8108_Motor[0],0,0,0,0, -Control_Info.L_Leg_Info.Moment.T);
	            GIM_Motor_Motor_CAN_TxMessage(&FDCAN3_TxFrame,&GIM_8108_Motor[1],0,0,0,0,  Control_Info.R_Leg_Info.Moment.T);
				      DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[0],0,0,0,0,0);		
				      DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[1],0,0,0,0,0);
				      DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[2],0,0,0,0,0);
				      DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[3],0,0,0,0,0);
						
						
				
				    }else if(Control_Info.Init.Balance_Init.IF_Balance_Init == 1){
				
				
							 if(Control_Info.Chassis_Situation == CHASSIS_BALANCE){

								 if(Control_Info.Climb.Climb_Flag == 1){
								 
									GIM_Motor_Motor_CAN_TxMessage(&FDCAN3_TxFrame,&GIM_8108_Motor[0],0,0.f,0,0.f,0);
									GIM_Motor_Motor_CAN_TxMessage(&FDCAN3_TxFrame,&GIM_8108_Motor[1],0,0.f,0,0.f,0);
									DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[0],0,0,7.f,1.f,0);		
									DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[1],0,0,7.f,1.f,0);
									DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[2],0,0,7.f,1.f,0);
									DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[3],0,0,7.f,1.f,0);
								 
								 }else{
								 
									GIM_Motor_Motor_CAN_TxMessage(&FDCAN3_TxFrame,&GIM_8108_Motor[0],0,0,0,0, -Control_Info.L_Leg_Info.Moment.T);
									GIM_Motor_Motor_CAN_TxMessage(&FDCAN3_TxFrame,&GIM_8108_Motor[1],0,0,0,0,  Control_Info.R_Leg_Info.Moment.T);
//									DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[0],0,0,0,0,Control_Info.L_Leg_Info.SendValue.T1);		
//									DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[1],0,0,0,0,Control_Info.L_Leg_Info.SendValue.T2);
//									DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[2],0,0,0,0,Control_Info.R_Leg_Info.SendValue.T2);
//									DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[3],0,0,0,0,Control_Info.R_Leg_Info.SendValue.T1);
						
								 }
				  }
			
			  }
			 }
		  } 
			 
		}
			
		}else{
		
			
			if(Control_Info.Slip.Slip_Flag == 1){
      
				
				GIM_Motor_Motor_CAN_TxMessage(&FDCAN3_TxFrame,&GIM_8108_Motor[0],0,0,0,5.f,0);
			  GIM_Motor_Motor_CAN_TxMessage(&FDCAN3_TxFrame,&GIM_8108_Motor[1],0,0,0,5.f,0);
			  DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[0],0,0,10.f,2.f,0);		
			  DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[1],0,0,10.f,2.f,0);
			  DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[2],0,0,10.f,2.f,0);
	      DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[3],0,0,10.f,2.f,0);
				
				
				
				
			}else{
			
			  GIM_Motor_Motor_CAN_TxMessage(&FDCAN3_TxFrame,&GIM_8108_Motor[0],0,0,0,0,0);
			  GIM_Motor_Motor_CAN_TxMessage(&FDCAN3_TxFrame,&GIM_8108_Motor[1],0,0,0,0,0);
			  DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[0],0,0,0.f,0.f,0);		
			  DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[1],0,0,0.f,0.f,0);
			  DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[2],0,0,0.f,0.f,0);
	      DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[3],0,0,0.f,0.f,0);
		
			
			}

		
		}
		
	
		osDelay(1);
  }
 
}

static void Referee_System_Info_Send_Gimbal(void){

	
	SendGimbal_TxFrame.Header.Identifier = 0x301;
  SendGimbal_TxFrame.Data[0] = (uint8_t)(Referee_System_Info.Power_Heat_Data.Shooter_17mm_1_Barrel_Heat >> 8);
  SendGimbal_TxFrame.Data[1] = (uint8_t)(Referee_System_Info.Power_Heat_Data.Shooter_17mm_1_Barrel_Heat);
	SendGimbal_TxFrame.Data[2] = (uint8_t)(Referee_System_Info.Robot_Status.Shooter_Barrel_Heat_Limit >> 8);
  SendGimbal_TxFrame.Data[3] = (uint8_t)(Referee_System_Info.Robot_Status.Shooter_Barrel_Heat_Limit);
	SendGimbal_TxFrame.Data[4] = (uint8_t)(Referee_System_Info.Power_Heat_Data.Update_Flag);
	 Referee_System_Info.Power_Heat_Data.Update_Flag = 0;
		
  User_FDCAN_AddMessageToTxFifoQ(&SendGimbal_TxFrame);

}


static void Check_Motor_Online(){

	
 for(uint8_t i = 0; i<4; i++){	
	
	    if(DM_8009_Motor[i].Oline_cnt>200){
	      DM_8009_Motor[i].Oline_cnt--;
		  DM_8009_Motor[i].lost = 0;
		}else{
		    DM_8009_Motor[i].Oline_cnt = 0;
		    DM_8009_Motor[i].lost = 1;
		}
  }
  
  for(uint8_t i = 0; i<2; i++){	
	
	    if(GIM_8108_Motor[i].Oline_cnt>200){
	      
		  GIM_8108_Motor[i].Oline_cnt--;
		  GIM_8108_Motor[i].lost = 0;

		}else{

		  GIM_8108_Motor[i].Oline_cnt = 0;
		  GIM_8108_Motor[i].lost = 1;
		  
		}
   }
  
	if(DM_8009_Motor[0].lost + DM_8009_Motor[1].lost + DM_8009_Motor[2].lost + DM_8009_Motor[3].lost + GIM_8108_Motor[0].lost + GIM_8108_Motor[1].lost == 0) Control_Info.Motor_Online.Enable_Flag = 1;
	else Control_Info.Motor_Online.Enable_Flag = 0;
	
	
	
}
static void Check_Motor_Enable(){

     for(uint8_t i = 0; i<4; i++){	
          DM_Motor_Enable_Cnt = 30;
			 if( DM_8009_Motor[i].lost == 1){
			   MIT_Motor_Command(&FDCAN2_TxFrame,i+3,Motor_Enable);
			   while(DM_Motor_Enable_Cnt > 0) DM_Motor_Enable_Cnt--;
			 }
		 } 
      
		 for(uint8_t i = 0; i<2; i++){	
          GIM_Motor_Enable_Cnt = 30;
			 if( GIM_8108_Motor[i].lost == 1){
			   MIT_Motor_Command(&FDCAN2_TxFrame,i+1,Motor_Enable);
			   while(GIM_Motor_Enable_Cnt > 0) GIM_Motor_Enable_Cnt--;
			 }
		 } 
}
