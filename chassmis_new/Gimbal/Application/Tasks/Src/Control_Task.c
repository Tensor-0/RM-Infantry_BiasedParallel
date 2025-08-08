/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Control_Task.c
  * @brief          : Control task
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
#include "Control_Task.h"
#include "INS_Task.h"
#include "vision_task.h"
#include "remote_control.h"
#include "motor.h"
#include "referee_info.h"
#include "tim.h"
#include "bsp_uart.h"
#include "usart.h"

#include <stdio.h>
#include <stdarg.h>
#include "string.h"
/* Private variables ---------------------------------------------------------*/

Control_Info_Typedef Control_Info={
// .Gimbal = {
//   .Measure.Yaw_Angle = &INS_Info.yaw_angle,
//	 .Measure.Pitch_Angle = &INS_Info.rol_angle,
//	 .Measure.Yaw_Gyro = &INS_Info.gyro[2],
//   .Measure.Pitch_Gyro = &INS_Info.gyro[1],
// },
// .Shoot = {
//   .Measure.ShootLeftSpeed = &DJI_Motor[Left_Shoot].Data.velocity,
//   .Measure.ShootRightSpeed = &DJI_Motor[Right_Shoot].Data.velocity,
//	 .Measure.TriggerSpeed = & DJI_Motor[Trigger].Data.velocity,
//	 .Measure.TriggerAngle = & DJI_Motor[Trigger].Data.angle,
//	 .Trigger_Buf = 8,
// },
 
};
/* Private function prototypes -----------------------------------------------*/
static void Control_Task_Init(void);
static void Control_Mode_Update(Control_Info_Typedef *Control_Info);
static void Gimbal_Mode_Update(Control_Info_Typedef *Control_Info);
static void Gimbal_Target_Update(Control_Info_Typedef *Control_Info);
static void Gimbal_Measure_Update(Control_Info_Typedef *Control_Info);
static void Gimbal_Control_Update(Control_Info_Typedef *Control_Info);
static void Gimbal_Output_Update(Control_Info_Typedef *Control_Info);
static void Shoot_Mode_Update(Control_Info_Typedef *Control_Info);
static void Shoot_Measure_Update(Control_Info_Typedef *Control_Info);
static void Shoot_Target_Update(Control_Info_Typedef *Control_Info);
static void Shoot_Control_Update(Control_Info_Typedef *Control_Info);
static void Shoot_Heat_Limit(Control_Info_Typedef *Control_Info);


PID_Info_TypeDef PID_Shoot[2];
PID_Info_TypeDef PID_Trigger[2];

PID_Info_TypeDef PID_Pitch_Vision[2];
PID_Info_TypeDef PID_Yaw_Vision[2];

PID_Info_TypeDef PID_Pitch_KeyBoard[2];
PID_Info_TypeDef PID_Yaw_KeyBoard[2];


float PID_Pitch_Vision_Angle_Param[6]  = {1.2f,0.001f,10.f,0.001f,1000.f,200.f};
float PID_Pitch_Vision_Velocity_Param[6] = {-0.8f,0.1f,0,0,2.5f,7.f};

float PID_Yaw_Vision_Angle_Param[6]  = {1.f,0.f,0,0,0.f,50.f};
float PID_Yaw_Vision_Velocity_Param[6] = {2.f,0.1f,0,0,1.2f,3.f};

float PID_Pitch_KeyBoard_Angle_Param[6]  = {0.8f,0.f,10.f,0.001f,0.f,200.f};
float PID_Pitch_KeyBoard_Velocity_Param[6] = {-0.8f,0.1f,0,0,2.5f,7.f};

float PID_Yaw_KeyBoard_Angle_Param[6]  = {0.8f,0.f,0,0,0.f,50.f};
float PID_Yaw_KeyBoard_Velocity_Param[6] = {1.f,0.1f,0,0,1.2f,3.f};



float PID_Shoot_Velocity_Param[6]  = {13.f,0.1f,0,0,5000.f,12000.f};

float PID_Trigger_Angle_Param[6]     = {30.f,0.1f,0,0,2000,12000};
float PID_Trigger_Velocity_Param[6]  = {13.f,0.1f,0,0,5000.f,12000.f};

//uint8_t SendBuf[12] = {0};
//uint8_t *float1,*float2;
//int16_t Yaw_Veloctiy;
//uint8_t buf = 0;
//int16_t Target;
//float TargetP = 0;
//float forward = 0;
//float forward1 = 0;

/* USER CODE BEGIN Header_Control_Task */
/**
* @brief Function implementing the StartControlTas thread.
* @param argument: Not used
* @retval Noneppp
*/
/* USER CODE END Header_Control_Task */

void Control_Task(void const * argument)
{
  /* USER CODE BEGIN Control_Task */
   TickType_t systick = 0;

   Control_Task_Init();

  /* Infinite loop */
  for(;;)
  {
    systick = osKernelSysTick();
   
		Control_Mode_Update(&Control_Info);
		Gimbal_Mode_Update(&Control_Info);
    Gimbal_Measure_Update(&Control_Info);
    Gimbal_Target_Update(&Control_Info);
    Gimbal_Control_Update(&Control_Info);
    Gimbal_Output_Update(&Control_Info);
		Shoot_Heat_Limit(&Control_Info);
	  Shoot_Mode_Update(&Control_Info);
		Shoot_Measure_Update(&Control_Info);
    Shoot_Target_Update(&Control_Info);
    Shoot_Control_Update(&Control_Info);
	
		Usart_Justfloat_Transmit( Control_Info.Refree.Heat_Update_Flag,0,0);
    osDelayUntil(&systick,1);
  }
  /* USER CODE END Control_Task */
}

static void Control_Task_Init(void){

  PID_Init(&PID_Pitch_Vision[0],PID_POSITION,PID_Pitch_Vision_Angle_Param);
	PID_Init(&PID_Pitch_Vision[1],PID_POSITION,PID_Pitch_Vision_Velocity_Param);
	
	PID_Init(&PID_Yaw_Vision[0],PID_POSITION,PID_Yaw_Vision_Angle_Param);
	PID_Init(&PID_Yaw_Vision[1],PID_POSITION,PID_Yaw_Vision_Velocity_Param);
	
	PID_Init(&PID_Pitch_KeyBoard[0],PID_POSITION,PID_Pitch_KeyBoard_Angle_Param);
	PID_Init(&PID_Pitch_KeyBoard[1],PID_POSITION,PID_Pitch_KeyBoard_Velocity_Param);
	
	PID_Init(&PID_Yaw_KeyBoard[0],PID_POSITION,PID_Yaw_KeyBoard_Angle_Param);
	PID_Init(&PID_Yaw_KeyBoard[1],PID_POSITION,PID_Yaw_KeyBoard_Velocity_Param);
	
	
	PID_Init(&PID_Shoot[0],PID_POSITION,PID_Shoot_Velocity_Param);
	PID_Init(&PID_Shoot[1],PID_POSITION,PID_Shoot_Velocity_Param);
	
	PID_Init(&PID_Trigger[0],PID_POSITION,PID_Trigger_Angle_Param);
	PID_Init(&PID_Trigger[1],PID_POSITION,PID_Trigger_Velocity_Param);

}

static void Control_Mode_Update(Control_Info_Typedef *Control_Info){

  if(remote_ctrl.rc_lost != 1){
	
     if(remote_ctrl.rc.s[0] == 1 && remote_ctrl.rc.s[1] == 1){
	
	     Control_Info->Control_Mode = CONTROL_KEYBOARD;
	
	   }else{
		 
		   Control_Info->Control_Mode = CONTROL_REMOTE;

		 }
	}else{
	
	   Control_Info->Control_Mode = CONTROL_OFF;
	
	}


}

static void Gimbal_Mode_Update(Control_Info_Typedef *Control_Info){

  if(remote_ctrl.rc_lost != 1){

		if( Control_Info->Control_Mode == CONTROL_REMOTE){
		
       if(remote_ctrl.rc.s[0] == 2){
	
	       Control_Info->Gimbal.Mode = GIMBAL_OFF;
	
 	     }else if(remote_ctrl.rc.s[0] == 3){
		 
	 	     Control_Info->Gimbal.Mode = GIMBAL_IMU;
		 
	  	 }else if(remote_ctrl.rc.s[0] == 1){
			 
			  	Control_Info->Gimbal.Mode = GIMBAL_VISION;
				 
			 }	
	
		}else if(Control_Info->Control_Mode == CONTROL_KEYBOARD){
		
		  if(remote_ctrl.mouse.press_r == 1) 	Control_Info->Gimbal.Mode = GIMBAL_VISION;
      else 	Control_Info->Gimbal.Mode = GIMBAL_IMU;
		
		
		
		}	 
			 
	}else{
	
	  	 Control_Info->Gimbal.Mode = GIMBAL_OFF;
	
	}

}
static void Gimbal_Measure_Update(Control_Info_Typedef *Control_Info){

   Control_Info->Gimbal.Measure.Pitch_Angle = INS_Info.rol_angle;
	 Control_Info->Gimbal.Measure.Yaw_Angle   = INS_Info.yaw_angle;
   Control_Info->Gimbal.Measure.Pitch_Gyro  = INS_Info.gyro[1];
	 Control_Info->Gimbal.Measure.Yaw_Gyro    = INS_Info.gyro[2];
	 Control_Info->Shoot.Measure.TriggerAngle    = DJI_Motor[0].Data.angle;
	
}

static void Gimbal_Target_Update(Control_Info_Typedef *Control_Info){

	 if( Control_Info->Control_Mode == CONTROL_REMOTE){

      if( Control_Info->Gimbal.Mode == GIMBAL_IMU){
			
			   Control_Info->Gimbal.Target.Pitch_Angle +=  remote_ctrl.rc.ch[1]*0.0004f;
				 Control_Info->Gimbal.Target.Yaw_Angle   += -remote_ctrl.rc.ch[0]*0.0004f;

			}else if(Control_Info->Gimbal.Mode == GIMBAL_VISION){
			
			if(Control_Info->Gimbal.Vision.Distance != -1){	
				
			   Control_Info->Gimbal.Target.Pitch_Angle =  Control_Info->Gimbal.Vision.Pitch_Angle;
				 Control_Info->Gimbal.Target.Yaw_Angle   =  Control_Info->Gimbal.Vision.Yaw_Angle;
			
			}else{
			
			   Control_Info->Gimbal.Target.Pitch_Angle =  Control_Info->Gimbal.Target.Last_Pitch_Angle; 
				 Control_Info->Gimbal.Target.Yaw_Angle   =  Control_Info->Gimbal.Target.Last_Yaw_Angle;
			
			}	
				
			}else if(Control_Info->Gimbal.Mode == GIMBAL_OFF){
			
			   Control_Info->Gimbal.Target.Pitch_Angle = Control_Info->Gimbal.Measure.Pitch_Angle;
			   Control_Info->Gimbal.Target.Yaw_Angle   = Control_Info->Gimbal.Measure.Yaw_Angle;

			} 

		
	 }else if(Control_Info->Control_Mode == CONTROL_KEYBOARD){
	 
	    if( Control_Info->Gimbal.Mode == GIMBAL_IMU){
	   				 Control_Info->Gimbal.Target.Yaw_Angle     += -remote_ctrl.mouse.x*0.0004f;
		  			 Control_Info->Gimbal.Target.Pitch_Angle   += -remote_ctrl.mouse.y*0.0004f;
			}else if(Control_Info->Gimbal.Mode == GIMBAL_VISION){
			
		    if(Control_Info->Gimbal.Vision.Distance != 1){	
				
			   Control_Info->Gimbal.Target.Pitch_Angle =  Control_Info->Gimbal.Vision.Pitch_Angle;
				 Control_Info->Gimbal.Target.Yaw_Angle   =  Control_Info->Gimbal.Vision.Yaw_Angle;
			
			}else{
			
			   Control_Info->Gimbal.Target.Pitch_Angle =  Control_Info->Gimbal.Target.Last_Pitch_Angle; 
				 Control_Info->Gimbal.Target.Yaw_Angle   =  Control_Info->Gimbal.Target.Last_Yaw_Angle;
			
			}	
		 
	   }
  }else{
		
		   	 Control_Info->Gimbal.Target.Pitch_Angle = Control_Info->Gimbal.Measure.Pitch_Angle;
			   Control_Info->Gimbal.Target.Yaw_Angle   = Control_Info->Gimbal.Measure.Yaw_Angle;
		   
		
		
		
		}
	 
	if(Damiao_Pitch_Motor.Data.Position > 0.78f && Damiao_Pitch_Motor.Data.Position < 1.6f){
	
	   Control_Info->Gimbal.Target.Pitch_Angle = Control_Info->Gimbal.Target.Pitch_Angle;
	
	}else {
		
		if(Damiao_Pitch_Motor.Data.Position <= 0.78f){
	
	     if(Control_Info->Gimbal.Target.Pitch_Angle >= Control_Info->Gimbal.Target.Last_Pitch_Angle){
			 
			    Control_Info->Gimbal.Target.Pitch_Angle  = Control_Info->Gimbal.Target.Last_Pitch_Angle;
				 
			 }else{
			 
			   Control_Info->Gimbal.Target.Pitch_Angle  = Control_Info->Gimbal.Target.Pitch_Angle;

			 }
					
			
	}else if(Damiao_Pitch_Motor.Data.Position > 1.6f){
		 
       if(Control_Info->Gimbal.Target.Pitch_Angle <= Control_Info->Gimbal.Target.Last_Pitch_Angle){
			 
			    Control_Info->Gimbal.Target.Pitch_Angle  = Control_Info->Gimbal.Target.Last_Pitch_Angle;
				 
			 }else{
			 
			   Control_Info->Gimbal.Target.Pitch_Angle  = Control_Info->Gimbal.Target.Pitch_Angle;

			 }
					 
   }
 }
		
    if( Control_Info->Gimbal.Target.Yaw_Angle >= 180.f)  Control_Info->Gimbal.Target.Yaw_Angle-=360.f;
    else if ( Control_Info->Gimbal.Target.Yaw_Angle <= -180.f)  Control_Info->Gimbal.Target.Yaw_Angle+=360.f;
 
 
	  Control_Info->Gimbal.Feed_Forward.Pitch_Angle = -(Control_Info->Gimbal.Target.Pitch_Angle - Control_Info->Gimbal.Target.Last_Pitch_Angle)*15.f;
 	  Control_Info->Gimbal.Feed_Forward.Yaw_Angle   =  (Control_Info->Gimbal.Target.Yaw_Angle   - Control_Info->Gimbal.Target.Last_Yaw_Angle)*10.f;

 
    Control_Info->Gimbal.Target.Last_Yaw_Angle = Control_Info->Gimbal.Target.Yaw_Angle;
    Control_Info->Gimbal.Target.Last_Pitch_Angle = Control_Info->Gimbal.Target.Pitch_Angle;

		
		
}

static void Gimbal_Control_Update(Control_Info_Typedef *Control_Info){


	if(Control_Info->Gimbal.Mode == GIMBAL_IMU){
	
    	 f_PID_Calculate(&PID_Pitch_KeyBoard[0],Control_Info->Gimbal.Target.Pitch_Angle,Control_Info->Gimbal.Measure.Pitch_Angle);
       f_PID_Calculate(&PID_Pitch_KeyBoard[1],PID_Pitch_KeyBoard[0].Output,Control_Info->Gimbal.Measure.Pitch_Gyro);	
	
	}else if(Control_Info->Gimbal.Mode == GIMBAL_VISION){
	
	     f_PID_Calculate(&PID_Pitch_Vision[0],Control_Info->Gimbal.Target.Pitch_Angle,Control_Info->Gimbal.Measure.Pitch_Angle);
       f_PID_Calculate(&PID_Pitch_Vision[1],PID_Pitch_Vision[0].Output,Control_Info->Gimbal.Measure.Pitch_Gyro);	
	
	}
	
      Control_Info->Gimbal.Yaw_Err =  Control_Info->Gimbal.Target.Yaw_Angle - Control_Info->Gimbal.Measure.Yaw_Angle;
      
      if(  Control_Info->Gimbal.Yaw_Err >= 180.f)   Control_Info->Gimbal.Yaw_Err -= 360.f;
      else if (  Control_Info->Gimbal.Yaw_Err <= -180.f)  Control_Info->Gimbal.Yaw_Err +=360.f;
 
  if(Control_Info->Gimbal.Mode == GIMBAL_IMU){
		
		 	f_PID_Calculate(&PID_Yaw_KeyBoard[0],Control_Info->Gimbal.Yaw_Err,0);
	    f_PID_Calculate(&PID_Yaw_KeyBoard[1],PID_Yaw_KeyBoard[0].Output,Control_Info->Gimbal.Measure.Yaw_Gyro);	
	
	
	}else if(Control_Info->Gimbal.Mode == GIMBAL_VISION){
    
  		f_PID_Calculate(&PID_Yaw_Vision[0],Control_Info->Gimbal.Yaw_Err,0);
	    f_PID_Calculate(&PID_Yaw_Vision[1],PID_Yaw_Vision[0].Output,Control_Info->Gimbal.Measure.Yaw_Gyro);		
	
	}
     

}

static void Gimbal_Output_Update(Control_Info_Typedef *Control_Info){

	
	if(Control_Info->Gimbal.Mode != GIMBAL_OFF){
		
	 	if(Control_Info->Gimbal.Mode == GIMBAL_VISION){
	
      Control_Info->Gimbal.Output.Yaw   = PID_Yaw_Vision[1].Output   + Control_Info->Gimbal.Feed_Forward.Yaw_Angle;
      Control_Info->Gimbal.Output.Pitch = PID_Pitch_Vision[1].Output + Control_Info->Gimbal.Feed_Forward.Pitch_Angle;

		}else if(Control_Info->Gimbal.Mode == GIMBAL_IMU){
		  
			Control_Info->Gimbal.Output.Yaw   = PID_Yaw_KeyBoard[1].Output    ;
      Control_Info->Gimbal.Output.Pitch = PID_Pitch_KeyBoard[1].Output  ;
		
		}
			
	}else{
	
	     Control_Info->Gimbal.Output.Yaw = 0;
	    Control_Info->Gimbal.Output.Pitch  = 0;
	}
 

}

static void Shoot_Mode_Update(Control_Info_Typedef *Control_Info){

	if(remote_ctrl.rc_lost != 1){

		if( Control_Info->Control_Mode == CONTROL_REMOTE){
		
       if(remote_ctrl.rc.s[0] == 2){
	
	       Control_Info->Shoot.Mode = SHOOTER_OFF;
	
 	     }else if(remote_ctrl.rc.s[0] == 3){
		 
	 	     Control_Info->Shoot.Mode = SHOOTER_BURSTS;
		 
	  	 }else if(remote_ctrl.rc.s[0] == 1){
			 
			  	Control_Info->Shoot.Mode = SHOOTER_VISION;
				 
			 }	
	
		}else if(Control_Info->Control_Mode == CONTROL_KEYBOARD){
		
      
  			if( Control_Info->Shoot.Mode == SHOOTER_OFF) Control_Info->Shoot.Mode = SHOOTER_SINGLE;
			
		
		}else{
	
	 	     Control_Info->Shoot.Mode = SHOOTER_OFF;
	
	  }
	
	}


}
static void Shoot_Measure_Update(Control_Info_Typedef *Control_Info){

  Control_Info->Shoot.Measure.ShootLeftSpeed  = DJI_Motor[1].Data.velocity;
	Control_Info->Shoot.Measure.ShootRightSpeed = DJI_Motor[2].Data.velocity;

	Control_Info->Shoot.Measure.TriggerAngle  = DJI_Motor[0].Data.angle;
	Control_Info->Shoot.Measure.TriggerSpeed  = DJI_Motor[0].Data.velocity;
   

}

static void Shoot_Heat_Limit(Control_Info_Typedef *Control_Info){

	if(remote_ctrl.mouse.press_l ==1  && Control_Info->Shoot.Limit.First_Press_L == 0){
	
		Control_Info->Shoot.Limit.First_Press_L = 1;
		Control_Info->Shoot.Limit.When_Press_L_Heat = Control_Info->Refree.Shooter_Barrel_Heat_limit - Control_Info->Refree.Shooter_17mm_1_Barrel_Heat;
		Control_Info->Shoot.Limit.Allowable_Shoot_17mm = Control_Info->Shoot.Limit.When_Press_L_Heat / 10.f;
	
	
	}else if(remote_ctrl.mouse.press_l == 0){
	
	  Control_Info->Shoot.Limit.First_Press_L = 0;
  	Control_Info->Shoot.Limit.When_Press_L_Heat = 0;
	  Control_Info->Shoot.Limit.Allowable_S = 0;
	  Control_Info->Shoot.Limit.Allow_Flag = 0;
	  Control_Info->Shoot.Limit.Count	= 0;
		Control_Info->Shoot.Limit.Allowable_Shoot_17mm = 0;
	
	}
	
 if(remote_ctrl.mouse.press_l == 1 && Control_Info->Shoot.Limit.First_Press_L == 1){
	
	 Control_Info->Shoot.Target.TriggerSpeed = 5400;
 
	 
	 if(Control_Info->Refree.Heat_Update_Flag == 1) Control_Info->Shoot.Limit.Allowable_Shoot_17mm = Control_Info->Refree.Shooter_Barrel_Heat_limit - Control_Info->Refree.Shooter_17mm_1_Barrel_Heat;
	 
	 
   Control_Info->Shoot.Limit.Allowable_S = (Control_Info->Shoot.Limit.Allowable_Shoot_17mm  )/ 20.f  ;
   
	 
   if( Control_Info->Shoot.Limit.Count < (Control_Info->Shoot.Limit.Allowable_S * 1000 + 40)  ){
     
		 Control_Info->Shoot.Limit.Count++;
     Control_Info->Shoot.Limit.Allow_Flag = 1;
		 
	 }else if(Control_Info->Shoot.Limit.Count >= Control_Info->Shoot.Limit.Allowable_S * 1000 ){
	 
	 
	   Control_Info->Shoot.Limit.Allow_Flag = 0;
	 
	 
	 }
 }
}


static void Shoot_Target_Update(Control_Info_Typedef *Control_Info){

   if(Control_Info->Shoot.Mode != SHOOTER_OFF){
	  
	   if( Control_Info->Control_Mode == CONTROL_REMOTE){
 
	     Control_Info->Shoot.Target.ShootSpeed = 7300 ;
//	     Control_Info->Shoot.Target.TriggerSpeed = remote_ctrl.rc.ch[4]*12;
	      Control_Info->Shoot.Target.TriggerSpeed = -5400 * Control_Info->Shoot.Limit.Allow_Flag ;
		 }else if(Control_Info->Control_Mode == CONTROL_KEYBOARD){
		 
		   Control_Info->Shoot.Target.ShootSpeed = 7300 ;

	     if(remote_ctrl.mouse.press_l == 1 && KeyBoard_Info.Flag.Press_L == 0){
	
		     Control_Info->Shoot.Target.TriggerAngle -= (float) (360.f/8.f);
         KeyBoard_Info.Flag.Press_L = 1;
	  
	     }else if(remote_ctrl.mouse.press_l==0) KeyBoard_Info.Flag.Press_L = 0;
	
	     if( Control_Info->Shoot.Target.TriggerAngle >= 180.f)  Control_Info->Shoot.Target.TriggerAngle  -=   360.f;
       else if ( Control_Info->Shoot.Target.TriggerAngle <= -180.f)  Control_Info->Shoot.Target.TriggerAngle +=   360.f;
	 
	     Control_Info->Shoot.TriggerAngle_Err = Control_Info->Shoot.Target.TriggerAngle - Control_Info->Shoot.Measure.TriggerAngle;
		
		   if( Control_Info->Shoot.TriggerAngle_Err >= 180.f )         Control_Info->Shoot.TriggerAngle_Err -= 360.f;
  	   else if( Control_Info->Shoot.TriggerAngle_Err <= -180.f)    Control_Info->Shoot.TriggerAngle_Err += 360.f;

		 }      
	 
	 }else{
	 
	    Control_Info->Shoot.Target.ShootSpeed = 0;
	    Control_Info->Shoot.Target.TriggerSpeed = 0;
	 
	 }


}

static void Shoot_Control_Update(Control_Info_Typedef *Control_Info){
   
    f_PID_Calculate(&PID_Shoot[0], -Control_Info->Shoot.Target.ShootSpeed,Control_Info->Shoot.Measure.ShootLeftSpeed);
    f_PID_Calculate(&PID_Shoot[1],  Control_Info->Shoot.Target.ShootSpeed,Control_Info->Shoot.Measure.ShootRightSpeed);
	
	
	if(Control_Info->Shoot.Mode == SHOOTER_SINGLE){
		MiniPC_SendBuf[1] = 0X03;
	  f_PID_Calculate(&PID_Trigger[0],  Control_Info->Shoot.TriggerAngle_Err,0.f);
    f_PID_Calculate(&PID_Trigger[1], PID_Trigger[0].Output,Control_Info->Shoot.Measure.TriggerSpeed);
  
	}else{
	
	  f_PID_Calculate(&PID_Trigger[1], Control_Info->Shoot.Target.TriggerSpeed,Control_Info->Shoot.Measure.TriggerSpeed);

	
	}
	
	
	
//	  Control_Info->SendValue[0] = (int16_t)(PID_Trigger[1].Output);
//	  Control_Info->SendValue[1] = (int16_t)(PID_Shoot[0].Output);
//	  Control_Info->SendValue[2] = (int16_t)(PID_Shoot[1].Output);

}