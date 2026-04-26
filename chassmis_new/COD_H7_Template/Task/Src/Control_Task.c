#include "Control_Task.h"
#include "bsp_can.h"
#include "cmsis_os.h"
#include "Motor.h"
#include "pid.h"
#include "usart.h"
#include "remote_control.h"
#include "math.h"
#include "arm_math.h" 
#include "Bmi088.h" 
#include "Quaternion.h"
#include "bsp_uart.h"
#include "INS_Task.h"
#include "lpf.h"
#include "bsp_rs485.h"
#include "fdcan.h"
#include "RLS.h"
#include "HIPNUC.h"
#include "adc.h"
#include "ramp.h"
#include "Kalman_Filter.h"
#include "MPC_Task.h"
#include "Referee_System.h"





float K11[6] = {0,-36.140790f,105.489227f,-136.322613f,1.919991f};
float K12[6] = {0,26.834069f,-27.356129f,-12.472068f,0.340263f};
float K13[6] = {0,-157.819813f,179.659458f,-74.294691f,-0.615055f};
float K14[6] = {0,-103.535471f,123.894827f,-59.108802f,-1.130595f};
float K15[6] = {0,97.529204f,33.106580f,-107.687034f,53.362493f};
float K16[6] = {0,19.295810f,-11.843360f,-2.315494f,4.629933f};
float K21[6] = {0,1023.753828f,-1008.054199f,316.613274f,11.685807f};
float K22[6] = {0,115.149588f,-127.877647f,52.962945f,0.803046f};
float K23[6] = {0,324.863650f,-162.780346f,-53.460392f,45.222067f};
float K24[6] = {0,198.382539f,-86.659354f,-48.911500f,37.585154f};
float K25[6] = {0,2065.511401f,-2423.095976f,1040.219425f,-5.051954f};
float K26[6] = {0,60.218950f,-103.798952f,62.137222f,-6.008688f};



static float PID_Leg_length_thrust_param[7]={1000.f,1.f,100000.f,0.f,0,10,200};
static float PID_Tp_param[7]={0.f,0.f,0.f,0.f,0,0,0};
static float PID_Phi_param[7]={0.f,0.f,0.f,0.f,0,0,0};
static float PID_Yaw_param[2][7] = {

	[0] = {1.f,0.f,0.f,0.f,0.f,0.f,50.f},
	[1] = {0.8f,0.0f,50.f,0.f,0.f,0.f,6.f},
	
};
static float PID_Leg_Coordinate_param[7]={300.f,0.f,20.0f,0.f,0.f,0.f,20};
static void Control_Init(Control_Info_Typedef *Control_Info);
static void Check_Control_Slip(Control_Info_Typedef *Control_Info);
static void Control_Mode_Update(Control_Info_Typedef *Control_Info);
static void Joint_Angle_Zero_Offset(Control_Info_Typedef *Control_Info);
static void VMC_Calculate(Control_Info_Typedef *Control_Info);
static void Control_LQR_K_Update(Control_Info_Typedef *Control_Info);
static void Control_LQR_Stand_T_Balance_Tp_Calculate(Control_Info_Typedef *Control_Info);
static void Control_Target_Update(Control_Info_Typedef *Control_Info);
static void Control_Measure_Update(Control_Info_Typedef *Control_Info);
static void Control_LQR_X_Update(Control_Info_Typedef *Control_Info);
static void VMC_Measure_F_Tp_Calculate(Control_Info_Typedef *Control_Info);
static void Control_Comprehensive_F_Calculate(Control_Info_Typedef *Control_Info);
static void VMC_F_Tp_To_Joint_Calculate(Control_Info_Typedef *Control_Info);
static void Power_Control(Control_Info_Typedef *Control_Info);
static void Jump(Control_Info_Typedef *Control_Info);
static void Climb_20cm(Control_Info_Typedef *Control_Info);


PID_Info_TypeDef PID_Leg_length_thrust[2];
PID_Info_TypeDef  PID_Tp;
PID_Info_TypeDef PID_Phi;
PID_Info_TypeDef PID_Yaw[2];
PID_Info_TypeDef PID_Leg_Coordinate;


LowPassFilter1p_Info_TypeDef PD_Yaw_LPF;
LowPassFilter1p_Info_TypeDef L_Position_LPF;
LowPassFilter1p_Info_TypeDef R_Position_LPF;


Control_Info_Typedef Control_Info ={

  .L_Leg_Info ={
	   .VMC = {
			   .L5 = 0.15f,
		     .L1 = 0.15f,
		     .L4 = 0.15f,
			   .L2 = 0.27f,
		     .L3 = 0.27f,
			   .Target_L0 = 0.17f,
		 },
	},
   .R_Leg_Info ={
	   .VMC = {
			   .L5 = 0.15f,
		     .L1 = 0.15f,
		     .L4 = 0.15f,
			   .L2 = 0.27f,
		     .L3 = 0.27f,
			   .Target_L0 = 0.18f,

		 },
	},

 


};

TickType_t systick = 0;
__attribute__((section (".AXI_SRAM"))) uint16_t ADC_Val[2];
float Vbus;
void Control_Task(void const * argument)
{
 

 PID_Init(&PID_Leg_length_thrust[0],PID_POSITION,PID_Leg_length_thrust_param);
 PID_Init(&PID_Leg_length_thrust[1],PID_POSITION,PID_Leg_length_thrust_param);
 PID_Init(&PID_Tp,PID_POSITION,PID_Tp_param);
 PID_Init(&PID_Phi,PID_POSITION,PID_Phi_param);
 PID_Init(&PID_Yaw[0],PID_POSITION,PID_Yaw_param[0]);
 PID_Init(&PID_Yaw[1],PID_POSITION,PID_Yaw_param[1]);
 PID_Init(&PID_Leg_Coordinate,PID_POSITION,PID_Leg_Coordinate_param);
// LowPassFilter1p_Init(&L_Position_LPF,0.01f,0.001f);
// LowPassFilter1p_Init(&R_Position_LPF,0.01f,0.001f);


	HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_Val,2);
	
	for(;;)
  {  
	
	    systick = osKernelSysTick();
			Vbus = (ADC_Val[0]*3.3f/65535)*11.0f;
      Control_Mode_Update(&Control_Info);
		  Control_Init(&Control_Info);
		  Joint_Angle_Zero_Offset(&Control_Info);
		  VMC_Calculate(&Control_Info);
		  Control_LQR_K_Update(&Control_Info);
		  Control_Target_Update(&Control_Info); 
		  Control_Measure_Update(&Control_Info); 
		  Control_LQR_X_Update(&Control_Info);
		  VMC_Measure_F_Tp_Calculate(&Control_Info);
		  Climb_20cm(&Control_Info);
		  Control_LQR_Stand_T_Balance_Tp_Calculate(&Control_Info);
      Control_Comprehensive_F_Calculate(&Control_Info);
			//Power_Control(&Control_Info);
		  VMC_F_Tp_To_Joint_Calculate(&Control_Info);

		 Usart_Justfloat_Transmit(PID_Yaw[1].Dout,0,0);
		
		usart_printf("%f,%f\n",Control_Info.L_Leg_Info.Support.FN,Control_Info.R_Leg_Info.Support.FN );
		
      osDelayUntil(&systick,1);
			
		
  }
  

}


static void Power_Control(Control_Info_Typedef *Control_Info){
	
	
if( Control_Info->Chassis_Situation == CHASSIS_BALANCE ){
	
//	Control_Info->Power_Control.Velocity_2[0] = GIM_8108_Motor[0].Data.Velocity * GIM_8108_Motor[0].Data.Velocity;
//	Control_Info->Power_Control.Velocity_2[1] = GIM_8108_Motor[1].Data.Velocity * GIM_8108_Motor[1].Data.Velocity;
//	Control_Info->Power_Control.All_Velocity_2 = Control_Info->Power_Control.Velocity_2[0] + Control_Info->Power_Control.Velocity_2[1];

//	
//	
//	Control_Info->Power_Control.Balance.Torgue[0] = -(Control_Info->L_Leg_Info.LQR_Output[0][4] + Control_Info->L_Leg_Info.LQR_Output[0][5]);
//	Control_Info->Power_Control.Balance.Torgue[1] =   Control_Info->R_Leg_Info.LQR_Output[0][4] + Control_Info->R_Leg_Info.LQR_Output[0][5];
//	
//	Control_Info->Power_Control.Balance.Torgue_2[0] = Control_Info->Power_Control.Balance.Torgue[0] * Control_Info->Power_Control.Balance.Torgue[0];
//	Control_Info->Power_Control.Balance.Torgue_2[1] = Control_Info->Power_Control.Balance.Torgue[1] * Control_Info->Power_Control.Balance.Torgue[1];
//	
//	Control_Info->Power_Control.Balance.All_Torgue_2 = 	Control_Info->Power_Control.Balance.Torgue_2[0] + 	Control_Info->Power_Control.Balance.Torgue_2[1];
//	
//	Control_Info->Power_Control.Balance.Mechanical_Power =   Control_Info->Power_Control.Balance.Torgue[0] * GIM_8108_Motor[0].Data.Velocity
//	                                                       + Control_Info->Power_Control.Balance.Torgue[1] * GIM_8108_Motor[1].Data.Velocity;
//	
//	Control_Info->Power_Control.Balance.Predict_Power =   Control_Info->Power_Control.Balance.Mechanical_Power + Control_Info->Power_Control.K1*Control_Info->Power_Control.Balance.All_Torgue_2
//	                                                                                                           + Control_Info->Power_Control.K2*Control_Info->Power_Control.All_Velocity_2;
//	
//	
//	Control_Info->Power_Control.Other.Torgue[0] = - (   Control_Info->L_Leg_Info.LQR_Output[0][0] + Control_Info->L_Leg_Info.LQR_Output[0][1]
//	                                                  + Control_Info->L_Leg_Info.LQR_Output[0][2] + Control_Info->L_Leg_Info.LQR_Output[0][3] + PID_Yaw[0].Output );
//  
//	Control_Info->Power_Control.Other.Torgue[1] =       Control_Info->R_Leg_Info.LQR_Output[0][0] + Control_Info->R_Leg_Info.LQR_Output[0][1]
//	                                                  + Control_Info->R_Leg_Info.LQR_Output[0][2] + Control_Info->R_Leg_Info.LQR_Output[0][3] - PID_Yaw[0].Output;
//	
//	Control_Info->Power_Control.Other.Torgue_2[0] = 	Control_Info->Power_Control.Other.Torgue[0] * 	Control_Info->Power_Control.Other.Torgue[0];
//	Control_Info->Power_Control.Other.Torgue_2[1] = 	Control_Info->Power_Control.Other.Torgue[1] * 	Control_Info->Power_Control.Other.Torgue[1];

//	Control_Info->Power_Control.Other.All_Torgue_2 = 	Control_Info->Power_Control.Other.Torgue_2[0] + 	Control_Info->Power_Control.Other.Torgue_2[1];

//	Control_Info->Power_Control.Other.Mechanical_Power =   Control_Info->Power_Control.Other.Torgue[0] * GIM_8108_Motor[0].Data.Velocity
//	                                                     + Control_Info->Power_Control.Other.Torgue[1] * GIM_8108_Motor[1].Data.Velocity;
//	
//	Control_Info->Power_Control.Other.Predict_Power =   Control_Info->Power_Control.Other.Mechanical_Power + Control_Info->Power_Control.K1*Control_Info->Power_Control.Other.All_Torgue_2
//	                                                                                                         + Control_Info->Power_Control.K2*Control_Info->Power_Control.All_Velocity_2;
	
	Control_Info->Power_Control.Velocity_2[0] = GIM_8108_Motor[0].Data.Velocity * GIM_8108_Motor[0].Data.Velocity;
	Control_Info->Power_Control.Velocity_2[1] = GIM_8108_Motor[1].Data.Velocity * GIM_8108_Motor[1].Data.Velocity;
	Control_Info->Power_Control.All_Velocity_2 = Control_Info->Power_Control.Velocity_2[0] + Control_Info->Power_Control.Velocity_2[1];


  Control_Info->Power_Control.Torgue[0] = 	GIM_8108_Motor[0].Data.Torque;
  Control_Info->Power_Control.Torgue[1] = 	GIM_8108_Motor[1].Data.Torque;

	
																																																					 
	Control_Info->Power_Control.Torgue_2[0] = Control_Info->Power_Control.Torgue[0]  * 	 Control_Info->Power_Control.Torgue[0] ;
	Control_Info->Power_Control.Torgue_2[1] = Control_Info->Power_Control.Torgue[1]  *	 Control_Info->Power_Control.Torgue[1]; 

	Control_Info->Power_Control.All_Torgue_2 =   Control_Info->Power_Control.Torgue_2[0]   + Control_Info->Power_Control.Torgue_2[1];

	Control_Info->Power_Control.Mechanical_Power =   Control_Info->Power_Control.Torgue[0] * GIM_8108_Motor[0].Data.Velocity
	                                               + Control_Info->Power_Control.Torgue[1] * GIM_8108_Motor[1].Data.Velocity;
	
 Control_Info->Power_Control.Predict_Power =  Control_Info->Power_Control.Mechanical_Power + Control_Info->Power_Control.K1*Control_Info->Power_Control.All_Torgue_2  + Control_Info->Power_Control.K2 *Control_Info->Power_Control.All_Velocity_2 + 1.44f;	


 if( Control_Info->Power_Control.Predict_Power < 0)	Control_Info->Power_Control.Predict_Power = 0;
	
// Power_Control_RLS_Info.Data.U[0] = Control_Info->Power_Control.Predict_Power;
// Power_Control_RLS_Info.Data.Y[0] = Referee_System_Info.Power_Heat_Data.Chassis_Power;

// Power_Control_RLS_Info.Data.X[0] = Control_Info->Power_Control.All_Torgue_2;
// Power_Control_RLS_Info.Data.X[1] = Control_Info->Power_Control.All_Velocity_2;	

if(Control_Info->Power_Control.Referee_System_Power_Update_Flag == 1){
	
// RLS_Update(&Power_Control_RLS_Info);
// Control_Info->Power_Control.K1 = 	Power_Control_RLS_Info.Data.W[0];
// Control_Info->Power_Control.K2 = 	Power_Control_RLS_Info.Data.W[1];
// Control_Info->Power_Control.Referee_System_Power_Update_Flag = 0;
}





}





}

static void Control_Mode_Update(Control_Info_Typedef *Control_Info){

  if(remote_ctrl.rc_lost == 0){

     if(remote_ctrl.rc.s[1] == 1 && remote_ctrl.rc.s[0] == 1  ){

       Control_Info->Control_Mode = CONTROL_KEYBOARD;

	   }else{

       Control_Info->Control_Mode = CONTROL_REMOTE;
     }


  }else{

    Control_Info->Control_Mode = CONTROL_OFF;

  }



}

static void Control_Init(Control_Info_Typedef *Control_Info){
	
   if(Control_Info->Control_Mode != CONTROL_OFF){

       if(Control_Info->Control_Mode == CONTROL_REMOTE){ 

          if(remote_ctrl.rc.s[1] == 1 || remote_ctrl.rc.s[1] == 3){
	
 	         Control_Info->Init.Begin_Init = 1;
	
 	        }else{
	
 	         Control_Info->Init.Begin_Init  = 0;
					if(Control_Info->Chassis_Situation == CHASSIS_BALANCE) Control_Info->Chassis_Situation = CHASSIS_WEAK;
	
 	        }

	     }else if(Control_Info->Control_Mode == CONTROL_KEYBOARD){

         if(remote_ctrl.key.set.G == 1 && KeyBoard_Info.Flag.G == 0){
		    
					if(Control_Info->Chassis_Situation == CHASSIS_BALANCE) Control_Info->Chassis_Situation = CHASSIS_WEAK;
					Control_Info->Init.Begin_Init =  !Control_Info->Init.Begin_Init;
					KeyBoard_Info.Flag.G = 1;
		     
		     }else if(remote_ctrl.key.set.G == 0) KeyBoard_Info.Flag.G = 0;      

	   }    
    
   }else{
      
		   Control_Info->Init.Begin_Init  = 0;
       Control_Info->Chassis_Situation = CHASSIS_WEAK;


   }

	  Check_Control_Slip(Control_Info);
	 
  if(Control_Info->Init.Begin_Init == 1 && Control_Info->Chassis_Situation == CHASSIS_WEAK){	
	
	                Control_Info->Slip.Slip_Cnt = 0;
               		Control_Info->Slip.Slip_Flag = 0;
                  Control_Info->Climb.Begin_Climb_Flag = 0;
	                Control_Info->Climb.Climb_Flag = 0;
		              Control_Info->Climb.Climb_Complete_Flag = 0;
	                Control_Info->Climb.Cnt = 0;
   if(Control_Info->Init.Joint_Init.IF_Joint_Init == 0){  
	   	
			 if(DM_8009_Motor[0].Data.Position > -0.64f )  
			 Control_Info->Init.Joint_Init.Joint_Reduction_Flag[0] = 1; else Control_Info->Init.Joint_Init.Joint_Reduction_Flag[0] = 0;
			 if(DM_8009_Motor[1].Data.Position < 0.64f )    
		   Control_Info->Init.Joint_Init.Joint_Reduction_Flag[1] = 1; else Control_Info->Init.Joint_Init.Joint_Reduction_Flag[1] = 0; 
			 if(DM_8009_Motor[2].Data.Position < 0.64f )    
			 Control_Info->Init.Joint_Init.Joint_Reduction_Flag[2] = 1; else Control_Info->Init.Joint_Init.Joint_Reduction_Flag[2] = 0;
			 if(DM_8009_Motor[3].Data.Position > -0.64f )  
			 Control_Info->Init.Joint_Init.Joint_Reduction_Flag[3] = 1; else Control_Info->Init.Joint_Init.Joint_Reduction_Flag[3] = 0;
			
			 if(Control_Info->Init.Joint_Init.Joint_Reduction_Flag[0] + Control_Info->Init.Joint_Init.Joint_Reduction_Flag[1] 
			  + Control_Info->Init.Joint_Init.Joint_Reduction_Flag[2] + Control_Info->Init.Joint_Init.Joint_Reduction_Flag[3] == 4)  
			
			Control_Info->Init.Joint_Init.IF_Joint_Init = 1;  else  Control_Info->Init.Joint_Init.IF_Joint_Init =0;

		}else if(Control_Info->Init.Joint_Init.IF_Joint_Init == 1){
			
			if(Control_Info->Init.Velocity_Init.IF_Velocity_Init == 0){
			
				if(fabsf((-GIM_8108_Motor[0].Data.Velocity + GIM_8108_Motor[1].Data.Velocity)/2.f) < 0.2f){
				
				  Control_Info->Init.Velocity_Init.Velocity_Cnt ++;
					
					if(Control_Info->Init.Velocity_Init.Velocity_Cnt == 200){
					
						 Control_Info->Init.Velocity_Init.IF_Velocity_Init = 1;
					    Control_Info->Init.Velocity_Init.Velocity_Cnt = 0;
					}
				  
				}
				
			}else if(Control_Info->Init.Velocity_Init.IF_Velocity_Init == 1){
				
   			if(Control_Info->Init.Yaw_Init.IF_Yaw_Init == 0){
				
				     if(fabsf(Control_Info->Yaw_Err)>1.f) Control_Info->Init.Yaw_Init.IF_Yaw_Init = 0;
				     else {
						 
						   Control_Info->Init.Yaw_Init.Yaw_Right_Cnt++;
							 if(Control_Info->Init.Yaw_Init.Yaw_Right_Cnt > 300 ){
							 
							   Control_Info->Init.Yaw_Init.IF_Yaw_Init = 1;
							   Control_Info->Init.Yaw_Init.Yaw_Right_Cnt = 0;
							 }
						 
						 }
				
				}else if(Control_Info->Init.Yaw_Init.IF_Yaw_Init == 1){
			
			         if(Control_Info->Init.Balance_Init.IF_Balance_Init == 0){
				
				         if(fabsf(INS_Info.Pitch_Angle)< 5.f) Control_Info->Init.Balance_Init.IF_Balance_Init = 1;
				
				      }else if(Control_Info->Init.Balance_Init.IF_Balance_Init == 1){
				  		   
 								Control_Info->Chassis_Situation = CHASSIS_BALANCE;

				 }
		   }
		 }	
	}
}





 if(Control_Info->Init.Begin_Init == 0 && Control_Info->Chassis_Situation == CHASSIS_WEAK){

  
  Control_Info->Init.Joint_Init.Joint_Reduction_Flag[0] = 0;
	Control_Info->Init.Joint_Init.Joint_Reduction_Flag[1] = 0;
	Control_Info->Init.Joint_Init.Joint_Reduction_Flag[2] = 0;
	Control_Info->Init.Joint_Init.Joint_Reduction_Flag[3] = 0;
  Control_Info->Init.Joint_Init.IF_Joint_Init = 0;
	Control_Info->Init.Balance_Init.IF_Balance_Init = 0;
	Control_Info->Init.Yaw_Init.IF_Yaw_Init = 0;
	Control_Info->Init.Yaw_Init.Yaw_Right_Cnt = 0;
 Control_Info->Init.Velocity_Init.IF_Velocity_Init = 0;
						    Control_Info->Init.Velocity_Init.Velocity_Cnt = 0;

}

}



static void Check_Control_Slip(Control_Info_Typedef *Control_Info){

    if(Control_Info->Chassis_Situation == CHASSIS_BALANCE){
		
		   if(fabsf(INS_Info.Pitch_Angle)>7.f) Control_Info->Slip.Slip_Cnt++;
			 
			 if(Control_Info->Slip.Slip_Cnt > 500) {
				 Control_Info->Slip.Slip_Cnt = 0;
				 Control_Info->Slip.Slip_Flag = 1;
			   Control_Info->Chassis_Situation = CHASSIS_WEAK;
			   Control_Info->Init.Begin_Init = 0 ;
			 }
	 }

   if(Control_Info->Slip.Slip_Flag == 1){
	  
	    Control_Info->Slip.Slip_Cnt++;
      Control_Info->Init.Begin_Init = 0 ;
      if(Control_Info->Slip.Slip_Cnt > 200){
			 
			    Control_Info->Slip.Slip_Flag = 0;
			    Control_Info->Slip.Slip_Cnt = 0;
			}  
	 
	 }

}


static void Joint_Angle_Zero_Offset(Control_Info_Typedef *Control_Info){

    Control_Info->L_Leg_Info.VMC.Phi1 = (3.141593f + 0.64f) + (DM_8009_Motor[0].Data.Position );
    Control_Info->L_Leg_Info.VMC.Phi4 = DM_8009_Motor[1].Data.Position - 0.64f;
    Control_Info->L_Leg_Info.VMC.Phi1_dot = DM_8009_Motor[0].Data.Velocity;
	  Control_Info->L_Leg_Info.VMC.Phi4_dot = DM_8009_Motor[1].Data.Velocity;
  if( fabsf(Control_Info->L_Leg_Info.VMC.Phi1_dot) < 0.04f)     Control_Info->L_Leg_Info.VMC.Phi1_dot = 0;
  if( fabsf(Control_Info->L_Leg_Info.VMC.Phi4_dot) < 0.04f)     Control_Info->L_Leg_Info.VMC.Phi4_dot = 0;
	
	
	  Control_Info->R_Leg_Info.VMC.Phi1 = (3.141593f + 0.64f) + (DM_8009_Motor[3].Data.Position );
    Control_Info->R_Leg_Info.VMC.Phi4 = DM_8009_Motor[2].Data.Position - 0.64f;
	  Control_Info->R_Leg_Info.VMC.Phi1_dot = DM_8009_Motor[3].Data.Velocity;
	  Control_Info->R_Leg_Info.VMC.Phi4_dot = DM_8009_Motor[2].Data.Velocity;
  if( fabsf(Control_Info->R_Leg_Info.VMC.Phi1_dot) < 0.04f)     Control_Info->R_Leg_Info.VMC.Phi1_dot = 0;
  if( fabsf(Control_Info->R_Leg_Info.VMC.Phi4_dot) < 0.04f)     Control_Info->R_Leg_Info.VMC.Phi4_dot = 0;
}


static void VMC_Calculate(Control_Info_Typedef *Control_Info){

Control_Info->L_Leg_Info.VMC.X_B = Control_Info->L_Leg_Info.VMC.L1 * arm_cos_f32(Control_Info->L_Leg_Info.VMC.Phi1);
Control_Info->L_Leg_Info.VMC.Y_B = Control_Info->L_Leg_Info.VMC.L1 * arm_sin_f32(Control_Info->L_Leg_Info.VMC.Phi1);
Control_Info->L_Leg_Info.VMC.X_D = Control_Info->L_Leg_Info.VMC.L5 + Control_Info->L_Leg_Info.VMC.L4 * arm_cos_f32(Control_Info->L_Leg_Info.VMC.Phi4);
Control_Info->L_Leg_Info.VMC.Y_D = Control_Info->L_Leg_Info.VMC.L4 * arm_sin_f32(Control_Info->L_Leg_Info.VMC.Phi4);
Control_Info->L_Leg_Info.VMC.X_D_Difference_X_B = Control_Info->L_Leg_Info.VMC.X_D - Control_Info->L_Leg_Info.VMC.X_B;
Control_Info->L_Leg_Info.VMC.Y_D_Difference_Y_B = Control_Info->L_Leg_Info.VMC.Y_D - Control_Info->L_Leg_Info.VMC.Y_B;
Control_Info->L_Leg_Info.VMC.LBD_2 = Control_Info->L_Leg_Info.VMC.X_D_Difference_X_B*Control_Info->L_Leg_Info.VMC.X_D_Difference_X_B +Control_Info->L_Leg_Info.VMC.Y_D_Difference_Y_B*Control_Info->L_Leg_Info.VMC.Y_D_Difference_Y_B;
Control_Info->L_Leg_Info.VMC.A0 =  2 *Control_Info->L_Leg_Info.VMC.L2  * Control_Info->L_Leg_Info.VMC.X_D_Difference_X_B;
Control_Info->L_Leg_Info.VMC.B0 =  2 *Control_Info->L_Leg_Info.VMC.L2  * Control_Info->L_Leg_Info.VMC.Y_D_Difference_Y_B;
arm_sqrt_f32(Control_Info->L_Leg_Info.VMC.A0*Control_Info->L_Leg_Info.VMC.A0 +  Control_Info->L_Leg_Info.VMC.B0* Control_Info->L_Leg_Info.VMC.B0 - Control_Info->L_Leg_Info.VMC.LBD_2*Control_Info->L_Leg_Info.VMC.LBD_2,&Control_Info->L_Leg_Info.VMC.C0);
Control_Info->L_Leg_Info.VMC.Phi2 =2 * atan2f(Control_Info->L_Leg_Info.VMC.B0+Control_Info->L_Leg_Info.VMC.C0 , Control_Info->L_Leg_Info.VMC.A0 + Control_Info->L_Leg_Info.VMC.LBD_2);
Control_Info->L_Leg_Info.VMC.X_C  =Control_Info->L_Leg_Info.VMC.X_B +Control_Info->L_Leg_Info.VMC.L2 * arm_cos_f32(Control_Info->L_Leg_Info.VMC.Phi2);
Control_Info->L_Leg_Info.VMC.Y_C  =Control_Info->L_Leg_Info.VMC.Y_B +Control_Info->L_Leg_Info.VMC.L2 * arm_sin_f32(Control_Info->L_Leg_Info.VMC.Phi2);
Control_Info->L_Leg_Info.VMC.Phi3 = atan2f(Control_Info->L_Leg_Info.VMC.Y_C-Control_Info->L_Leg_Info.VMC.Y_D,Control_Info->L_Leg_Info.VMC.X_C-Control_Info->L_Leg_Info.VMC.X_D);
arm_sqrt_f32(powf(Control_Info->L_Leg_Info.VMC.X_C -Control_Info->L_Leg_Info.VMC.L5/2,2) +Control_Info->L_Leg_Info.VMC.Y_C*Control_Info->L_Leg_Info.VMC.Y_C,&Control_Info->L_Leg_Info.VMC.L0);
Control_Info->L_Leg_Info.VMC.Phi0 = atan2f(Control_Info->L_Leg_Info.VMC.Y_C,Control_Info->L_Leg_Info.VMC.X_C - (Control_Info->L_Leg_Info.VMC.L5/2));

float Phi1_2,Phi3,Phi2,Sin_Phi2_3,Phi3_4;

Phi1_2 = 	Control_Info->L_Leg_Info.VMC.Phi1 - Control_Info->L_Leg_Info.VMC.Phi2;
  Phi3 =  Control_Info->L_Leg_Info.VMC.Phi3;
	Phi2 =  Control_Info->L_Leg_Info.VMC.Phi2;
Phi3_4 =  Control_Info->L_Leg_Info.VMC.Phi3 - Control_Info->L_Leg_Info.VMC.Phi4;

Sin_Phi2_3 = arm_sin_f32( Control_Info->L_Leg_Info.VMC.Phi2 - Control_Info->L_Leg_Info.VMC.Phi3);	
	
Control_Info->L_Leg_Info.VMC.X_C_dot = 	  (( 0.15f * arm_sin_f32(Phi1_2) * arm_sin_f32(Phi3)) / (Sin_Phi2_3)) * Control_Info->L_Leg_Info.VMC.Phi1_dot
                                       +  (( 0.15f * arm_sin_f32(Phi3_4) * arm_sin_f32(Phi2)) / (Sin_Phi2_3)) * Control_Info->L_Leg_Info.VMC.Phi4_dot;
Control_Info->L_Leg_Info.VMC.Y_C_dot = 	 -(( 0.15f * arm_sin_f32(Phi1_2) * arm_cos_f32(Phi3)) / (Sin_Phi2_3)) * Control_Info->L_Leg_Info.VMC.Phi1_dot
                                       + -(( 0.15f * arm_sin_f32(Phi3_4) * arm_cos_f32(Phi2)) / (Sin_Phi2_3)) * Control_Info->L_Leg_Info.VMC.Phi4_dot;	
																			 
Control_Info->L_Leg_Info.VMC.Phi0_dot = 	(Control_Info->L_Leg_Info.VMC.X_C_dot * arm_cos_f32(	1.5707965f - Control_Info->L_Leg_Info.VMC.Phi0)
	                                      + Control_Info->L_Leg_Info.VMC.Y_C_dot * arm_sin_f32(	1.5707965f - Control_Info->L_Leg_Info.VMC.Phi0))/Control_Info->L_Leg_Info.VMC.L0;

Control_Info->R_Leg_Info.VMC.X_B = Control_Info->R_Leg_Info.VMC.L1 * arm_cos_f32(Control_Info->R_Leg_Info.VMC.Phi1);
Control_Info->R_Leg_Info.VMC.Y_B = Control_Info->R_Leg_Info.VMC.L1 * arm_sin_f32(Control_Info->R_Leg_Info.VMC.Phi1);
Control_Info->R_Leg_Info.VMC.X_D = Control_Info->R_Leg_Info.VMC.L5 + Control_Info->R_Leg_Info.VMC.L4 * arm_cos_f32(Control_Info->R_Leg_Info.VMC.Phi4);
Control_Info->R_Leg_Info.VMC.Y_D = Control_Info->R_Leg_Info.VMC.L4 * arm_sin_f32(Control_Info->R_Leg_Info.VMC.Phi4);
Control_Info->R_Leg_Info.VMC.X_D_Difference_X_B = Control_Info->R_Leg_Info.VMC.X_D - Control_Info->R_Leg_Info.VMC.X_B;
Control_Info->R_Leg_Info.VMC.Y_D_Difference_Y_B = Control_Info->R_Leg_Info.VMC.Y_D - Control_Info->R_Leg_Info.VMC.Y_B;
Control_Info->R_Leg_Info.VMC.LBD_2 =   Control_Info->R_Leg_Info.VMC.X_D_Difference_X_B*Control_Info->R_Leg_Info.VMC.X_D_Difference_X_B +Control_Info->R_Leg_Info.VMC.Y_D_Difference_Y_B*Control_Info->R_Leg_Info.VMC.Y_D_Difference_Y_B;
Control_Info->R_Leg_Info.VMC.A0 =  2 *Control_Info->R_Leg_Info.VMC.L2  * Control_Info->R_Leg_Info.VMC.X_D_Difference_X_B;
Control_Info->R_Leg_Info.VMC.B0 =  2 *Control_Info->R_Leg_Info.VMC.L2  * Control_Info->R_Leg_Info.VMC.Y_D_Difference_Y_B;
arm_sqrt_f32(Control_Info->R_Leg_Info.VMC.A0*Control_Info->R_Leg_Info.VMC.A0 +  Control_Info->R_Leg_Info.VMC.B0* Control_Info->R_Leg_Info.VMC.B0 - Control_Info->R_Leg_Info.VMC.LBD_2*Control_Info->R_Leg_Info.VMC.LBD_2,&Control_Info->R_Leg_Info.VMC.C0);
Control_Info->R_Leg_Info.VMC.Phi2 =2 * atan2f(Control_Info->R_Leg_Info.VMC.B0+Control_Info->R_Leg_Info.VMC.C0 , Control_Info->R_Leg_Info.VMC.A0 + Control_Info->R_Leg_Info.VMC.LBD_2);
Control_Info->R_Leg_Info.VMC.X_C  =Control_Info->R_Leg_Info.VMC.X_B +Control_Info->R_Leg_Info.VMC.L2 * arm_cos_f32(Control_Info->R_Leg_Info.VMC.Phi2);
Control_Info->R_Leg_Info.VMC.Y_C  =Control_Info->R_Leg_Info.VMC.Y_B +Control_Info->R_Leg_Info.VMC.L2 * arm_sin_f32(Control_Info->R_Leg_Info.VMC.Phi2);
Control_Info->R_Leg_Info.VMC.Phi3 = atan2f(Control_Info->R_Leg_Info.VMC.Y_C-Control_Info->R_Leg_Info.VMC.Y_D,Control_Info->R_Leg_Info.VMC.X_C-Control_Info->R_Leg_Info.VMC.X_D);
arm_sqrt_f32(powf(Control_Info->R_Leg_Info.VMC.X_C -Control_Info->R_Leg_Info.VMC.L5/2,2) +Control_Info->R_Leg_Info.VMC.Y_C*Control_Info->R_Leg_Info.VMC.Y_C,&Control_Info->R_Leg_Info.VMC.L0);
Control_Info->R_Leg_Info.VMC.Phi0 = atan2f(Control_Info->R_Leg_Info.VMC.Y_C,Control_Info->R_Leg_Info.VMC.X_C - (Control_Info->R_Leg_Info.VMC.L5/2));

Phi1_2 = 	Control_Info->R_Leg_Info.VMC.Phi1 - Control_Info->R_Leg_Info.VMC.Phi2;
  Phi3 =  Control_Info->R_Leg_Info.VMC.Phi3;
  Phi2 =  Control_Info->R_Leg_Info.VMC.Phi2;
Phi3_4 =  Control_Info->R_Leg_Info.VMC.Phi3 - Control_Info->R_Leg_Info.VMC.Phi4;
Sin_Phi2_3 = arm_sin_f32( Control_Info->R_Leg_Info.VMC.Phi2 - Control_Info->R_Leg_Info.VMC.Phi3);	
	
Control_Info->R_Leg_Info.VMC.X_C_dot = 	 (( 0.15f * arm_sin_f32(Phi1_2) * arm_sin_f32(Phi3)) / (Sin_Phi2_3)) * Control_Info->R_Leg_Info.VMC.Phi1_dot
                                       + (( 0.15f * arm_sin_f32(Phi3_4) * arm_sin_f32(Phi2)) / (Sin_Phi2_3)) * Control_Info->R_Leg_Info.VMC.Phi4_dot;
	
Control_Info->R_Leg_Info.VMC.Y_C_dot = 	 -(( 0.15f * arm_sin_f32(Phi1_2) * arm_cos_f32(Phi3)) / (Sin_Phi2_3)) * Control_Info->R_Leg_Info.VMC.Phi1_dot
                                       + -(( 0.15f * arm_sin_f32(Phi3_4) * arm_cos_f32(Phi2)) / (Sin_Phi2_3)) * Control_Info->R_Leg_Info.VMC.Phi4_dot;	

Control_Info->R_Leg_Info.VMC.Phi0_dot = 	(Control_Info->R_Leg_Info.VMC.X_C_dot * arm_cos_f32(	 Control_Info->R_Leg_Info.VMC.Phi0 - 1.5707965f)
	                                       + Control_Info->R_Leg_Info.VMC.Y_C_dot * arm_sin_f32(	 Control_Info->R_Leg_Info.VMC.Phi0 - 1.5707965f))/Control_Info->R_Leg_Info.VMC.L0;

}

static void Control_LQR_K_Update(Control_Info_Typedef *Control_Info){
	
	float L0; 
	L0= Control_Info->L_Leg_Info.VMC.L0;
	
	Control_Info->L_Leg_Info.LQR_K[0][0] =  K11[1]*powf(L0,3)   + K11[2]*powf(L0,2)    +K11[3]*L0    +K11[4];      
	Control_Info->L_Leg_Info.LQR_K[0][1] =  K12[1]*powf(L0,3)   + K12[2]*powf(L0,2)    +K12[3]*L0    +K12[4];
	Control_Info->L_Leg_Info.LQR_K[0][2] =  K13[1]*powf(L0,3)   + K13[2]*powf(L0,2)    +K13[3]*L0    +K13[4];
	Control_Info->L_Leg_Info.LQR_K[0][3] =  K14[1]*powf(L0,3)   + K14[2]*powf(L0,2)    +K14[3]*L0    +K14[4];
	Control_Info->L_Leg_Info.LQR_K[0][4] =  K15[1]*powf(L0,3)   + K15[2]*powf(L0,2)    +K15[3]*L0    +K15[4];
	Control_Info->L_Leg_Info.LQR_K[0][5] =  K16[1]*powf(L0,3)   + K16[2]*powf(L0,2)    +K16[3]*L0    +K16[4];

	Control_Info->L_Leg_Info.LQR_K[1][0] =   K21[1]*powf(L0,3)   + K21[2]*powf(L0,2)    +K21[3]*L0    +K21[4];     
	Control_Info->L_Leg_Info.LQR_K[1][1] =   K22[1]*powf(L0,3)   + K22[2]*powf(L0,2)    +K22[3]*L0    +K22[4];
	Control_Info->L_Leg_Info.LQR_K[1][2] =   K23[1]*powf(L0,3)   + K23[2]*powf(L0,2)    +K23[3]*L0    +K23[4];
	Control_Info->L_Leg_Info.LQR_K[1][3] =   K24[1]*powf(L0,3)   + K24[2]*powf(L0,2)    +K24[3]*L0    +K24[4];
	Control_Info->L_Leg_Info.LQR_K[1][4] =   K25[1]*powf(L0,3)   + K25[2]*powf(L0,2)    +K25[3]*L0    +K25[4];
	Control_Info->L_Leg_Info.LQR_K[1][5] =   K26[1]*powf(L0,3)   + K26[2]*powf(L0,2)    +K26[3]*L0    +K26[4];
		
	
	float L1;
	L1= Control_Info->R_Leg_Info.VMC.L0;
	
	Control_Info->R_Leg_Info.LQR_K[0][0] =   K11[1]*powf(L1,3)   + K11[2]*powf(L1,2)    +K11[3]*L1    +K11[4];      
	Control_Info->R_Leg_Info.LQR_K[0][1] =   K12[1]*powf(L1,3)   + K12[2]*powf(L1,2)    +K12[3]*L1    +K12[4];
	Control_Info->R_Leg_Info.LQR_K[0][2] =   K13[1]*powf(L1,3)   + K13[2]*powf(L1,2)    +K13[3]*L1    +K13[4];
	Control_Info->R_Leg_Info.LQR_K[0][3] =   K14[1]*powf(L1,3)   + K14[2]*powf(L1,2)    +K14[3]*L1    +K14[4];
	Control_Info->R_Leg_Info.LQR_K[0][4] =   K15[1]*powf(L1,3)   + K15[2]*powf(L1,2)    +K15[3]*L1    +K15[4];
	Control_Info->R_Leg_Info.LQR_K[0][5] =   K16[1]*powf(L1,3)   + K16[2]*powf(L1,2)    +K16[3]*L1    +K16[4];

	Control_Info->R_Leg_Info.LQR_K[1][0] =   K21[1]*powf(L1,3)   + K21[2]*powf(L1,2)    +K21[3]*L1    +K21[4];     
	Control_Info->R_Leg_Info.LQR_K[1][1] =   K22[1]*powf(L1,3)   + K22[2]*powf(L1,2)    +K22[3]*L1    +K22[4];
	Control_Info->R_Leg_Info.LQR_K[1][2] =   K23[1]*powf(L1,3)   + K23[2]*powf(L1,2)    +K23[3]*L1    +K23[4];
	Control_Info->R_Leg_Info.LQR_K[1][3] =   K24[1]*powf(L1,3)   + K24[2]*powf(L1,2)    +K24[3]*L1    +K24[4];
	Control_Info->R_Leg_Info.LQR_K[1][4] =   K25[1]*powf(L1,3)   + K25[2]*powf(L1,2)    +K25[3]*L1    +K25[4];
	Control_Info->R_Leg_Info.LQR_K[1][5] =   K26[1]*powf(L1,3)   + K26[2]*powf(L1,2)    +K26[3]*L1    +K26[4];

}

static void Control_Target_Update(Control_Info_Typedef *Control_Info){

	
	if(Control_Info->Control_Mode == CONTROL_REMOTE){
	
	  Control_Info->Target_Velocity =f_Ramp_Calc(Control_Info->Target_Velocity,remote_ctrl.rc.ch[3]*0.00303f,0.0015f);

		
	}else if(Control_Info->Control_Mode == CONTROL_KEYBOARD){
	
	  Control_Info->Target_Velocity = f_Ramp_Calc(Control_Info->Target_Velocity,(remote_ctrl.key.set.W - remote_ctrl.key.set.S )*2.f,0.0015f);
		
		if(remote_ctrl.key.set.X == 1 && KeyBoard_Info.Flag.X == 0){
		
			if(Control_Info->Leg_Length_Mode == LEG_LENGTH_NORMAL) Control_Info->Leg_Length_Mode = LEG_LENGTH_HIGH;
			else if(Control_Info->Leg_Length_Mode == LEG_LENGTH_HIGH) Control_Info->Leg_Length_Mode = LEG_LENGTH_NORMAL;
			KeyBoard_Info.Flag.X = 1;
		 
		}else if(remote_ctrl.key.set.X == 0) KeyBoard_Info.Flag.X = 0;      
		
		if(Control_Info->Leg_Length_Mode == LEG_LENGTH_NORMAL){
			
      Control_Info->L_Leg_Info.VMC.Target_L0 = 0.165f;
      Control_Info->R_Leg_Info.VMC.Target_L0 = 0.165f;
			
		}else if(Control_Info->Leg_Length_Mode == LEG_LENGTH_HIGH){
		Control_Info->L_Leg_Info.VMC.Target_L0 = 0.38f;
      Control_Info->R_Leg_Info.VMC.Target_L0 = 0.38f;
		
		}
		
		
	}
	
	if( Control_Info->Climb.Begin_Climb_Flag == 1)  VAL_LIMIT(Control_Info->Target_Velocity,-1.2f,1.2f);
	else VAL_LIMIT( Control_Info->Target_Velocity,-1.8f,1.8f);
	 
	  if( Control_Info->Target_Velocity != 0){
		
		  	Control_Info->L_Leg_Info.Measure.Chassis_Position = 0;
		   	Control_Info->R_Leg_Info.Measure.Chassis_Position = 0;
		
		
		}
 
		
	
		
		


}




static void Control_Measure_Update(Control_Info_Typedef *Control_Info){

	Control_Info->L_Leg_Info.Measure.Phi = -INS_Info.Angle[2] - 0.0f;
	Control_Info->L_Leg_Info.Measure.Phi_dot  = -INS_Info.Gyro[0];
	Control_Info->L_Leg_Info.Measure.Theta =  ( 1.5707965f - Control_Info->L_Leg_Info.VMC.Phi0) - Control_Info->L_Leg_Info.Measure.Phi -0.02f ;
	Control_Info->L_Leg_Info.Measure.Theta_dot =( Control_Info->L_Leg_Info.Measure.Theta -  Control_Info->L_Leg_Info.Measure.Last_Theta)/0.001f; //+ INS_Info.gyro[2];
	Control_Info->L_Leg_Info.Measure.Last_Theta =  Control_Info->L_Leg_Info.Measure.Theta;
	
	Control_Info->R_Leg_Info.Measure.Phi = -INS_Info.Angle[2] -  0.0f;
	Control_Info->R_Leg_Info.Measure.Phi_dot  = -INS_Info.Gyro[0];
	Control_Info->R_Leg_Info.Measure.Theta =  (Control_Info->R_Leg_Info.VMC.Phi0 - 1.5707965f )  - Control_Info->R_Leg_Info.Measure.Phi ;
	Control_Info->R_Leg_Info.Measure.Theta_dot = ( Control_Info->R_Leg_Info.Measure.Theta -  Control_Info->R_Leg_Info.Measure.Last_Theta)/0.001f; //+ INS_Info.gyro[2];
	Control_Info->R_Leg_Info.Measure.Last_Theta =  Control_Info->R_Leg_Info.Measure.Theta;

if(Control_Info->Chassis_Situation  == CHASSIS_BALANCE){	

  Control_Info->L_Leg_Info.W_Velocity  =   ( -GIM_8108_Motor[0].Data.Velocity - Control_Info->L_Leg_Info.VMC.Phi0_dot - INS_Info.Gyro[0]) *0.08f
	                                         + ( Control_Info->L_Leg_Info.VMC.L0 * Control_Info->L_Leg_Info.Measure.Theta_dot * arm_cos_f32(Control_Info->L_Leg_Info.Measure.Theta)) ;
   
	Control_Info->L_Leg_Info.Predict_Velocity = Control_Info->L_Leg_Info.Fusion_Velocity + Control_Info->Accel*0.001f;
	
	Control_Info->L_Leg_Info.Fusion_Velocity =  0.8f*Control_Info->L_Leg_Info.Predict_Velocity +0.2f*Control_Info->L_Leg_Info.W_Velocity;
	
	Control_Info->L_Leg_Info.Measure.Chassis_Velocity = Control_Info->L_Leg_Info.Fusion_Velocity;
	
	
	Control_Info->R_Leg_Info.W_Velocity =   ( GIM_8108_Motor[1].Data.Velocity  +  Control_Info->R_Leg_Info.VMC.Phi0_dot - INS_Info.Gyro[0] ) *0.08f
	                                         + ( Control_Info->R_Leg_Info.VMC.L0 * Control_Info->R_Leg_Info.Measure.Theta_dot * arm_cos_f32(Control_Info->R_Leg_Info.Measure.Theta));
	
	Control_Info->R_Leg_Info.Predict_Velocity = Control_Info->R_Leg_Info.Fusion_Velocity + Control_Info->Accel*0.001f;
	
	Control_Info->R_Leg_Info.Fusion_Velocity =  0.8f*Control_Info->R_Leg_Info.Predict_Velocity +0.2f*Control_Info->R_Leg_Info.W_Velocity;
	
	Control_Info->R_Leg_Info.Measure.Chassis_Velocity = Control_Info->R_Leg_Info.Fusion_Velocity;	
		
	Control_Info->Chassis_Velocity =   (Control_Info->L_Leg_Info.Measure.Chassis_Velocity +  Control_Info->R_Leg_Info.Measure.Chassis_Velocity)/2.f * 0.909f;
	
  Control_Info->L_Leg_Info.Measure.Chassis_Position +=  Control_Info->Chassis_Velocity*0.001f ;

 	Control_Info->R_Leg_Info.Measure.Chassis_Position +=  Control_Info->Chassis_Velocity*0.001f ;
	 


}else{

  Control_Info->L_Leg_Info.Measure.Chassis_Velocity = 0;
	Control_Info->L_Leg_Info.Measure.Chassis_Position +=  0;
	Control_Info->Chassis_Velocity = 0;
	Control_Info->R_Leg_Info.Measure.Chassis_Velocity = 0;
 	Control_Info->R_Leg_Info.Measure.Chassis_Position +=  0;


}	
	

Control_Info->Accel =  (float) (( -INS_Info.Accel[1] - powf(INS_Info.Gyro[2],2)*0.161f) - GravityAccel* arm_sin_f32 (-INS_Info.Angle[2])) * arm_cos_f32 (-INS_Info.Angle[2]) + 
	                              ( INS_Info.Accel[2] - GravityAccel* arm_cos_f32 (-INS_Info.Angle[2])) * arm_sin_f32 (-INS_Info.Angle[2]) ; 


}

static void Control_LQR_X_Update(Control_Info_Typedef *Control_Info){

Control_Info->L_Leg_Info.LQR_X[0] = (Control_Info->L_Leg_Info.Target.Theta - Control_Info->L_Leg_Info.Measure.Theta);
Control_Info->L_Leg_Info.LQR_X[1] = (Control_Info->L_Leg_Info.Target.Theta_dot - Control_Info->L_Leg_Info.Measure.Theta_dot);
Control_Info->L_Leg_Info.LQR_X[2] = (Control_Info->L_Leg_Info.Target.Chassis_Position  - Control_Info->L_Leg_Info.Measure.Chassis_Position) ;
Control_Info->L_Leg_Info.LQR_X[3] = (Control_Info->Target_Velocity -   Control_Info->Chassis_Velocity );
Control_Info->L_Leg_Info.LQR_X[4] = (Control_Info->L_Leg_Info.Target.Phi - Control_Info->L_Leg_Info.Measure.Phi);
Control_Info->L_Leg_Info.LQR_X[5] = (Control_Info->L_Leg_Info.Target.Phi_dot - Control_Info->L_Leg_Info.Measure.Phi_dot);

Control_Info->R_Leg_Info.LQR_X[0] = (Control_Info->R_Leg_Info.Target.Theta - Control_Info->R_Leg_Info.Measure.Theta);
Control_Info->R_Leg_Info.LQR_X[1] = (Control_Info->R_Leg_Info.Target.Theta_dot - Control_Info->R_Leg_Info.Measure.Theta_dot);
Control_Info->R_Leg_Info.LQR_X[2] = (Control_Info->R_Leg_Info.Target.Chassis_Position  - Control_Info->R_Leg_Info.Measure.Chassis_Position) ;
Control_Info->R_Leg_Info.LQR_X[3] = (Control_Info->Target_Velocity  -    Control_Info->Chassis_Velocity );
Control_Info->R_Leg_Info.LQR_X[4] = (Control_Info->R_Leg_Info.Target.Phi - Control_Info->R_Leg_Info.Measure.Phi);
Control_Info->R_Leg_Info.LQR_X[5] = (Control_Info->R_Leg_Info.Target.Phi_dot - Control_Info->R_Leg_Info.Measure.Phi_dot);

}
static void VMC_Measure_F_Tp_Calculate(Control_Info_Typedef *Control_Info){

    Control_Info->L_Leg_Info.Measure.F = (((DM_8009_Motor[1].Data.Torque * arm_cos_f32(( Control_Info->L_Leg_Info.VMC.Phi0 -  Control_Info->L_Leg_Info.VMC.Phi3)))/
                                          (0.15f *  arm_sin_f32 (( Control_Info->L_Leg_Info.VMC.Phi3 - (Control_Info->L_Leg_Info.VMC.Phi4)))))      -
	                                      ((DM_8009_Motor[0].Data.Torque * arm_cos_f32(( Control_Info->L_Leg_Info.VMC.Phi0 -  Control_Info->L_Leg_Info.VMC.Phi2)))/
                                          (0.15f * arm_sin_f32 ((Control_Info->L_Leg_Info.VMC.Phi1 - (Control_Info->L_Leg_Info.VMC.Phi2))))));

    Control_Info->L_Leg_Info.Measure.Tp= Control_Info->L_Leg_Info.VMC.L0*(((DM_8009_Motor[0].Data.Torque * arm_sin_f32(( Control_Info->L_Leg_Info.VMC.Phi0 -  Control_Info->L_Leg_Info.VMC.Phi2)))/
                                         (0.15f *  arm_sin_f32 (( Control_Info->L_Leg_Info.VMC.Phi1 - (Control_Info->L_Leg_Info.VMC.Phi2)))))      -
	                                     ((DM_8009_Motor[1].Data.Torque * arm_sin_f32(( Control_Info->L_Leg_Info.VMC.Phi0 -  Control_Info->L_Leg_Info.VMC.Phi3)))/
                                         (0.15f * arm_sin_f32 ((Control_Info->L_Leg_Info.VMC.Phi3 - (Control_Info->L_Leg_Info.VMC.Phi4))))));

	 Control_Info->R_Leg_Info.Measure.F = (((DM_8009_Motor[2].Data.Torque * arm_cos_f32(( Control_Info->R_Leg_Info.VMC.Phi0 -  Control_Info->R_Leg_Info.VMC.Phi3)))/
                                         (0.15f *  arm_sin_f32 (( Control_Info->R_Leg_Info.VMC.Phi3 - (Control_Info->R_Leg_Info.VMC.Phi4)))))      -
	                                     ((DM_8009_Motor[3].Data.Torque * arm_cos_f32(( Control_Info->R_Leg_Info.VMC.Phi0 -  Control_Info->R_Leg_Info.VMC.Phi2)))/
                                         (0.15f * arm_sin_f32 ((Control_Info->R_Leg_Info.VMC.Phi1 - (Control_Info->R_Leg_Info.VMC.Phi2))))));

    Control_Info->R_Leg_Info.Measure.Tp= Control_Info->R_Leg_Info.VMC.L0*(((DM_8009_Motor[3].Data.Torque * arm_sin_f32(( Control_Info->R_Leg_Info.VMC.Phi0 -  Control_Info->R_Leg_Info.VMC.Phi2)))/
                                         (0.15f *  arm_sin_f32 (( Control_Info->R_Leg_Info.VMC.Phi1 - (Control_Info->R_Leg_Info.VMC.Phi2)))))      -
	                                    ((DM_8009_Motor[2].Data.Torque * arm_sin_f32(( Control_Info->R_Leg_Info.VMC.Phi0 -  Control_Info->R_Leg_Info.VMC.Phi3)))/
                                         (0.15f * arm_sin_f32 ((Control_Info->R_Leg_Info.VMC.Phi3 - (Control_Info->R_Leg_Info.VMC.Phi4))))));
    if(Control_Info->Chassis_Situation == CHASSIS_BALANCE){
	
	Control_Info->L_Leg_Info.Support.P =  Control_Info->L_Leg_Info.Measure.F * arm_cos_f32(Control_Info->L_Leg_Info.Measure.Theta)
										+(-(Control_Info->L_Leg_Info.Measure.Tp * arm_sin_f32(Control_Info->L_Leg_Info.Measure.Theta))/Control_Info->L_Leg_Info.VMC.L0);
	
											
	Control_Info->L_Leg_Info.Support.FN =    Control_Info->L_Leg_Info.Support.P + 3.8f*GravityAccel;

	Control_Info->R_Leg_Info.Support.P =  Control_Info->R_Leg_Info.Measure.F*arm_cos_f32(Control_Info->R_Leg_Info.Measure.Theta)
										+((Control_Info->R_Leg_Info.Measure.Tp*arm_sin_f32(Control_Info->R_Leg_Info.Measure.Theta))/Control_Info->R_Leg_Info.VMC.L0);
							
	Control_Info->R_Leg_Info.Support.FN =    Control_Info->R_Leg_Info.Support.P + 3.8f*GravityAccel ;
		
		
		if(Control_Info->L_Leg_Info.Support.FN < 50.f) Control_Info->L_Leg_Info.Support.Flag = 1;
	else 	Control_Info->L_Leg_Info.Support.Flag = 0;
		if(Control_Info->R_Leg_Info.Support.FN < 50.f) Control_Info->R_Leg_Info.Support.Flag = 1;
	else 	Control_Info->R_Leg_Info.Support.Flag = 0;
		
	}else{
		Control_Info->L_Leg_Info.Support.Flag = 0;
  Control_Info->R_Leg_Info.Support.Flag = 0;
	Control_Info->L_Leg_Info.Support.FN =100.f;
	Control_Info->R_Leg_Info.Support.FN =100.f;

	}
}

static void Climb_20cm(Control_Info_Typedef *Control_Info){

 if(Control_Info->Chassis_Situation == CHASSIS_BALANCE){
   
    if(Control_Info->Control_Mode == CONTROL_KEYBOARD){

			   if(remote_ctrl.key.set.C == 1 && KeyBoard_Info.Flag.C == 0){
		
			       Control_Info->Climb.Begin_Climb_Flag = !Control_Info->Climb.Begin_Climb_Flag;
					   Control_Info->Climb.Climb_Complete_Flag = 0;
			       KeyBoard_Info.Flag.C = 1;
		 
		      }else if(remote_ctrl.key.set.C == 0) KeyBoard_Info.Flag.C = 0;
			     
			 
		
		}
 
		if( Control_Info->Climb.Begin_Climb_Flag == 1 && Control_Info->Climb.Climb_Flag == 0){
		
		  Control_Info->L_Leg_Info.VMC.Target_L0 = 0.38f;
      Control_Info->R_Leg_Info.VMC.Target_L0 = 0.38f;
			
			if(fabsf(INS_Info.Pitch_Angle) > 10.f) Control_Info->Climb.Climb_Flag = 1;
		
		}
		
		if(Control_Info->Climb.Climb_Flag == 1 &&  Control_Info->Climb.Climb_Complete_Flag == 0){
		
		   if(Control_Info->L_Leg_Info.VMC.L0 <= 0.12f && Control_Info->R_Leg_Info.VMC.L0 <= 0.12f){
			 
          Control_Info->Climb.Cnt++;			 	 
			    if(Control_Info->Climb.Cnt > 300){
					  Control_Info->Climb.Climb_Complete_Flag = 1;
						Control_Info->Climb.Cnt = 0;
					
					}
			 }
		
		}
		
		
		
		if( Control_Info->Climb.Climb_Complete_Flag == 1){
			
		  Control_Info->Climb.Climb_Flag = 0;
		  Control_Info->Climb.Climb_Complete_Flag = 0;

			
		}
		
		
 
 
 }




}	

static void Jump(Control_Info_Typedef *Control_Info){

   if(remote_ctrl.rc.s[1] == 1){
	 
	    Control_Info->Jump.Chassis_Jump = 1;
		 
	 
	 }else if(remote_ctrl.rc.s[1] != 1){
	 
	  	  Control_Info->Jump.Chassis_Jump = 0;
       	Control_Info->Jump.Chassis_Jump_Complete = 0;
         Control_Info->Jump.Jump_Cnt1 = 0;
	      Control_Info->Jump.Jump_Cnt2 = 0;
		    Control_Info->Jump.Jump_Cnt3 = 0;
		    Control_Info->Jump.Flag2 = 0;
				Control_Info->Jump.Flag1 = 0; 
	 }
   
	 
	 
  if( Control_Info->Jump.Chassis_Jump == 1 && 	Control_Info->Jump.Chassis_Jump_Complete == 0){
	  
      if(Control_Info->Jump.Jump_Cnt1 < 200){
			 Control_Info->Jump.Jump_Cnt1++;
			 Control_Info->L_Leg_Info.VMC.Target_L0 = 0.08f;
		   Control_Info->R_Leg_Info.VMC.Target_L0 = 0.08f;
//       PID_Leg_length_thrust[0].param.kp = 2000;
//       PID_Leg_length_thrust[1].param.kp = 2000;

			}else if(Control_Info->Jump.Jump_Cnt1 == 200){
				
			 Control_Info->L_Leg_Info.Target.Phi =  20.f*0.0174532944f;
			 Control_Info->R_Leg_Info.Target.Phi = 20.f*0.0174532944f;
			 Control_Info->L_Leg_Info.VMC.Target_L0 = 0.4f;
		   Control_Info->R_Leg_Info.VMC.Target_L0 = 0.4f;
//       PID_Leg_length_thrust[0].param.kp = 20000;
//       PID_Leg_length_thrust[1].param.kp = 20000;
			
			 if( Control_Info->L_Leg_Info.VMC.L0 >= 0.36f && Control_Info->R_Leg_Info.VMC.L0 >= 0.36f)  Control_Info->Jump.Flag1 = 1;
			  
				if(Control_Info->Jump.Flag1 == 1){

					if(Control_Info->L_Leg_Info.VMC.L0 <= 0.12f && Control_Info->R_Leg_Info.VMC.L0 <= 0.12f) Control_Info->Jump.Flag2 = 1;
					
				  if(Control_Info->Jump.Flag2 == 1){
					
						 Control_Info->L_Leg_Info.Target.Phi =  20.f*0.0174532944f;
						 Control_Info->R_Leg_Info.Target.Phi = 20.f*0.0174532944f;
						 Control_Info->L_Leg_Info.VMC.Target_L0 = 0.16f;
						 Control_Info->R_Leg_Info.VMC.Target_L0 = 0.16f;
//						 PID_Leg_length_thrust[0].param.kp = 20000;
//						 PID_Leg_length_thrust[1].param.kp = 20000;
					
				   if( Control_Info->L_Leg_Info.VMC.L0 >= 0.14f && Control_Info->R_Leg_Info.VMC.L0 >= 0.14f)  Control_Info->Jump.Flag3 = 1;
	
					if(Control_Info->Jump.Flag3 == 1){
							Control_Info->L_Leg_Info.Support.Flag = 1;
							Control_Info->R_Leg_Info.Support.Flag = 1;
							Control_Info->Jump.Flag2 = 0;
							Control_Info->Jump.Flag1 = 0; 
							Control_Info->Jump.Chassis_Jump = 0;
							Control_Info->Jump.Chassis_Jump_Complete = 1;
							 Control_Info->Jump.Jump_Cnt1 = 0;
							Control_Info->Jump.Jump_Cnt2 = 0;
							Control_Info->Jump.Jump_Cnt3 = 0;
				 }
						
				}
				
				
			   }
		}
				
	}
	
	
}







static void Control_LQR_Stand_T_Balance_Tp_Calculate(Control_Info_Typedef *Control_Info){

	if(Control_Info->L_Leg_Info.Support.Flag == 1){
	
		Control_Info->L_Leg_Info.LQR_K[0][0] = 0;
		Control_Info->L_Leg_Info.LQR_K[0][1] = 0; 
		Control_Info->L_Leg_Info.LQR_K[0][2] = 0;
		Control_Info->L_Leg_Info.LQR_K[0][3] = 0; 
		Control_Info->L_Leg_Info.LQR_K[0][4] = 0;
		Control_Info->L_Leg_Info.LQR_K[0][5] = 0; 

		Control_Info->L_Leg_Info.LQR_K[1][2] = 0;  
		Control_Info->L_Leg_Info.LQR_K[1][3] = 0;  
		Control_Info->L_Leg_Info.LQR_K[1][4] = 0;  
		Control_Info->L_Leg_Info.LQR_K[1][5] = 0; 
	
	
	}
	
	if(Control_Info->R_Leg_Info.Support.Flag == 1){
	
		Control_Info->R_Leg_Info.LQR_K[0][0] = 0;
		Control_Info->R_Leg_Info.LQR_K[0][1] = 0; 
		Control_Info->R_Leg_Info.LQR_K[0][2] = 0;
		Control_Info->R_Leg_Info.LQR_K[0][3] = 0; 
		Control_Info->R_Leg_Info.LQR_K[0][4] = 0;
		Control_Info->R_Leg_Info.LQR_K[0][5] = 0; 

		Control_Info->R_Leg_Info.LQR_K[1][2] = 0;  
		Control_Info->R_Leg_Info.LQR_K[1][3] = 0;  
		Control_Info->R_Leg_Info.LQR_K[1][4] = 0;  
		Control_Info->R_Leg_Info.LQR_K[1][5] = 0; 
	
	
	}
	
	
	Control_Info->L_Leg_Info.LQR_Output[0][0] = Control_Info->L_Leg_Info.LQR_X[0]*Control_Info->L_Leg_Info.LQR_K[0][0];
	Control_Info->L_Leg_Info.LQR_Output[0][1] = Control_Info->L_Leg_Info.LQR_X[1]*Control_Info->L_Leg_Info.LQR_K[0][1];
	Control_Info->L_Leg_Info.LQR_Output[0][2] = Control_Info->L_Leg_Info.LQR_X[2]*Control_Info->L_Leg_Info.LQR_K[0][2];
	Control_Info->L_Leg_Info.LQR_Output[0][3] = Control_Info->L_Leg_Info.LQR_X[3]*Control_Info->L_Leg_Info.LQR_K[0][3];
	Control_Info->L_Leg_Info.LQR_Output[0][4] = Control_Info->L_Leg_Info.LQR_X[4]*Control_Info->L_Leg_Info.LQR_K[0][4];
	Control_Info->L_Leg_Info.LQR_Output[0][5] = Control_Info->L_Leg_Info.LQR_X[5]*Control_Info->L_Leg_Info.LQR_K[0][5];

    Control_Info->L_Leg_Info.Moment.Stand_T =    Control_Info->L_Leg_Info.LQR_Output[0][0] + Control_Info->L_Leg_Info.LQR_Output[0][1] + Control_Info->L_Leg_Info.LQR_Output[0][2]
                                               + Control_Info->L_Leg_Info.LQR_Output[0][3] + Control_Info->L_Leg_Info.LQR_Output[0][4] + Control_Info->L_Leg_Info.LQR_Output[0][5];

	VAL_LIMIT(Control_Info->L_Leg_Info.Moment.Stand_T,-12.5f,12.5f);	
	
	Control_Info->R_Leg_Info.LQR_Output[0][0] = Control_Info->R_Leg_Info.LQR_X[0]*Control_Info->R_Leg_Info.LQR_K[0][0];
	Control_Info->R_Leg_Info.LQR_Output[0][1] = Control_Info->R_Leg_Info.LQR_X[1]*Control_Info->R_Leg_Info.LQR_K[0][1];
	Control_Info->R_Leg_Info.LQR_Output[0][2] = Control_Info->R_Leg_Info.LQR_X[2]*Control_Info->R_Leg_Info.LQR_K[0][2];
	Control_Info->R_Leg_Info.LQR_Output[0][3] = Control_Info->R_Leg_Info.LQR_X[3]*Control_Info->R_Leg_Info.LQR_K[0][3];
	Control_Info->R_Leg_Info.LQR_Output[0][4] = Control_Info->R_Leg_Info.LQR_X[4]*Control_Info->R_Leg_Info.LQR_K[0][4];
	Control_Info->R_Leg_Info.LQR_Output[0][5] = Control_Info->R_Leg_Info.LQR_X[5]*Control_Info->R_Leg_Info.LQR_K[0][5];	
	

	Control_Info->R_Leg_Info.Moment.Stand_T =   Control_Info->R_Leg_Info.LQR_Output[0][0] + Control_Info->R_Leg_Info.LQR_Output[0][1] + Control_Info->R_Leg_Info.LQR_Output[0][2]
                                            + Control_Info->R_Leg_Info.LQR_Output[0][3] + Control_Info->R_Leg_Info.LQR_Output[0][4] + Control_Info->R_Leg_Info.LQR_Output[0][5];

  VAL_LIMIT(Control_Info->R_Leg_Info.Moment.Stand_T,-12.5f,12.5f);	

	Control_Info->L_Leg_Info.LQR_Output[1][0] =  Control_Info->L_Leg_Info.LQR_X[0]*Control_Info->L_Leg_Info.LQR_K[1][0];
	Control_Info->L_Leg_Info.LQR_Output[1][1] =  Control_Info->L_Leg_Info.LQR_X[1]*Control_Info->L_Leg_Info.LQR_K[1][1];
	Control_Info->L_Leg_Info.LQR_Output[1][2] =  Control_Info->L_Leg_Info.LQR_X[2]*Control_Info->L_Leg_Info.LQR_K[1][2];
	Control_Info->L_Leg_Info.LQR_Output[1][3] =  Control_Info->L_Leg_Info.LQR_X[3]*Control_Info->L_Leg_Info.LQR_K[1][3];
	Control_Info->L_Leg_Info.LQR_Output[1][4] =  Control_Info->L_Leg_Info.LQR_X[4]*Control_Info->L_Leg_Info.LQR_K[1][4];
	Control_Info->L_Leg_Info.LQR_Output[1][5] =  Control_Info->L_Leg_Info.LQR_X[5]*Control_Info->L_Leg_Info.LQR_K[1][5];

    Control_Info->L_Leg_Info.Moment.Balance_Tp =   Control_Info->L_Leg_Info.LQR_Output[1][0] + Control_Info->L_Leg_Info.LQR_Output[1][1] + Control_Info->L_Leg_Info.LQR_Output[1][2]
                                                 + Control_Info->L_Leg_Info.LQR_Output[1][3] + Control_Info->L_Leg_Info.LQR_Output[1][4] + Control_Info->L_Leg_Info.LQR_Output[1][5];
 
    VAL_LIMIT( Control_Info->L_Leg_Info.Moment.Balance_Tp,-60.f,60.f);	
	
	Control_Info->R_Leg_Info.LQR_Output[1][0] = Control_Info->R_Leg_Info.LQR_X[0]*Control_Info->R_Leg_Info.LQR_K[1][0];
	Control_Info->R_Leg_Info.LQR_Output[1][1] = Control_Info->R_Leg_Info.LQR_X[1]*Control_Info->R_Leg_Info.LQR_K[1][1];
	Control_Info->R_Leg_Info.LQR_Output[1][2] = Control_Info->R_Leg_Info.LQR_X[2]*Control_Info->R_Leg_Info.LQR_K[1][2];
	Control_Info->R_Leg_Info.LQR_Output[1][3] = Control_Info->R_Leg_Info.LQR_X[3]*Control_Info->R_Leg_Info.LQR_K[1][3];
	Control_Info->R_Leg_Info.LQR_Output[1][4] = Control_Info->R_Leg_Info.LQR_X[4]*Control_Info->R_Leg_Info.LQR_K[1][4];
	Control_Info->R_Leg_Info.LQR_Output[1][5] = Control_Info->R_Leg_Info.LQR_X[5]*Control_Info->R_Leg_Info.LQR_K[1][5];	
	
	Control_Info->R_Leg_Info.Moment.Balance_Tp =   Control_Info->R_Leg_Info.LQR_Output[1][0] + Control_Info->R_Leg_Info.LQR_Output[1][1] + Control_Info->R_Leg_Info.LQR_Output[1][2]
                                               + Control_Info->R_Leg_Info.LQR_Output[1][3] + Control_Info->R_Leg_Info.LQR_Output[1][4] + Control_Info->R_Leg_Info.LQR_Output[1][5];
  
	VAL_LIMIT( Control_Info->R_Leg_Info.Moment.Balance_Tp,-60.f,60.f);	


}	
	
static void Control_Comprehensive_F_Calculate(Control_Info_Typedef *Control_Info){

	
	  Control_Info->Yaw.Angle = DM_Yaw_Motor.Data.Position;
	  Control_Info->Yaw.Err = 0 - DM_Yaw_Motor.Data.Position;
		
	  if( Control_Info->Yaw.Err >= 3.141593f)   Control_Info->Yaw.Err   -= 3.141593f * 2.f;
		if( Control_Info->Yaw.Err <= -3.141593f)  Control_Info->Yaw.Err   += 3.141593f * 2.f;

	    PID_Calculate(&PID_Yaw[0],0,Control_Info->Yaw.Err * Rad_to_angle);
     	PID_Calculate(&PID_Yaw[1],PID_Yaw[0].Output,INS_Info.Gyro[2]);
	    
	    Control_Info->L_Leg_Info.Moment.Turn_T =   PID_Yaw[1].Output;
	   	Control_Info->R_Leg_Info.Moment.Turn_T =  - PID_Yaw[1].Output;



        Control_Info->L_Leg_Info.Moment.Leg_Length_F =  PID_Calculate(&PID_Leg_length_thrust[0],Control_Info->L_Leg_Info.VMC.Target_L0,Control_Info->L_Leg_Info.VMC.L0);
        Control_Info->R_Leg_Info.Moment.Leg_Length_F =  PID_Calculate(&PID_Leg_length_thrust[1],Control_Info->R_Leg_Info.VMC.Target_L0,Control_Info->R_Leg_Info.VMC.L0);
     
        Control_Info->L_Leg_Info.Moment.Roll_F = - INS_Info.Roll_Angle * 30.f; 
        Control_Info->R_Leg_Info.Moment.Roll_F =   INS_Info.Roll_Angle * 30.f; 


        PID_Calculate(&PID_Leg_Coordinate,0,Control_Info->L_Leg_Info.Measure.Theta - Control_Info->R_Leg_Info.Measure.Theta);
     	  			
		Control_Info->L_Leg_Info.Moment.Leg_Coordinate_Tp = -PID_Leg_Coordinate.Output;
		Control_Info->R_Leg_Info.Moment.Leg_Coordinate_Tp = -PID_Leg_Coordinate.Output;

        Control_Info->L_Leg_Info.VMC.Gravity_Compensation = 100.f;
        Control_Info->R_Leg_Info.VMC.Gravity_Compensation = 100.f;

        if(Control_Info->L_Leg_Info.Support.Flag == 1){

		    Control_Info->L_Leg_Info.Moment.Turn_T  = 0.f;
            Control_Info->L_Leg_Info.Moment.Stand_T = 0.f;
			Control_Info->L_Leg_Info.Moment.Roll_F  = 0.f;
            Control_Info->L_Leg_Info.VMC.Gravity_Compensation = 200.f;

		}

		if(Control_Info->R_Leg_Info.Support.Flag == 1) {

		    Control_Info->R_Leg_Info.Moment.Turn_T  = 0.f;
            Control_Info->R_Leg_Info.Moment.Stand_T = 0.f;
			Control_Info->R_Leg_Info.Moment.Roll_F  = 0.f;
            Control_Info->R_Leg_Info.VMC.Gravity_Compensation = 200.f;	
		}

      Control_Info->L_Leg_Info.Moment.T =   (Control_Info->L_Leg_Info.Moment.Stand_T +	Control_Info->L_Leg_Info.Moment.Turn_T);
	    Control_Info->R_Leg_Info.Moment.T =    Control_Info->R_Leg_Info.Moment.Stand_T +    Control_Info->R_Leg_Info.Moment.Turn_T;

		   if(Control_Info->Init.Yaw_Init.IF_Yaw_Init == 0) {
			 
			    Control_Info->L_Leg_Info.Moment.T =  	Control_Info->L_Leg_Info.Moment.Turn_T;
	        Control_Info->R_Leg_Info.Moment.T =   Control_Info->R_Leg_Info.Moment.Turn_T;
			   
			 
			 }
		
		
		  VAL_LIMIT(Control_Info->L_Leg_Info.Moment.T,-54.f,54.f);
			VAL_LIMIT(Control_Info->R_Leg_Info.Moment.T,-54.f,54.f);


    Control_Info->L_Leg_Info.Moment.F =   Control_Info->L_Leg_Info.VMC.Gravity_Compensation  +  Control_Info->L_Leg_Info.Moment.Leg_Length_F + Control_Info->L_Leg_Info.Moment.Roll_F;
		Control_Info->R_Leg_Info.Moment.F =   Control_Info->R_Leg_Info.VMC.Gravity_Compensation  +  Control_Info->R_Leg_Info.Moment.Leg_Length_F + Control_Info->R_Leg_Info.Moment.Roll_F;
			
//      VAL_LIMIT(Control_Info->L_Leg_Info.Moment.F,-300.f,300.f);
//			VAL_LIMIT(Control_Info->R_Leg_Info.Moment.F,-300.f,300.f);
	
//   	  Control_Info->K = (1.f  - 1.f*fabsf(-INS_Info.Angle[2]));
//	    VAL_LIMIT(Control_Info->K,0.f,1.f);	 
			 Control_Info->K = 1.f;
		Control_Info->L_Leg_Info.Moment.Tp = -Control_Info->L_Leg_Info.Moment.Balance_Tp*Control_Info->K + Control_Info->L_Leg_Info.Moment.Leg_Coordinate_Tp;
		Control_Info->R_Leg_Info.Moment.Tp =  Control_Info->R_Leg_Info.Moment.Balance_Tp*Control_Info->K + Control_Info->L_Leg_Info.Moment.Leg_Coordinate_Tp;

      VAL_LIMIT(Control_Info->L_Leg_Info.Moment.Tp,-54.f,54.f);    
			VAL_LIMIT(Control_Info->R_Leg_Info.Moment.Tp,-54.f,54.f);    

}

static void VMC_F_Tp_To_Joint_Calculate(Control_Info_Typedef *Control_Info){

	 float Phi2_3,Phi0_3,Phi1_2,Phi0_2,Phi3_4;
   
	 Phi2_3=  ( Control_Info->L_Leg_Info.VMC.Phi2 -  Control_Info->L_Leg_Info.VMC.Phi3);
   Phi0_3=  ( Control_Info->L_Leg_Info.VMC.Phi0 -  Control_Info->L_Leg_Info.VMC.Phi3);
   Phi1_2=  ((Control_Info->L_Leg_Info.VMC.Phi1) - Control_Info->L_Leg_Info.VMC.Phi2);
   Control_Info->L_Leg_Info.SendValue.T1 = -(((Control_Info->L_Leg_Info.VMC.L1 * arm_sin_f32(Phi1_2))*(Control_Info->L_Leg_Info.Moment.F 
	                                      *Control_Info->L_Leg_Info.VMC.L0 * arm_sin_f32(Phi0_3)+ 
	                                      Control_Info->L_Leg_Info.Moment.Tp * arm_cos_f32(Phi0_3)))
	                                    /(Control_Info->L_Leg_Info.VMC.L0*arm_sin_f32(Phi2_3)));
   Phi0_2 =  ( Control_Info->L_Leg_Info.VMC.Phi0 -  Control_Info->L_Leg_Info.VMC.Phi2);
   Phi3_4 =  ( Control_Info->L_Leg_Info.VMC.Phi3 - (Control_Info->L_Leg_Info.VMC.Phi4));
  Control_Info->L_Leg_Info.SendValue.T2=  -(((Control_Info->L_Leg_Info.VMC.L4 * arm_sin_f32(Phi3_4))*(Control_Info->L_Leg_Info.Moment.F  
	                                     *Control_Info->L_Leg_Info.VMC.L0 * arm_sin_f32(Phi0_2)+ 
	                                      Control_Info->L_Leg_Info.Moment.Tp  * arm_cos_f32(Phi0_2)))
	                                    /(Control_Info->L_Leg_Info.VMC.L0*arm_sin_f32(Phi2_3)));
	
	 Phi2_3=  ( Control_Info->R_Leg_Info.VMC.Phi2 -  Control_Info->R_Leg_Info.VMC.Phi3);
   Phi0_3=  ( Control_Info->R_Leg_Info.VMC.Phi0 -  Control_Info->R_Leg_Info.VMC.Phi3);
   Phi1_2=  ((Control_Info->R_Leg_Info.VMC.Phi1) - Control_Info->R_Leg_Info.VMC.Phi2);
   Control_Info->R_Leg_Info.SendValue.T1 = -(((Control_Info->R_Leg_Info.VMC.L1 * arm_sin_f32(Phi1_2))*(Control_Info->R_Leg_Info.Moment.F 
	                                      *Control_Info->R_Leg_Info.VMC.L0 * arm_sin_f32(Phi0_3)+ 
	                                      Control_Info->R_Leg_Info.Moment.Tp * arm_cos_f32(Phi0_3)))
	                                    /(Control_Info->R_Leg_Info.VMC.L0*arm_sin_f32(Phi2_3)));
   Phi0_2 =  ( Control_Info->R_Leg_Info.VMC.Phi0 -  Control_Info->R_Leg_Info.VMC.Phi2);
   Phi3_4 =  ( Control_Info->R_Leg_Info.VMC.Phi3 - (Control_Info->R_Leg_Info.VMC.Phi4));
  Control_Info->R_Leg_Info.SendValue.T2=  -(((Control_Info->R_Leg_Info.VMC.L4 * arm_sin_f32(Phi3_4))*(Control_Info->R_Leg_Info.Moment.F  
	                                     *Control_Info->R_Leg_Info.VMC.L0 * arm_sin_f32(Phi0_2)+ 
	                                      Control_Info->R_Leg_Info.Moment.Tp  * arm_cos_f32(Phi0_2)))
	                                    /(Control_Info->R_Leg_Info.VMC.L0*arm_sin_f32(Phi2_3)));
																			
	     VAL_LIMIT(Control_Info->L_Leg_Info.SendValue.T1,-54.f,54.f);
			VAL_LIMIT(Control_Info->L_Leg_Info.SendValue.T2,-54.f,54.f);  
     VAL_LIMIT(Control_Info->R_Leg_Info.SendValue.T1,-54.f,54.f);
			VAL_LIMIT(Control_Info->R_Leg_Info.SendValue.T2,-54.f,54.f);  			
																		
																			
}
