#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H
#include "Kalman_Filter.h"

typedef enum 
{
  CONTROL_OFF,
  CONTROL_REMOTE,
  CONTROL_KEYBOARD,
}Control_Mode_e;

typedef enum 
{
  CHASSIS_WEAK,
  CHASSIS_BALANCE,
  CHASSIS_SLIP,
  Chassis_Situation_NUM,
}Chassis_Situation_e;

typedef enum 
{
  CHASSIS_FRONT,
  CHASSIS_SIDE,
  CHASSIS_SPIN,
  CHASSIS_MODE_NUM,
}Chassis_Mode_e;

typedef enum 
{
  LEG_LENGTH_NORMAL,
  LEG_LENGTH_HIGH,
}Leg_Length_Mode_e;



typedef struct 
{
	float Fusion_Velocity;
	float Predict_Velocity;
	float W_Velocity;
	float LQR_K[2][6];
	float LQR_X[6];
	float LQR_Output[2][6];

  struct
  {
	float L1,L2,L3,L4,L5;
  float L0,L0_dot,Last_L0,Target_L0;
	float Phi1,Phi4;
	float  Phi2,Phi3,Phi0,Last_Phi0;
	float X_D_Difference_X_B,Y_D_Difference_Y_B;
	float Phi1_dot,Phi4_dot,Phi0_dot;
	float X_B,Y_B,X_D,Y_D,X_C,Y_C,X_C_dot,Y_C_dot;
	float A0,B0,C0,LBD_2;
	float	Gravity_Compensation;
  }VMC;
	
		struct
  {
		float Phi;
		float Phi_dot;
    float Chassis_Position;
    float Chassis_Velocity;
    float Theta;
    float Theta_dot;
  
	}Target;
  
  struct
  {
		
		float Phi;
		float Phi_dot;
		float Last_Phi;
    float Chassis_Position;
    float Chassis_Velocity;
		float Theta;
    float Last_Theta;
    float Theta_dot;
		
		float F;
		float Tp;
  }Measure;
	
	  struct
  {
		float FN;
		float P;
		bool Flag;
 
  }Support;
	
	struct{
		float T;
		float Tp;
		float F;
		float Balance_Tp;
		float Leg_Coordinate_Tp;
		float Stand_T;
	  float Turn_T;
		float Roll_F;
		float Leg_Length_F;
	}Moment;
  
	struct{
	
   float T1;
	 float T2;
		
	}SendValue;
	
	
	
}Leg_Info_Typedef;


typedef struct{
    Control_Mode_e   Control_Mode;
	Chassis_Situation_e  Chassis_Situation;
    Chassis_Mode_e Chassis_Mode;
		Leg_Length_Mode_e  Leg_Length_Mode;

	
	
	
	struct{
	bool Chassis_Jump;
	bool Chassis_Jump_Complete;
	bool Flag1;
	bool Flag2;
	bool Flag3;
	uint16_t Jump_Cnt1;
	uint16_t Jump_Cnt2;
  uint16_t Jump_Cnt3;
  }Jump;
	
	struct
  {
		
		 
		bool Begin_Init;
		bool Motor_Enable;
	
		struct{
		 bool Joint_Reduction_Flag[4];
		 bool IF_Joint_Init;
		}Joint_Init;
		
		struct{
		bool IF_Balance_Init;
		uint16_t Balance_Right_Cnt;
		}Balance_Init;
        
	 struct{
		bool IF_Yaw_Init;
		uint16_t Yaw_Right_Cnt;
		}Yaw_Init;
		
	 struct{
		bool IF_Velocity_Init;
		uint16_t Velocity_Cnt;
	 }Velocity_Init;	
		
	}Init;
	
	
	struct{
		bool Enable_Flag;
	}Motor_Online;
	
	
	
	
  Leg_Info_Typedef L_Leg_Info;
	Leg_Info_Typedef R_Leg_Info;
	
	float Yaw_Err;
	float Target_Yaw;
	float Accel;
	float Fusion_Velocity;
	float Chassis_Velocity;
	float Predict_Velocity;
	float Target_Velocity;
	float K;

  struct{
     float Angle;
		 float Velocity;
     float Err;
 }Yaw;
	
	
	
	struct{
		float K1, K2;
		float Torgue[2];
		float Torgue_2[2];
		float Velocity_2[2];
		float All_Torgue_2;
		float All_Velocity_2;
    struct{
      float All_Torgue_2;
		  float Torgue_2[2];
			float Torgue[2];
		  float Mechanical_Power;
			float Predict_Power;
	  }Other;		
		struct{
      float All_Torgue_2;
		  float Torgue_2[2];
			float Torgue[2];
		  float Mechanical_Power;
			float Predict_Power;
	  }Balance;
		float Predict_Power;
		float Mechanical_Power;
		float Limit_Power;		
		bool  Referee_System_Power_Update_Flag;
		
		
	}Power_Control;
	
	  struct
  {
		bool Begin_Climb_Flag;
		bool Climb_Flag;
		uint16_t Cnt;
    bool Climb_Complete_Flag;
  }Climb;
	
	struct
  {
    uint16_t Slip_Cnt;
		bool Slip_Flag;
		
  }Slip;
	
}Control_Info_Typedef;
	
	
	
extern 	Control_Info_Typedef 	Control_Info;







#endif