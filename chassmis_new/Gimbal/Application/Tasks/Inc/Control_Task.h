/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Control_Task.h
  * @brief          : Control task
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"

/* Exported defines -----------------------------------------------------------*/
#define SHOOT_SPEED_15M_S    4600
#define SHOOT_SPEED_18M_S    5100
#define SHOOT_SPEED_30M_S    7250
#define TRIGGER_FREQ_1_HZ     450
#define TRIGGER_FREQ_2_HZ     750
#define TRIGGER_FREQ_3_HZ     1000
#define TRIGGER_FREQ_4_HZ     1250
#define TRIGGER_FREQ_5_HZ     1500
#define TRIGGER_FREQ_6_HZ     1750
#define TRIGGER_FREQ_7_HZ     2000
#define TRIGGER_FREQ_8_HZ     2400
#define TRIGGER_FREQ_9_HZ     2700
#define TRIGGER_FREQ_10_HZ    3000
#define TRIGGER_FREQ_12_HZ    3600
#define TRIGGER_FREQ_14_HZ    4200
#define TRIGGER_FREQ_16_HZ    4800
#define SignBit(x)  ((x) > 0 ? 1 : ((x) < 0 ? -1 : 0))

/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef enum that contains the Shooter Mode.
 */
typedef enum{
  SHOOTER_OFF,
  SHOOTER_SINGLE,
  SHOOTER_BURSTS,
  SHOOTER_VISION,
  SHOOTER_MODE_NUM,
}Shooter_Mode_e;

/**
 * @brief typedef enum that contains the Gimbaol Mode.
 */
typedef enum{
	GIMBAL_OFF,
	GIMBAL_IMU,
	GIMBAL_VISION,
  GIMBAL_MODE_NUM,
}Gimbal_Mode_e;



typedef enum{
	CONTROL_REMOTE,
	CONTROL_KEYBOARD,
	CONTROL_OFF,

}Control_Mode_e;


/**
 * @brief typedef structure that contains the information for the gimbal control.
 */

typedef struct
{
  Shooter_Mode_e Mode;
	
	bool Vision_Fire_Flag;
		
	struct{

    int16_t ShootSpeed;
    float TriggerSpeed;
    float TriggerAngle;
	}Target;
	
	struct{
    int16_t ShootLeftSpeed;
    int16_t ShootRightSpeed;
    int16_t TriggerSpeed;
    float  TriggerAngle;
	}Measure;
   
	float TriggerAngle_Err;
	bool Shoot_Mode_Change;

  
	struct{
  
		bool First_Press_L;
	  uint16_t When_Press_L_Heat;
		uint16_t Update_Heat;
		uint16_t Count;
		uint8_t Allowable_Shoot_17mm;
		float Allowable_S;
		bool  Allow_Flag;
	}Limit;
	
	
	
	
	
}Shoot_Info_Typedef;
typedef struct
{
   Gimbal_Mode_e Mode;
	 float Yaw_Err;
	
	struct{
		float Last_Pitch_Angle;
		float Last_Yaw_Angle;
    float Pitch_Angle;
		float Yaw_Angle;
		float Pitch_Gyro;
		float Yaw_Gyro;
	}Target;
	
	struct{
    float Pitch_Angle;
		float Yaw_Angle;
	}Feed_Forward;
	
	
	
	struct{
		uint8_t Yaw_int[4];
	  uint8_t Pitch_int[4];
	 uint8_t Distance_int[4];
    float Distance;
		float Pitch_Angle;
		float Yaw_Angle;
		bool Flag;
	}Vision;
	
	
	struct{
    float Pitch_Angle;
		float Yaw_Angle;
		float Pitch_Gyro;
		float Yaw_Gyro;
	}Measure;

	struct{
		float Pitch;
		float Yaw;
	}Output;
	
	
  	struct{
		float Min;
		float Max;
	}Limit_Pitch;
	
	
	
	

}Gimbal_Info_Typedef;
typedef struct
{
	Control_Mode_e  Control_Mode;
  int16_t SendValue[3];
  Gimbal_Info_Typedef Gimbal;
  Shoot_Info_Typedef  Shoot;
  
	struct{
		uint16_t Shooter_17mm_1_Barrel_Heat;
		float Shoot_Initial_Speed;
		uint8_t Shooter_Barrel_Cooling_Value;
		uint16_t Shooter_Barrel_Heat_limit;
		bool Heat_Update_Flag;
	}Refree;	
	
}Control_Info_Typedef;

/* Exported variables ----------------------------------------------------------*/
extern Control_Info_Typedef Control_Info;
extern float forward;

#endif // CONTROL_TASK_H
