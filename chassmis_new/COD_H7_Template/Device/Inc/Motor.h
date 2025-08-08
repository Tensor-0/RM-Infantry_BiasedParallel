#ifndef Motor_H
#define Motor_H

#include "bsp_can.h"
#include "stdbool.h"

typedef enum{

  DM_4310,
  DM_6220,
	DM_8009,
  DM_Motor_Type_Num,

}DM_Motor_Type_e;

typedef enum{

  Motor_Enable,
  Motor_Disable,
  Motor_Save_Zero_Position,
  MIT_Motor_CMD_Type_Num,

}MIT_Motor_CMD_e;


typedef struct 
{
  uint8_t  State; 	/*!< Motor ERROR Message */
  uint16_t  P_int;
  uint16_t  V_int;
  uint16_t  T_int;
  float  Position;   /*!< Motor Positon */
  float  Velocity;   /*!< Motor Velocity  */
  float  Torque;  /*!< Motor Torque */
  float  Temperature_MOS;   /*!< Motor Temperature_MOS */
  float  Temperature_Rotor;   /*!< Motor Temperature_Rotor */
}DM_Motor_Data_Typedef;

typedef struct 
{
	uint8_t   ID;
	uint8_t  State;
  uint16_t  P_int;
  uint16_t  V_int;
  uint16_t  C_int;
	uint16_t  T_int;
  float  Position;   /*!< Motor Positon */
  float  Velocity;   /*!< Motor Velocity  */
	float  Current;
  float  Torque;  /*!< Motor Torque */

}GIM_Motor_Data_Typedef;

typedef struct
{
  uint32_t Tx_Identifier;   
  uint32_t Rx_Identifier;  
}Motor_CANIdentifierInfo_typedef;

typedef struct
{
	bool lost;
	uint8_t Oline_cnt;
  Motor_CANIdentifierInfo_typedef CAN_Identifier_Info;
	DM_Motor_Type_e Type;
	DM_Motor_Data_Typedef Data;  
}DM_Motor_Info_Typedef;

typedef struct
{
	bool lost;
	uint8_t Oline_cnt;
  Motor_CANIdentifierInfo_typedef CAN_Identifier_Info;
	GIM_Motor_Data_Typedef Data;
}GIM_Motor_Info_Typedef;

typedef struct
{
	float  KP;
	float  KD;
	float  Position;   /*!< Motor Positon */
  float  Velocity;   /*!< Motor Velocity  */
  float  Torque;  /*!< Motor Torque */
	
}MIT_Motor_Control_Typedef;

extern DM_Motor_Info_Typedef DM_8009_Motor[4], DM_Yaw_Motor;

extern GIM_Motor_Info_Typedef GIM_8108_Motor[2];

extern MIT_Motor_Control_Typedef DM_Motor_Control[2];

void DM_Motor_Info_Update(uint32_t *Identifier,uint8_t *Data,DM_Motor_Info_Typedef *DM_Motor);

extern void GIM_Motor_Info_Update(uint32_t *Identifier,uint8_t *Data,GIM_Motor_Info_Typedef *GIM_Motor);

extern void DM_Motor_Multi_Info_Update(uint8_t *Data,DM_Motor_Info_Typedef *DM_Motor);

void MIT_Motor_Command(FDCAN_TxFrame_TypeDef *TxFrame,uint16_t TxStdId,uint8_t CMD);

extern void DM_Motor_CAN_TxMessage(FDCAN_TxFrame_TypeDef *TxFrame,DM_Motor_Info_Typedef *DM_Motor,
	                     float Postion, float Velocity, float KP, float KD, float Torque);

extern void GIM_Motor_Motor_CAN_TxMessage(FDCAN_TxFrame_TypeDef *TxFrame,GIM_Motor_Info_Typedef *GIM_Motor,
	                     float Postion, float Velocity, float KP, float KD, float Torque);
#endif