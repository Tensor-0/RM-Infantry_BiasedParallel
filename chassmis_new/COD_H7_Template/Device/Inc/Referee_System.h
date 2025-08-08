#ifndef REFEREE_SYSTEM_H
#define REFEREE_SYSTEM_H

#include "stdint.h"
#include "stdbool.h"

#define Referee_System_Info_Max_Length  136  

#define FrameHeader_Length    5U   /*!< the length of frame header */
#define CMDID_Length          2U   /*!< the length of CMD ID */
#define CRC16_Length          2U   /*!< the length of CRC ID */

#pragma  pack()


extern uint8_t float32TObit8(uint8_t i,float change_info);
extern float bit8TOfloat32(uint8_t change_info[4]);

extern void Referee_Frame_Update(uint8_t *Buff);
extern uint8_t Referee_MultiRx_Buf[2][Referee_System_Info_Max_Length];
extern float bit32TOfloat32(uint32_t Change_Info);
extern int16_t bit8TObit16(uint8_t change_info[2]);

// #define GAME_STATUS_ID                    0x0001U  /*!< game status data */
// #define GAME_RESULT_ID                    0x0002U  /*!< game result data */
// #define GAME_ROBOTHP_ID                   0x0003U  /*!< robot HP data */

// #define EVENE_DATA_ID                     0x0101U  /*!< site event data */
// #define SUPPLY_ACTION_ID                  0x0102U  /*!< supply station action data */

//#define REFEREE_WARNING_ID                0x0104U  /*!< referee warning data */
//#define DART_INFO_ID                     0x0105U  /*!< dart shoot data */

#define ROBOT_STATUS_ID                   0x0201U  /*!< robot status data */

#define POWER_HEAT_ID                     0x0202U  /*!< real power heat data */

//#define ROBOT_POSITION_ID                 0x0203U  /*!< robot position data */
// #define ROBOT_BUFF_ID                     0x0204U  /*!< robot buff data */
// #define AIR_SUPPORT_ID                  0x0205U  /*!< aerial robot energy data */
// #define ROBOT_HURT_ID                     0x0206U  /*!< robot hurt data */
#define SHOOT_DATA_ID                       0x0207U  /*!< real robo t shoot data */
//#define PROJECTILE_ALLOWANCE_ID             0x0208U  /*!< bullet remain data */
//                
//#define RFID_STATUS_ID                    0x0209U  /*!< RFID status data */
//#define DART_CLIENT_CMD_ID                0x020AU  /*!< DART Client cmd data */
// #define GROUND_ROBOT_POSITION_ID                0x020BU  /*!< ground robot position */

//#define RADAR_MARAKING_ID                  0x020CU     /*!< Radar marking progress*/
//#define SENTRY_INFO_ID          0X020DU    /*!< SENTRY make autonomous decisions*/
//#define RADAR_INFO_ID             0X020EU   /*!< RADAR make autonomous decisions*/



typedef struct 
{ 
  uint8_t Game_Type : 4; 
  uint8_t Game_Progress : 4; 
  uint16_t Stage_Remain_Time; 
  uint64_t SyncTimeStamp; 
}Game_Status_t; 

typedef struct 
{ 
   uint8_t Winner; 
}Game_Result_t; 

typedef  struct 
{ 
  uint16_t Red_1_Robot_HP; 
  uint16_t Red_2_Robot_HP; 
  uint16_t Red_3_Robot_HP; 
  uint16_t Red_4_Robot_HP; 
  uint16_t Red_5_Robot_HP; 
  uint16_t Red_7_Robot_HP; 
  uint16_t Red_Outpost_HP; 
  uint16_t Red_Base_HP; 
  uint16_t Blue_1_Robot_HP; 
  uint16_t Blue_2_Robot_HP; 
  uint16_t Blue_3_Robot_HP; 
  uint16_t Blue_4_Robot_HP; 
  uint16_t Blue_5_Robot_HP; 
  uint16_t Blue_7_Robot_HP; 
  uint16_t Blue_Outpost_HP; 
  uint16_t Blue_base_HP; 
	
}Game_Robot_HP_t; 



typedef struct 
{ 
  uint32_t Event_Data; 
}Event_Data_t; 

typedef  struct 
{ 
  uint8_t Reserved; 
  uint8_t Supply_Robot_ID; 
  uint8_t Supply_Projectile_Step; 
  uint8_t Supply_Projectile_Num; 
} Ext_Supply_Projectile_Action_t;

typedef struct 
{ 
  uint8_t Level; 
  uint8_t Offending_Robot_ID; 
  uint8_t Count; 
}Referee_Warning_t; 

typedef struct 
{ 
  uint8_t Dart_Remaining_Time; 
  uint16_t Dart_Info; 
}Dart_Info_t; 

typedef  struct 
{ 
  uint8_t Robot_ID; 
  uint8_t Robot_Level; 
  uint16_t Current_HP;  
  uint16_t Maximum_HP; 
  uint16_t Shooter_Barrel_Cooling_Value; 
  uint16_t Shooter_Barrel_Heat_Limit;
  uint16_t Chassis_Power_Limit;  
  uint8_t Power_Management_Gimbal_Output : 1; 
  uint8_t Power_Management_Chassis_Output : 1;  
  uint8_t Power_Management_Shooter_Output : 1; 
}Robot_Status_t; 

typedef struct 
{ 
	bool Update_Flag;
  uint16_t Chassis_Voltage; 
  uint16_t Chassis_Current; 
  float Chassis_Power; 
  uint16_t Buffer_Energy; 
  uint16_t Shooter_17mm_1_Barrel_Heat; 
  uint16_t Shooter_17mm_2_Barrel_Heat; 
  uint16_t Shooter_42mm_Barrel_Heat; 
}Power_Heat_Data_t; 

typedef struct 
{ 
  float X; 
  float Y; 
  float Angle; 
}Robot_Pos_t; 

typedef struct 
{ 
  uint8_t Recovery_Buff;  
  uint8_t Cooling_Buff;  
  uint8_t Defence_Buff;  
  uint8_t Vulnerability_Buff; 
  uint16_t Attack_Buff; 
}Buff_t; 

typedef struct 
{ 
  uint8_t Airforce_Status; 
  uint8_t Time_Remain; 
}Air_Support_Data_t; 

typedef struct 
{ 
  uint8_t Armor_ID : 4; 
  uint8_t HP_Deduction_Reason : 4; 
}Hurt_Data_t; 

typedef struct 
{ 
  uint8_t Bullet_Type;  
  uint8_t Shooter_Number; 
  uint8_t Launching_Frequency;  
  float Initial_Speed;  
}Shoot_Data_t; 

typedef struct 
{
  uint8_t Index;
  uint16_t DataLength;
  
	
#ifdef GAME_STATUS_ID
  Game_status_t Game_Status;
#endif

#ifdef GAME_RESULT_ID
  game_result_t game_result;
#endif

#ifdef  GAME_ROBOTHP_ID
	game_robot_HP_t game_robot_HP;
#endif	
	
#ifdef EVENE_DATA_ID
  event_data_t event_data;
#endif

#ifdef SUPPLY_ACTION_ID   	
  supply_projectile_action_t  supply_projectile_action;
#endif

#ifdef REFEREE_WARNING_ID
    referee_warning_t referee_warning;
#endif

#ifdef DART_INFO_ID
  dart_info_t dart_info;
#endif

#ifdef ROBOT_STATUS_ID
  Robot_Status_t Robot_Status;
#endif

#ifdef POWER_HEAT_ID
  Power_Heat_Data_t Power_Heat_Data;
#endif

#ifdef ROBOT_POSITION_ID
  robot_pos_t  robot_pos;
#endif

#ifdef ROBOT_BUFF_ID
  buff_t  buff;
#endif

#ifdef AIR_SUPPORT_ID
 air_support_data_t air_support_data;
#endif

#ifdef ROBOT_HURT_ID
  hurt_data_t  hurt_data;
#endif

#ifdef SHOOT_DATA_ID
  Shoot_Data_t  Shoot_Data;
#endif

#ifdef PROJECTILE_ALLOWANCE_ID
  projectile_allowance_t projectile_allowance;
#endif
#ifdef  RFID_STATUS_ID
    rfid_status_t rfid_status;
#endif

#ifdef DART_CLIENT_CMD_ID
    dart_client_cmd_t  dart_client_cmd;
#endif

#ifdef GROUND_ROBOT_POSITION_ID
    ground_robot_position_t  ground_robot_position;
#endif 

#ifdef RADAR_MARAKING_ID
    radar_mark_data_t  radar_mark_data;
#endif

#ifdef SENTRY_INFO_ID
    sentry_info_t  sentry_info;
#endif

#ifdef RADAR_INFO_ID
    radar_info_t  radar_info;
#endif
}Referee_System_Info_TypeDef;




extern Referee_System_Info_TypeDef  Referee_System_Info;




#endif
