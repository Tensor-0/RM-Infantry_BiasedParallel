#include "Referee_System.h"
#include "CRC.h"
#include "Control_Task.h"



__attribute__((section (".AXI_SRAM"))) uint8_t Referee_MultiRx_Buf[2][Referee_System_Info_Max_Length];

Referee_System_Info_TypeDef  Referee_System_Info;

static void Referee_System_Info_Update(uint8_t *Buff,Referee_System_Info_TypeDef *Referee_System_Info);

void Referee_Frame_Update(uint8_t *Buff)
{
	Referee_System_Info.Index = 0;
	Referee_System_Info.DataLength = 0;
	
  while (Buff[Referee_System_Info.Index] == 0xA5)
	{
    if(Verify_CRC8_Check_Sum(&Buff[Referee_System_Info.Index],FrameHeader_Length) == true)
    {
      Referee_System_Info.DataLength = (uint16_t)(Buff[Referee_System_Info.Index+2]<<8 | Buff[Referee_System_Info.Index+1]) + FrameHeader_Length + CMDID_Length + CRC16_Length;
      if(Verify_CRC16_Check_Sum(&Buff[Referee_System_Info.Index],Referee_System_Info.DataLength) == true)
      {
        Referee_System_Info_Update(Buff,&Referee_System_Info);
      }
    }else{
		 break;
		}
		Referee_System_Info.Index += Referee_System_Info.DataLength;
  }
}

static void Referee_System_Info_Update(uint8_t *Buff,Referee_System_Info_TypeDef *Referee_System_Info){

  switch (bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length]))
  {
     #ifdef ROBOT_STATUS_ID
    case ROBOT_STATUS_ID:
		Referee_System_Info->Robot_Status.Robot_ID  = 	 Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length];
		Referee_System_Info->Robot_Status.Robot_Level =  Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 1];
		Referee_System_Info->Robot_Status.Current_HP =   bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 2]);
		Referee_System_Info->Robot_Status.Maximum_HP =   bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 4]);
		Referee_System_Info->Robot_Status.Shooter_Barrel_Cooling_Value =  bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 6]);
		Referee_System_Info->Robot_Status.Shooter_Barrel_Heat_Limit =     bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 8]);
		Referee_System_Info->Robot_Status.Chassis_Power_Limit =           bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 24]);
    Referee_System_Info->Robot_Status.Power_Management_Gimbal_Output  = (Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 26] & 0x01);
    Referee_System_Info->Robot_Status.Power_Management_Chassis_Output = (Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 26] & 0x02) >> 1;
    Referee_System_Info->Robot_Status.Power_Management_Shooter_Output = (Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 26] & 0x04) >> 2;
    break;
     #endif       
		
   #ifdef POWER_HEAT_ID
    case POWER_HEAT_ID:
		  Referee_System_Info->Power_Heat_Data.Update_Flag = 1;
      Referee_System_Info->Power_Heat_Data.Chassis_Voltage    = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length]);
      Referee_System_Info->Power_Heat_Data.Chassis_Current = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 2]);
      Referee_System_Info->Power_Heat_Data.Chassis_Power   = bit8TOfloat32(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 4]);
      Referee_System_Info->Power_Heat_Data.Buffer_Energy = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 8]);
      Referee_System_Info->Power_Heat_Data.Shooter_17mm_1_Barrel_Heat = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 10]);
      Referee_System_Info->Power_Heat_Data.Shooter_17mm_2_Barrel_Heat = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 12]);
      Referee_System_Info->Power_Heat_Data.Shooter_42mm_Barrel_Heat = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 14]);
    break;
    #endif
		 
	 #ifdef SHOOT_DATA_ID
    case SHOOT_DATA_ID:
      Referee_System_Info->Shoot_Data.Bullet_Type  = Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length];
      Referee_System_Info->Shoot_Data.Shooter_Number   = Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 1];
      Referee_System_Info->Shoot_Data.Launching_Frequency  = Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 2];
      Referee_System_Info->Shoot_Data.Initial_Speed = bit8TOfloat32(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 3]);
    break;
#endif
  }

}

int16_t bit8TObit16(uint8_t change_info[2])
{
	union
	{
    int16_t  bit16;
		uint8_t  byte[2];
	}u16val;

  u16val.byte[0] = change_info[0];
  u16val.byte[1] = change_info[1];

	return u16val.bit16;
}


float bit32TOfloat32(uint32_t Change_Info)
{
	union
	{
    float float32;
		uint32_t  byte;
	}u32val;

  u32val.byte = Change_Info;

	return u32val.float32;
}



uint8_t float32TObit8(uint8_t i,float change_info)
{
	union
	{
    float float32;
		uint8_t  byte[4];
	}u32val;

  u32val.float32 = change_info;

	return u32val.byte[i];
}

float bit8TOfloat32(uint8_t change_info[4])
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