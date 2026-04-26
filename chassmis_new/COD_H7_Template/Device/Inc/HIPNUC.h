#ifndef HIPNUC_H
#define HIPNUC_H

#include "stdint.h"

typedef struct 
{  
    
  int8_t Temperature;  	
	float Air_Pressure;
 	float Accel[3];
	float Gyro[3]; 
	float Roll;
	float Pitch;
	float Yaw;
  float quat[4];
	
}HiPNUC_Info_Typedef;


extern HiPNUC_Info_Typedef HiPNUC_Info;

extern uint8_t HiPNUC_Data[2][82];

extern void HiPNUC_Info_Update(HiPNUC_Info_Typedef *HiPNUC_Info,volatile const uint8_t *Data);









#endif




