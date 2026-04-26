#ifndef INS_TASK_H
#define INS_TASK_H

#include "stdint.h"

typedef struct 
{
	float Pitch_Angle;
	float Yaw_Angle;
	float Yaw_TolAngle;
	float Roll_Angle;

  float Pitch_Gyro;
  float Yaw_Gyro;
  float Roll_Gyro;

  float Angle[3];
	float Gyro[3];	
	float Accel[3];
	
	float Last_Yaw_Angle;
	int16_t YawRoundCount;


}INS_Info_Typedef;


extern INS_Info_Typedef INS_Info; 

#endif //INS_TASK_H
