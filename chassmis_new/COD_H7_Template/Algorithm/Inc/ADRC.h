#ifndef ADRC_H
#define ADRC_H

#include "stdbool.h"
#include "config.h"

typedef struct
{
	 bool  Initialized;
   float Input; 
   float r; 
   float h; 
	 float Output; 
   float d_Output;
   float Err;
	 float fh;
	 float X[2];
   float Last_X[2];
}Tracking_Differentiator_Info_TypeDef;	

extern void Tracking_Differentiator_Init(Tracking_Differentiator_Info_TypeDef *TD,float r,float h);

extern void Tracking_Differentiator_Update(Tracking_Differentiator_Info_TypeDef *TD,float Input);

#endif