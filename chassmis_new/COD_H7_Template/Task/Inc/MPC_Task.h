#ifndef MPC_TASK_H
#define MPC_TASK_H

#include "Kalman_Filter.h"

typedef struct 
{

	struct{
	Matrix A;
  Matrix B;
  Matrix X;
  Matrix U;
	Matrix Cache[2];
	Matrix Output;
	}Mat;
	
	struct{
  float *A;
  float *B;
  float *X;
	float	*U;
  float *Cache[2];
  float *Output;

  }Data;
	
	float X[6] ;
  float U[2] ;
  float Output[6] ;
  float Predict[6] ;
	
	
	
	arm_status MatStatus;
	
	
}Model_Predict_Control_Info;

extern Model_Predict_Control_Info MPC;
#endif
