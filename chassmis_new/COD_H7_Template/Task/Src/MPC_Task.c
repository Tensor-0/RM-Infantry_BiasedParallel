#include "cmsis_os.h"
#include "MPC_Task.h"
#include "Control_Task.h"
#include "Motor.h"

float A[36] = {1.002199f,0.005004f,0.000000f,0.000000f,0.000265f,0.000000f,
               0.880039f,1.002199f,0.000000f,0.000000f,0.106165f,0.000265f,
              -0.000196f,-0.000000f,1.000000f,0.005000f,-0.000005f,-0.000000f,
              -0.078301f,-0.000196f,0.000000f,1.000000f,-0.001858f,-0.000005f,
               0.001122f,0.000002f,0.000000f,0.000000f,1.000929f,0.005002f,
               0.449202f,0.001122f,0.000000f,0.000000f,0.371512f,1.000929f   };
float B[12] = { -0.000727f,0.000520f,
                -0.291088f,0.208160f,
                0.000093f,-0.000034f,
                0.037107f,-0.013493f,
                -0.000214f,0.000791f,
                -0.085732f,0.316510f};

								
static void Model_Predict_Contol_Init(Model_Predict_Control_Info *MPC);
static void Model_Predict_Contol_Calculate(Model_Predict_Control_Info *MPC);
								

								
Model_Predict_Control_Info	MPC;							
								
void MPC_Task(void const * argument)
{

	Model_Predict_Contol_Init(&MPC);
	
  for(;;)
	{
	 

	  osDelay(1);
  }
		
		
}
		
		
		
static void Model_Predict_Contol_Init(Model_Predict_Control_Info *MPC){

  MPC->Data.A = (float *)malloc(4 * 6 * 6);
  MPC->Data.B = (float *)malloc(4 * 2 * 6);
  MPC->Data.X = (float *)malloc(4 * 1 * 6);
  MPC->Data.U = (float *)malloc(4 * 1 * 2);
  MPC->Data.Cache[0] = (float *)malloc(4 * 1 * 6);
  MPC->Data.Cache[1] = (float *)malloc(4 * 1 * 6);
  MPC->Data.Output = (float *)malloc(4 * 1 * 6);

	
	
	
   memset(MPC->Data.A, 0, 4*6*6);
   memset(MPC->Data.B, 0, 4*2*6);
   memset(MPC->Data.X, 0, 4*6*1);
   memset(MPC->Data.U, 0, 4*2*1);
   memset(MPC->Data.Cache[0], 0, 4*6*1);
   memset(MPC->Data.Cache[1], 0, 4*6*1);
   memset(MPC->Data.Output, 0, 4*6*1);

	 MPC->Data.A = A;
   MPC->Data.B = B;
   MPC->Data.X = MPC->X;
   MPC->Data.U = MPC->U;
   MPC->Data.Output = MPC->Output;
	
   Matrix_Init(&MPC->Mat.A, 6,6, (float *)MPC->Data.A);
   Matrix_Init(&MPC->Mat.B, 6,2, (float *)MPC->Data.B);
   Matrix_Init(&MPC->Mat.X, 6,1, (float *)MPC->Data.X);
   Matrix_Init(&MPC->Mat.U, 2,1, (float *)MPC->Data.U);
   Matrix_Init(&MPC->Mat.Cache[0], 6,1, (float *)MPC->Data.Cache[0]);
   Matrix_Init(&MPC->Mat.Cache[1], 6,1, (float *)MPC->Data.Cache[1]);
   Matrix_Init(&MPC->Mat.Output, 6,1, (float *)MPC->Data.Output);



}		
		
		
static void Model_Predict_Contol_Calculate(Model_Predict_Control_Info *MPC){


 MPC->MatStatus = Matrix_Multiply(&MPC->Mat.A,&MPC->Mat.X,&MPC->Mat.Cache[0]);
 MPC->MatStatus = Matrix_Multiply(&MPC->Mat.B,&MPC->Mat.U,&MPC->Mat.Cache[1]);
 MPC->MatStatus = Matrix_Add(&MPC->Mat.Cache[0],&MPC->Mat.Cache[1],&MPC->Mat.Output);



}
		
		
		
		
		
		
		
		
	