
#include "Control_Task.h"
#include "INS_Task.h"//陀螺仪
#include "cmsis_os.h"//FreeRTOS
#include "bsp_uart.h"//串口
#include "bsp_adc.h"//ADC--电压检测--低电压警告
#include "Remote_Control.h"//遥控器
#include "Image_Transmission.h"
#include "Ramp.h"//斜坡函数和移动平均滤波器，用于控制系统的信号平滑处理
#include "PID.h"
#include "Motor.h"
#include "arm_math.h"

//float K11[6] = {0,-161.117157f,196.474176f,-210.652836f,-22.628436f};
//float K12[6] = {0,12.798983f,-27.543440f,-37.270138f,-0.389884f};
//float K13[6] = {0,-69.863233f,70.467840f,-24.641029f,-22.550669f};
//float K14[6] = {0,47.504341f,-45.704640f,3.658661f,-23.154080f};
//float K15[6] = {0,-879.770236f,945.699308f,-371.024185f,73.193803f};
//float K16[6] = {0,-44.249716f,49.457747f,-20.483717f,5.913833f};
//float K21[6] = {0,-503.385834f,537.716406f,-192.249143f,46.119864f};
//float K22[6] = {0,-10.488132f,14.656765f,0.162789f,2.506017f};
//float K23[6] = {0,-325.449185f,352.124943f,-138.337810f,25.760259f};
//float K24[6] = {0,-337.260638f,354.780154f,-132.228603f,23.521027f};
//float K25[6] = {0,852.653086f,-857.295404f,299.554560f,105.138722f};
//float K26[6] = {0,65.525226f,-68.428754f,25.522485f,1.830345f};


/*Q=diag([2000 1 500 100 5000 1]);


R=diag([0.75  0.25]); */
//float K11[6] = {0,-500.905786f,460.989299f,-264.986316f,-35.170758f};
//float K12[6] = {0,1.839739f,-14.912806f,-42.788271f,-0.510816f};
//float K13[6] = {0,-67.045277f,62.200677f,-20.550515f,-23.139665f};
//float K14[6] = {0,162.051504f,-148.028801f,39.259245f,-29.799921f};
//float K15[6] = {0,-1253.481896f,1223.843174f,-446.788399f,82.393889f};
//float K16[6] = {0,-72.291579f,71.733797f,-26.974774f,6.773165f};
//float K21[6] = {0,-1193.375356f,1173.257512f,-415.447351f,82.296961f};
//float K22[6] = {0,11.285863f,-2.650088f,4.258226f,2.422298f};
//float K23[6] = {0,-275.149529f,288.134679f,-113.153230f,22.256308f};
//float K24[6] = {0,-409.194628f,398.516303f,-141.434132f,24.470515f};
//float K25[6] = {0,1528.379420f,-1363.813466f,429.617529f,92.845408f};
//float K26[6] = {0,93.876852f,-87.983942f,30.027810f,1.450183f};


/*Q=diag([1000 10 500 100 5000 1]);


R=diag([0.75  0.25]); */
float K11[6] = {0,-412.921425f,387.990596f,-248.221187f,-28.349087f};
float K12[6] = {0,-0.922566f,-18.491245f,-37.011157f,-2.290644f};
float K13[6] = {0,-92.727783f,88.643227f,-30.230386f,-21.823378f};
float K14[6] = {0,105.630452f,-93.945519f,19.668294f,-26.112547f};
float K15[6] = {0,-1043.655528f,1080.127134f,-422.434524f,83.128812f};
float K16[6] = {0,-59.200577f,60.051940f,-23.270970f,6.282138f};
float K21[6] = {0,-583.855375f,662.372696f,-270.230733f,66.208364f};
float K22[6] = {0,-24.862832f,39.600787f,-14.957582f,6.326057f};
float K23[6] = {0,-249.498529f,291.240241f,-127.192951f,26.520236f};
float K24[6] = {0,-403.712603f,416.374845f,-158.410887f,28.690800f};
float K25[6] = {0,1563.102977f,-1429.120660f,463.833012f,86.995166f};
float K26[6] = {0,91.827424f,-88.152111f,31.005557f,1.174900f};
//========================================================================================
float K_3508_t=   2700.6667f;//
//K_3508_t = 2,730.6666666;
float K_Roll = 0.0003f;
//=========================================================================================


/*
启用一个PID控制的流程
1.定义PID参数数组float  PID_xxx_param[7] = {KP,KI,KD,Max_I,Max_Out,Dead_Band,Lpf_Cutoff_Freq};
2.定义PID控制器PID_Info_TypeDef  PID_xxx;
3.在Control_Init()函数中调用PID初始化函数，例如：PID_xxx.PID_Param_Init(&PID_xxx,PID_xxx_param);
4.在需要使用PID控制的地方，设置目标值和测量值，例如：PID_xxx.Target = xxx; PID_xxx.Measure = xxx;
5.调用PID计算函数，例如：PID_Calc(&PID_xxx);



*/
//static float  PID_Leg_Length_F_Param[7] 	  = {1500.f,1.f ,200000.f,0.f ,0  ,10.f,200.f}; //腿长PID(change)
//腿长
static float  PID_Leg_Length_F_Param[7] 	  = {1200.f,1.f ,20000.f,0.f ,0.f  ,10.f,100.f}; //腿长PID(change

//static float  PID_Leg_Length_R_Param[7] 	  = {0.5f,0.f ,0.f,0.f ,0.f  ,0.01f,0.2f};//腿部长度横滚补偿PID

static float  PID_Leg_Roll_F_Param[7]         = {50.f   ,0.f ,25.f     ,0.f  ,0.f  ,0.1f  ,50.f };


static float  PID_Leg_Coordinate_param[7]   = {300.f ,0.f ,20.0f   ,0.f ,0.f,0.f ,50   }; // 腿部协调PID


//===========================================================================================
//云台
static float  PID_Yaw_P_pama[7] 			      = {4.4f  ,0.f ,60.f    ,0   ,0  ,200 ,500  }; // 偏航角位置PID
static float  PID_Yaw_V_pama[7] 			      = {0.25f ,0   ,0.4f    ,0   ,0  ,200 ,70   }; // 偏航角速度PID

//由于驱动轮减速箱的机械设计存在问题，两边减速箱的阻力不同，导致输出相同的力矩，左右轮速度不同
//通过调整PID参数来尽量补偿这个问题
//static float  PID_Stool_param[2]                  = {15.f,0.f,0.f,0.f,0.f,2000.f,10000.f};//驱动轮速度控制PID

PID_Info_TypeDef PID_Leg_Coordinate;  // 腿部协调控制器
PID_Info_TypeDef PID_Leg_length_F[2]; // 左右腿长度控制器
//PID_Info_TypeDef PID_Leg_Length_R;   //腿长横滚补偿
PID_Info_TypeDef PID_Leg_Roll_F;//横滚补偿力




PID_Info_TypeDef PID_Yaw[2];          // 偏航控制器（位置+速度）
//PID_Info_TypeDef PID_Stool[2]; // 驱动轮控制器

//调试区----------------------------------------------------------------------------------------------


float Test_Theta = 0;                           //测试用的摆杆倾角--0度--摆杆垂直地面

//float Joint_Angle_offset_Num = 0.635f;          //机械安装偏移补偿(并联腿)

float Test_Vmc_Target_L0_Chassis_High   = 0.33f;//高底盘
float Test_Vmc_Target_L0_Chassis_Normal = 0.20f;//正常高度底盘

Control_Info_Typedef Control_Info ={
//腿长控制参数==================================================
	.Base_Leg_Length_Low 		= 0.20f,//基础低腿长
	.Base_Leg_Length_High   	= 0.31f,//基础高腿长
//横滚控制参数==================================================
	.Roll ={
		.Distance_Two_Wheel = 0.4157f,//轮机械间距，单位米
		.Offset = 0.0130947f,// //陀螺仪角度补偿
        .Target_Angle = 0.0f,////目标横滚角

	},
//================================================================	
//腿的机械参数，固定值--如果是新车，这一块要改
.L_Leg_Info = {

	.Biased = {
		.L_Thigh_Link = 0.11760f,//大腿连杆，连杆直驱--动力长后杆长度（AH）
		.L_Calf_Link  = 0.098f,//小腿连杆(仿生机器人领域标准术语）动力短前杆长度（AD）,连杆传动
		.K = 0.4558139f,//连杆长度比K=AD/AH

		
	},//偏置并联
	.Gravity_Compensation = 42.f,//重力的一半
	//.Link_Gravity_Compensation_Angle = 0.4701917f,//连杆重心补偿角度
	.Link_Gravity_Compensation_Angle = 0.0f,//连杆重心补偿角度
	.Phi_Comp_Angle =  0.007f,//当车平衡时，陀螺仪读出来的数据
//.Phi_Comp_Angle =  0.0f,//当车平衡时，陀螺仪读出来的数据
},//单腿测试



//右腿
.R_Leg_Info = {

	.Biased = {
		.L_Thigh_Link = 0.11760f,//大腿连杆，连杆直驱--动力长后杆长度（AH）
		.L_Calf_Link  = 0.098f,//小腿连杆(仿生机器人领域标准术语）动力短前杆长度（AD）,连杆传动
		.K = 0.4558139f,//连杆长度比K=AD/AH

		
	},//2
	.Gravity_Compensation = 42.f,
    .Link_Gravity_Compensation_Angle = 0.4701917f,
.Phi_Comp_Angle =  0.007f,//当车平衡时，陀螺仪读出来的数据
	//.Phi_Comp_Angle =  0.0f,//当车平衡时，陀螺仪读出来的数据
},




};   


//函数区-------------------------------------------------------------------------------------------
static void Pid_Init(Control_Info_Typedef *Control_Info);
/*清零所有状态量/设置默认PID参数/配置初始工作模式/分配内存资源/系统启动时调用一次*/
static void Check_Low_Voltage_Beep(Control_Info_Typedef *Control_Info);//1.检查电池（低电压警告）
static void Mode_Update(Control_Info_Typedef *Control_Info);           //2.模式更新
static void Joint_Angle_Offset(Control_Info_Typedef *Control_Info);    //3.看腿的形状（读关节角度，并映射到VMC坐标系）
static void VMC_Calculate(Control_Info_Typedef *Control_Info);         //4.知道形状后，连杆正运动学解算，求简化腿腿长L0和摆角Phi0，轮子速度
static void LQR_K_Update(Control_Info_Typedef *Control_Info);          //5.得到简化腿长后，更新LQR控制器的增益矩阵K

static void Measure_Update(Control_Info_Typedef *Control_Info);        //6.更新传感器数据和目标值

//底盘移动控制--前进后退（X轴），左右转（YAW轴）
static void Chassis_Move_Control(Control_Info_Typedef *Control_Info);
//底盘高度控制
static void Chassis_Height_Control(Control_Info_Typedef *Control_Info);
//底盘横滚控制
static void Chassis_Roll_Control(Control_Info_Typedef *Control_Info);
//腿长控制
static void Leg_Length_Control(Control_Info_Typedef *Control_Info);
//static void Target_Update(Control_Info_Typedef *Control_Info);
static void LQR_X_Update(Control_Info_Typedef *Control_Info);			//7.更新状态向量差值，为LQR控制器u = -K·X提供输入向量（值）
static void VMC_Measure_F_Tp_Calculate(Control_Info_Typedef *Control_Info);//8.求简化腿的顶力（F）和扭矩(Tp)

static void LQR_T_Tp_Calculate(Control_Info_Typedef *Control_Info);		//9.自适应的LQR，根据是否在空中来限制LQR的作用

static void Comprehensive_F_Calculate(Control_Info_Typedef *Control_Info);//10.综合计算
static void Joint_Tourgue_Calculate(Control_Info_Typedef *Control_Info);//11.实际向电机发送数据的计算



TickType_t Control_Task_SysTick = 0;


void Control_Task(void const * argument)
{
  /*只跑一次的*/
  
 	Pid_Init(&Control_Info);
	
 /* Infinite loop */
	for(;;)
  {
		Control_Task_SysTick = osKernelSysTick();
		//电池
		Check_Low_Voltage_Beep(&Control_Info);
		//开关
		//上级函数
        Mode_Update(&Control_Info);
		//K
		//传感器-----------------------
		Joint_Angle_Offset(&Control_Info);
		//解码器
		//从真实形状转化为简化模型，方便之后的目标控制
	    VMC_Calculate(&Control_Info);
		//上级函数--------------------------------
		 //转化到二阶简化模型之后，按照二阶独轮车的模型进行控制
		LQR_K_Update(&Control_Info);
		//传感器函数
		//测量与目标
	    Measure_Update(&Control_Info);
	//=======================================================
		//底盘移动控制
		Chassis_Move_Control(&Control_Info);
		//底盘高度控制
		Chassis_Height_Control(&Control_Info);
		//底盘横滚控制
		Chassis_Roll_Control(&Control_Info);

		//腿长控制
		Leg_Length_Control(&Control_Info);

	//==================================================================================================
		//上级函数
		//Target_Update(&Control_Info);
		//简化模型的目标控制完成之后，开始为转化到真实的复杂小车结构做准备
		//整个过程有点像解码器和编译器
		//解码器把来自真实世界的模型数据转化为简化模型的数据，然后在上位机编写控制逻辑算法，编写完成之后，
		//解码器把简化模型的数据转化为真实世界的数据
		 //解码器函数
		//力
	      VMC_Measure_F_Tp_Calculate(&Control_Info);	
        LQR_X_Update(&Control_Info);
		LQR_T_Tp_Calculate(&Control_Info);
		Comprehensive_F_Calculate(&Control_Info);
		//输出
		Joint_Tourgue_Calculate(&Control_Info);



//vofa区==============================================================================================
         //USART_Vofa_Justfloat_Transmit(Control_Info.R_Leg_Info.Measure.Chassis_Velocity,Control_Info.L_Leg_Info.Measure.Chassis_Velocity,0,0);
//USART_Vofa_Justfloat_Transmit(-INS_Info.Angle[2],INS_Info.Yaw_Angle,INS_Info.Roll_Angle,INS_Info.Roll_Angle);
  //USART_Vofa_Justfloat_Transmit(Control_Info.L_Leg_Info.Measure.Chassis_Position,Control_Info.L_Leg_Info.Measure.Phi,Control_Info.L_Leg_Info.Measure.Theta,Control_Info.L_Leg_Info.Measure.F);
	
//	USART_Vofa_Justfloat_Transmit((Control_Info.L_Leg_Info.Measure.Phi *RadiansToDegrees),Control_Info.L_Leg_Info.Moment.Balance_Tp,Control_Info.L_Leg_Info.SendValue.Current,(Control_Info.L_Leg_Info.Measure.Theta *RadiansToDegrees));
	//USART_Vofa_Justfloat_Transmit((Control_Info.L_Leg_Info.Measure.Phi *RadiansToDegrees),Control_Info.L_Leg_Info.Moment.Balance_Tp,Control_Info.L_Leg_Info.SendValue.Current,Control_Info.L_Leg_Info.F);
		//USART_Vofa_Justfloat_Transmit(INS_Info.Roll_Angle,Control_Info.L_Leg_Info.Moment.Roll_F,Control_Info.R_Leg_Info.Moment.Roll_F,Control_Info.L_Leg_Info.Moment.Leg_Length_F);
	//USART_Vofa_Justfloat_Transmit(Control_Info.L_Leg_Info.Measure.Chassis_Velocity,(-remote_ctrl.rc.ch[3] * 0.0031),Control_Info.R_Leg_Info.Measure.Chassis_Velocity,Control_Info.L_Leg_Info.Measure.Chassis_Position);
	
	USART_Vofa_Justfloat_Transmit(Control_Info.Target_Velocity,  Control_Info.R_Leg_Info.Measure.Chassis_Velocity,Control_Info.L_Leg_Info.Measure.Chassis_Velocity,Control_Info.L_Leg_Info.Measure.Chassis_Position);
//USART_Vofa_Justfloat_Transmit((Control_Info.L_Leg_Info.Velocity.Wheel-Control_Info.R_Leg_Info.Velocity.Wheel),(Control_Info.L_Leg_Info.Velocity.W-Control_Info.R_Leg_Info.Velocity.W),(Control_Info.L_Leg_Info.Velocity.X-Control_Info.R_Leg_Info.Velocity.X),0);
//=====================================================================================================	
	osDelayUntil(&Control_Task_SysTick,1);
  }
}
  /* USER CODE END Control_Task */
 static uint32_t Tick = 0;

static void Check_Low_Voltage_Beep(Control_Info_Typedef *Control_Info){
 
	 Control_Info->VDC = USER_ADC_Voltage_Update();
 	
//	 if(Control_Info->VDC < 22.f){//如果电压低于22V
//	    Tick++;
//		// 触发蜂鸣器报警模式（0.1s开/0.1s关，循环3次）
//		 if(Tick < 100){
//		    TIM12->CCR2 = 1000;
//		 }else if(Tick > 100 && Tick <200){
//		 	TIM12->CCR2 = 0;
//		 }else if(Tick > 200 && Tick < 300){
//		 	TIM12->CCR2 = 1000;
//		 }else if(Tick > 300 && Tick < 400){
//			TIM12->CCR2 = 0;
//		 }else if(Tick > 400 && Tick < 500){
//			TIM12->CCR2 = 1000;
//		 }else if(Tick > 500 && Tick < 600){
//			TIM12->CCR2 = 0;
//     }else if(Tick > 1000){
//			 Tick = 0;
//		 }
//		}else if(Control_Info->VDC >= 22.f){
//	 
//			TIM12->CCR2 = 0;
//			
//	  }
  }
//PID初始化
static void Pid_Init(Control_Info_Typedef *Control_Info){

	//腿长PID初始化
    PID_Init(&PID_Leg_length_F[0],PID_POSITION,PID_Leg_Length_F_Param);
    PID_Init(&PID_Leg_length_F[1],PID_POSITION,PID_Leg_Length_F_Param);
	//腿长横滚补偿PID初始化
	//PID_Init(&PID_Leg_length_F,PID_POSITION,PID_Leg_Length_R_Param);
	//腿长横滚补偿力PID初始化
	PID_Init(&PID_Leg_Roll_F,PID_POSITION,PID_Leg_Roll_F_Param);
    // 初始化腿部协调PID控制器
	PID_Init(&PID_Leg_Coordinate, PID_POSITION, PID_Leg_Coordinate_param);




	// 初始化偏航PID控制器（位置环+速度环）
	PID_Init(&PID_Yaw[0],PID_POSITION,PID_Yaw_P_pama);
	PID_Init(&PID_Yaw[1],PID_POSITION,PID_Yaw_V_pama);
	
}


static void Mode_Update(Control_Info_Typedef *Control_Info){


   if(remote_ctrl.rc.s[1] == 3 || remote_ctrl.rc.s[1]){
		 /*逻辑冗余：由于||(或运算符)的存在，当remote_ctrl.rc.s[1] == 3成立时，第二个条件不会被执行（短路求值）*/
	 
	   Control_Info->Init.IF_Begin_Init = 1;
	 if(remote_ctrl.rc.s[1] == 2){
	 Control_Info->Init.IF_Begin_Init = 0;
	 Control_Info->Chassis_Situation = CHASSIS_WEAK;
	 }
	 
	 }else{
	 
	  	Control_Info->Init.IF_Begin_Init = 0;
      if(Control_Info->Chassis_Situation == CHASSIS_BALANCE) Control_Info->Chassis_Situation = CHASSIS_WEAK; 
	 
	 } 
		 

   if(Control_Info->Init.IF_Begin_Init == 1 && Control_Info->Chassis_Situation == CHASSIS_WEAK){	
	
	        
   if(Control_Info->Init.Joint_Init.IF_Joint_Init == 0){  
	   	
			 if(DM_8009_Motor[0].Data.Position < 0.0f )  
			 Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[0] = 1; else Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[0] = 0;
			 if(DM_8009_Motor[1].Data.Position > -0.21f &&  DM_8009_Motor[1].Data.Position<-0.005f)    
		   Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[1] = 1; else Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[1] = 0; 
			 if(DM_8009_Motor[2].Data.Position < 0.20f&& DM_8009_Motor[2].Data.Position>0.01f)    
			 Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[2] = 1; else Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[2] = 0;
			 if(DM_8009_Motor[3].Data.Position > -0.0f )  
			 Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[3] = 1; else Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[3] = 0;
			
			 if(Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[0] + Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[1] 
			  + Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[2] + Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[3] == 4){  
			
			   Control_Info->Init.Joint_Init.IF_Joint_Init = 1;  
					
				}else  Control_Info->Init.Joint_Init.IF_Joint_Init =0;
					
					Control_Info->L_Leg_Info.Measure.Chassis_Position = 0 ;
          Control_Info->R_Leg_Info.Measure.Chassis_Position = 0 ; 

		}else if(Control_Info->Init.Joint_Init.IF_Joint_Init == 1){
			
			 Control_Info->Chassis_Situation = CHASSIS_BALANCE;
		   	Control_Info->L_Leg_Info.Measure.Chassis_Position = 0 ;
        Control_Info->R_Leg_Info.Measure.Chassis_Position = 0 ; 
				  
		}
				
}

    if(Control_Info->Init.IF_Begin_Init == 0 && Control_Info->Chassis_Situation == CHASSIS_WEAK){

  
			Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[0] = 0;
			Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[1] = 0;
			Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[2] = 0;
			Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[3] = 0;
			Control_Info->Init.Joint_Init.IF_Joint_Init = 0;


    }
}

static void Joint_Angle_Offset(Control_Info_Typedef *Control_Info){


    // //左小腿摆角与0号关节电机的映射(摆角=电机角度)
	 Control_Info->L_Leg_Info.Biased.Calf_Angle 		 =  DM_8009_Motor[0].Data.Position ;//* RadiansToDegrees;
	// //左大腿摆角与1号关节电机的映射（摆角=电机角度）
	 Control_Info->L_Leg_Info.Biased.Thigh_Angle 		 =  PI+ DM_8009_Motor[1].Data.Position;//) * RadiansToDegrees;
	//角速度
		//左大腿
		Control_Info->L_Leg_Info.Biased.Thigh_Angle_Dot = DM_8009_Motor[1].Data.Velocity;
		//左小腿
		Control_Info->L_Leg_Info.Biased.Calf_Angle_Dot  = DM_8009_Motor[0].Data.Velocity;
	//力矩映射
		//左大腿--对应T1
		Control_Info->L_Leg_Info.Biased.T_Thigh = DM_8009_Motor[1].Data.Torque;
		//左小腿---对应T2
		Control_Info->L_Leg_Info.Biased.T_Calf  = DM_8009_Motor[0].Data.Torque;


	//测试用
	//测试用：假设，当腿完全收缩时-也就是到达最低机械限位，时，大腿和小腿连杆可以张到180度（实际是180.03度）

	// //右大腿摆角和2号关节电机的映射(同左腿)
	 Control_Info->R_Leg_Info.Biased.Thigh_Angle 		 =  DM_8009_Motor[2].Data.Position ;//* RadiansToDegrees;
	// //右小腿摆角和3号关节电机的映射
	 Control_Info->R_Leg_Info.Biased.Calf_Angle  		 =  PI+ DM_8009_Motor[3].Data.Position ;//* RadiansToDegrees;
	//角速度
		//右大腿
		Control_Info->R_Leg_Info.Biased.Thigh_Angle_Dot = DM_8009_Motor[2].Data.Velocity;
		//右小腿
		Control_Info->R_Leg_Info.Biased.Calf_Angle_Dot  = DM_8009_Motor[3].Data.Velocity;
	//力矩映射
		//右大腿--对应T2
		Control_Info->R_Leg_Info.Biased.T_Thigh = DM_8009_Motor[2].Data.Torque;
		//右小腿---对应T1
		Control_Info->R_Leg_Info.Biased.T_Calf  = DM_8009_Motor[3].Data.Torque;


}

static void VMC_Calculate(Control_Info_Typedef *Control_Info){

	//a,b,简化表达用
	Control_Info->L_Leg_Info.a = Control_Info->L_Leg_Info.Biased.L_Calf_Link;
	Control_Info->L_Leg_Info.b = Control_Info->L_Leg_Info.Biased.L_Thigh_Link;
	//左夹角差半值=（左小腿摆角-左大腿摆角）/2
	//Control_Info->L_Leg_Info.M = (Control_Info->L_Leg_Info.Biased.Calf_Angle - Control_Info->L_Leg_Info.Biased.Thigh_Angle)/2.f;
	Control_Info->L_Leg_Info.M = -(Control_Info->L_Leg_Info.Biased.Calf_Angle - Control_Info->L_Leg_Info.Biased.Thigh_Angle)/2.f;
	//左夹角和半差=（左小腿摆角+左大腿摆角）/2
	//2025.11.14：修改为负号
	//2.0：改为正号
	Control_Info->L_Leg_Info.N = (Control_Info->L_Leg_Info.Biased.Calf_Angle + Control_Info->L_Leg_Info.Biased.Thigh_Angle)/2.f;
	//s_radicand  
	//中间变量 公式： S_Radicand = b^2 - a^2 sin^2(M)
	//M = (θ_1 - θ_2 )/2
	Control_Info->L_Leg_Info.S_Radicand = Control_Info->L_Leg_Info.b * Control_Info->L_Leg_Info.b - Control_Info->L_Leg_Info.a * Control_Info->L_Leg_Info.a * arm_sin_f32(Control_Info->L_Leg_Info.M) * arm_sin_f32(Control_Info->L_Leg_Info.M);
	//S 中间变量 公式： S = sqrt(S_Radicand)
	arm_sqrt_f32(Control_Info->L_Leg_Info.S_Radicand,&Control_Info->L_Leg_Info.S);
	//t //中间变量 公式： t=a*cosM+S
	Control_Info->L_Leg_Info.t = Control_Info->L_Leg_Info.a * arm_cos_f32(Control_Info->L_Leg_Info.M) + Control_Info->L_Leg_Info.S;
	//A //中间变量 公式： A= (a*t*sinM)/S
	Control_Info->L_Leg_Info.A = (Control_Info->L_Leg_Info.a * Control_Info->L_Leg_Info.t * arm_sin_f32(Control_Info->L_Leg_Info.M)) / Control_Info->L_Leg_Info.S;
    //=============================================================================================================================================================================================================================================

	//建模------------------------------------------------------------------------------------------------------------------------------
	//1.虚拟腿摆角
	Control_Info->L_Leg_Info.Sip_Leg_Angle  = Control_Info->L_Leg_Info.N;
	//虚拟腿长（AJ）AJ = t/K
	Control_Info->L_Leg_Info.Sip_Leg_Length = Control_Info->L_Leg_Info.t / Control_Info->L_Leg_Info.Biased.K;
    //J点速度
	//X轴方向速度 公式： X_J_Dot =(A*cosN*(Thigh_Angle_Dot -Calf_Angle_Dot) - t*sinN*(Thigh_Angle_Dot +Calf_Angle_Dot))/ (2*K)
     Control_Info->L_Leg_Info.X_J_Dot =   (Control_Info->L_Leg_Info.A * arm_cos_f32(Control_Info->L_Leg_Info.N) *(Control_Info->L_Leg_Info.Biased.Thigh_Angle_Dot - Control_Info->L_Leg_Info.Biased.Calf_Angle_Dot)
									     - Control_Info->L_Leg_Info.t * arm_sin_f32(Control_Info->L_Leg_Info.N) *(Control_Info->L_Leg_Info.Biased.Thigh_Angle_Dot + Control_Info->L_Leg_Info.Biased.Calf_Angle_Dot))/ (2.f * Control_Info->L_Leg_Info.Biased.K);
	//Y轴方向速度 公式： Y_J_Dot =(A*sinN*(Thigh_Angle_Dot -Calf_Angle_Dot) + t*cosN*(Thigh_Angle_Dot +Calf_Angle_Dot))/ (2*K)
     Control_Info->L_Leg_Info.Y_J_Dot =   (Control_Info->L_Leg_Info.A * arm_sin_f32(Control_Info->L_Leg_Info.N) *(Control_Info->L_Leg_Info.Biased.Thigh_Angle_Dot - Control_Info->L_Leg_Info.Biased.Calf_Angle_Dot)
									     + Control_Info->L_Leg_Info.t * arm_cos_f32(Control_Info->L_Leg_Info.N) *(Control_Info->L_Leg_Info.Biased.Thigh_Angle_Dot + Control_Info->L_Leg_Info.Biased.Calf_Angle_Dot))/ (2.f * Control_Info->L_Leg_Info.Biased.K);

//右腿---
//存在问题，即对于J点速度的计算，左右腿的结果不一样
//待实车测试验证
	//中间变量=====================================================================================================================================
	//a,b,简化表达用
	Control_Info->R_Leg_Info.a = Control_Info->R_Leg_Info.Biased.L_Calf_Link;
	Control_Info->R_Leg_Info.b = Control_Info->R_Leg_Info.Biased.L_Thigh_Link;
	//右夹角差半值=（右大腿摆角-右小腿摆角）/2
	Control_Info->R_Leg_Info.M = -(Control_Info->R_Leg_Info.Biased.Thigh_Angle - Control_Info->R_Leg_Info.Biased.Calf_Angle)/2.f;
	//右夹角和半差=（右小腿摆角+右大腿摆角）/2
	Control_Info->R_Leg_Info.N = (Control_Info->R_Leg_Info.Biased.Calf_Angle + Control_Info->R_Leg_Info.Biased.Thigh_Angle)/2.f;
	//s_radicand
	//s_radicand  
	//中间变量 公式： S_Radicand = b^2 - a^2 sin^2(M)
	//M = (θ_1 - θ_2 )/2
		//中间变量 公式： S_Radicand = b^2 - a^2 sin^2(M)
	Control_Info->R_Leg_Info.S_Radicand = Control_Info->R_Leg_Info.b * Control_Info->R_Leg_Info.b - Control_Info->R_Leg_Info.a * Control_Info->R_Leg_Info.a * arm_sin_f32(Control_Info->R_Leg_Info.M) * arm_sin_f32(Control_Info->R_Leg_Info.M);
	//S 中间变量 公式： S = sqrt(S_Radicand)
	arm_sqrt_f32(Control_Info->R_Leg_Info.S_Radicand,&Control_Info->R_Leg_Info.S);
	//t //中间变量 公式： t=a*cosM+S
	Control_Info->R_Leg_Info.t = Control_Info->R_Leg_Info.a * arm_cos_f32(Control_Info->R_Leg_Info.M) + Control_Info->R_Leg_Info.S;
	//A //中间变量 公式： A= (a*t*sinM)/S
	Control_Info->R_Leg_Info.A = (Control_Info->R_Leg_Info.a * Control_Info->R_Leg_Info.t * arm_sin_f32(Control_Info->R_Leg_Info.M)) / Control_Info->R_Leg_Info.S;
	//=============================================================================================================================================================================================================================================	
    //建模
	//1.虚拟腿摆角
	Control_Info->R_Leg_Info.Sip_Leg_Angle  = Control_Info->R_Leg_Info.N;
	//虚拟腿长（AJ）AJ = t/K
	Control_Info->R_Leg_Info.Sip_Leg_Length = Control_Info->R_Leg_Info.t / Control_Info->R_Leg_Info.Biased.K;
	//J点速度
	//速度有方向，取小车头前方向为正方向，垂直地面的法向量方向为单独分析单腿速度的Y轴正方向
	//X轴方向速度 公式： X_J_Dot =(A*cosN*(Thigh_Angle_Dot -Calf_Angle_Dot) - t*sinN*(Thigh_Angle_Dot +Calf_Angle_Dot))/ (2*K)
	Control_Info->R_Leg_Info.X_J_Dot =   (Control_Info->R_Leg_Info.A * arm_cos_f32(Control_Info->R_Leg_Info.N) *(Control_Info->R_Leg_Info.Biased.Thigh_Angle_Dot - Control_Info->R_Leg_Info.Biased.Calf_Angle_Dot)
									     - Control_Info->R_Leg_Info.t * arm_sin_f32(Control_Info->R_Leg_Info.N) *(Control_Info->R_Leg_Info.Biased.Thigh_Angle_Dot + Control_Info->R_Leg_Info.Biased.Calf_Angle_Dot))/ (2.f * Control_Info->R_Leg_Info.Biased.K);
	//Y轴方向速度 公式： Y_J_Dot =(A*sinN*(Thigh_Angle_Dot -Calf_Angle_Dot) + t*cosN*(Thigh_Angle_Dot +Calf_Angle_Dot))/ (2*K)
	Control_Info->R_Leg_Info.Y_J_Dot =   (Control_Info->R_Leg_Info.A * arm_sin_f32(Control_Info->R_Leg_Info.N) *(Control_Info->R_Leg_Info.Biased.Thigh_Angle_Dot - Control_Info->R_Leg_Info.Biased.Calf_Angle_Dot)
									     + Control_Info->R_Leg_Info.t * arm_cos_f32(Control_Info->R_Leg_Info.N) *(Control_Info->R_Leg_Info.Biased.Thigh_Angle_Dot + Control_Info->R_Leg_Info.Biased.Calf_Angle_Dot))/ (2.f * Control_Info->R_Leg_Info.Biased.K);



}

static void LQR_K_Update(Control_Info_Typedef *Control_Info){
	//简化表达
	float L_L0; 

L_L0= Control_Info->L_Leg_Info.Sip_Leg_Length;//偏腿

	Control_Info->L_Leg_Info.LQR_K[0][0] =   K11[1]*powf(L_L0,3)   + K11[2]*powf(L_L0,2)    +K11[3]*L_L0    +K11[4];      
	Control_Info->L_Leg_Info.LQR_K[0][1] =   K12[1]*powf(L_L0,3)   + K12[2]*powf(L_L0,2)    +K12[3]*L_L0    +K12[4];
	Control_Info->L_Leg_Info.LQR_K[0][2] =   K13[1]*powf(L_L0,3)   + K13[2]*powf(L_L0,2)    +K13[3]*L_L0    +K13[4];
	Control_Info->L_Leg_Info.LQR_K[0][3] =   K14[1]*powf(L_L0,3)   + K14[2]*powf(L_L0,2)    +K14[3]*L_L0    +K14[4];
	Control_Info->L_Leg_Info.LQR_K[0][4] =   K15[1]*powf(L_L0,3)   + K15[2]*powf(L_L0,2)    +K15[3]*L_L0    +K15[4];
	Control_Info->L_Leg_Info.LQR_K[0][5] =   K16[1]*powf(L_L0,3)   + K16[2]*powf(L_L0,2)    +K16[3]*L_L0    +K16[4];

	Control_Info->L_Leg_Info.LQR_K[1][0] =   K21[1]*powf(L_L0,3)   + K21[2]*powf(L_L0,2)    +K21[3]*L_L0    +K21[4];     
	Control_Info->L_Leg_Info.LQR_K[1][1] =   K22[1]*powf(L_L0,3)   + K22[2]*powf(L_L0,2)    +K22[3]*L_L0    +K22[4];
	Control_Info->L_Leg_Info.LQR_K[1][2] =   K23[1]*powf(L_L0,3)   + K23[2]*powf(L_L0,2)    +K23[3]*L_L0    +K23[4];
	Control_Info->L_Leg_Info.LQR_K[1][3] =   K24[1]*powf(L_L0,3)   + K24[2]*powf(L_L0,2)    +K24[3]*L_L0    +K24[4];
	Control_Info->L_Leg_Info.LQR_K[1][4] =   K25[1]*powf(L_L0,3)   + K25[2]*powf(L_L0,2)    +K25[3]*L_L0    +K25[4];
	Control_Info->L_Leg_Info.LQR_K[1][5] =   K26[1]*powf(L_L0,3)   + K26[2]*powf(L_L0,2)    +K26[3]*L_L0    +K26[4];
//右腿		
	 float R_L0;
	 
	 R_L0= Control_Info->R_Leg_Info.Sip_Leg_Length;//偏腿
	
	 Control_Info->R_Leg_Info.LQR_K[0][0] =   K11[1]*powf(R_L0,3)   + K11[2]*powf(R_L0,2)    +K11[3]*R_L0    +K11[4];      
	 Control_Info->R_Leg_Info.LQR_K[0][1] =   K12[1]*powf(R_L0,3)   + K12[2]*powf(R_L0,2)    +K12[3]*R_L0    +K12[4];
	 Control_Info->R_Leg_Info.LQR_K[0][2] =   K13[1]*powf(R_L0,3)   + K13[2]*powf(R_L0,2)    +K13[3]*R_L0    +K13[4];
	 Control_Info->R_Leg_Info.LQR_K[0][3] =   K14[1]*powf(R_L0,3)   + K14[2]*powf(R_L0,2)    +K14[3]*R_L0    +K14[4];
	 Control_Info->R_Leg_Info.LQR_K[0][4] =   K15[1]*powf(R_L0,3)   + K15[2]*powf(R_L0,2)    +K15[3]*R_L0    +K15[4];
	 Control_Info->R_Leg_Info.LQR_K[0][5] =   K16[1]*powf(R_L0,3)   + K16[2]*powf(R_L0,2)    +K16[3]*R_L0    +K16[4];

	 Control_Info->R_Leg_Info.LQR_K[1][0] =   K21[1]*powf(R_L0,3)   + K21[2]*powf(R_L0,2)    +K21[3]*R_L0    +K21[4];     
	 Control_Info->R_Leg_Info.LQR_K[1][1] =   K22[1]*powf(R_L0,3)   + K22[2]*powf(R_L0,2)    +K22[3]*R_L0    +K22[4];
	 Control_Info->R_Leg_Info.LQR_K[1][2] =   K23[1]*powf(R_L0,3)   + K23[2]*powf(R_L0,2)    +K23[3]*R_L0    +K23[4];
	 Control_Info->R_Leg_Info.LQR_K[1][3] =   K24[1]*powf(R_L0,3)   + K24[2]*powf(R_L0,2)    +K24[3]*R_L0    +K24[4];
	 Control_Info->R_Leg_Info.LQR_K[1][4] =   K25[1]*powf(R_L0,3)   + K25[2]*powf(R_L0,2)    +K25[3]*R_L0    +K25[4];
	 Control_Info->R_Leg_Info.LQR_K[1][5] =   K26[1]*powf(R_L0,3)   + K26[2]*powf(R_L0,2)    +K26[3]*R_L0    +K26[4];


}



static void Measure_Update(Control_Info_Typedef *Control_Info){
//填状态
//横滚

	//左腿
		//身体平衡
	//机体水平倾角
		Control_Info->L_Leg_Info.Measure.Phi       = -INS_Info.Angle[2]-   Control_Info->L_Leg_Info.Phi_Comp_Angle;//注意极性
	//Control_Info->L_Leg_Info.Measure.Phi       = INS_Info.Angle[2]+   Control_Info->L_Leg_Info.Phi_Comp_Angle;//注意极性
	
		Control_Info->L_Leg_Info.Measure.Phi_dot   = -INS_Info.Gyro[0];
		//腿的姿势
		//Control_Info->L_Leg_Info.Measure.Theta     = 	((PI/2) - Control_Info->L_Leg_Info.VMC.Phi0) - Control_Info->L_Leg_Info.Measure.Phi;
		Control_Info->L_Leg_Info.Measure.Theta     =     (	((PI/2) - Control_Info->L_Leg_Info.Sip_Leg_Angle) - Control_Info->L_Leg_Info.Measure.Phi);// +(PI/6.0f);//+ Control_Info->L_Leg_Info.Link_Gravity_Compensation_Angle;
		
		//Control_Info->L_Leg_Info.Measure.Theta_dot = 			 (Control_Info->L_Leg_Info.VMC.X_C_dot * arm_cos_f32( - Control_Info->L_Leg_Info.Measure.Theta)
		//											  		   +  Control_Info->L_Leg_Info.VMC.Y_C_dot * arm_sin_f32( - Control_Info->L_Leg_Info.Measure.Theta))
		//											  		   /  Control_Info->L_Leg_Info.VMC.L0;
		Control_Info->L_Leg_Info.Measure.Theta_dot = 			 (Control_Info->L_Leg_Info.X_J_Dot * arm_cos_f32( - Control_Info->L_Leg_Info.Measure.Theta)
													  		   +  Control_Info->L_Leg_Info.Y_J_Dot * arm_sin_f32( - Control_Info->L_Leg_Info.Measure.Theta))
													  		   /  Control_Info->L_Leg_Info.Sip_Leg_Length;
	
		//右腿
	// 右腿姿态（使用相同IMU数据）	
		Control_Info->R_Leg_Info.Measure.Phi 		  = -INS_Info.Angle[2] - Control_Info->L_Leg_Info.Phi_Comp_Angle;//注意极性
		//Control_Info->R_Leg_Info.Measure.Phi 		  = INS_Info.Angle[2] + Control_Info->L_Leg_Info.Phi_Comp_Angle;//注意极性
		Control_Info->R_Leg_Info.Measure.Phi_dot 	= -INS_Info.Gyro[0];
		
		Control_Info->R_Leg_Info.Measure.Theta 		= (Control_Info->R_Leg_Info.Sip_Leg_Angle -(PI/2)) - Control_Info->R_Leg_Info.Measure.Phi ;//+(PI/6.0f);//+Control_Info->R_Leg_Info.Link_Gravity_Compensation_Angle;
		Control_Info->R_Leg_Info.Measure.Theta_dot  = ( -Control_Info->R_Leg_Info.X_J_Dot     * arm_cos_f32(-Control_Info->R_Leg_Info.Measure.Theta)
                                               		+  Control_Info->R_Leg_Info.Y_J_Dot       * arm_sin_f32(-Control_Info->R_Leg_Info.Measure.Theta))
	                                             	/  Control_Info->R_Leg_Info.Sip_Leg_Length;	
	

Control_Info->L_Leg_Info.Velocity.Wheel = Chassis_Motor[0].Data.Velocity*(PI /30.f)/15.f;
/*计算原理​
W=v wheel− ϕ˙ + θ˙ 
​轮速补偿​：减去底盘旋转引起的速度分量
​腿部运动补偿​：加上虚拟腿摆动产生的速度分量
​目标​：获得底盘在水平方向的真实运动速度*/
Control_Info->L_Leg_Info.Velocity.W     = (Control_Info->L_Leg_Info.Velocity.Wheel - Control_Info->L_Leg_Info.Measure.Phi_dot + Control_Info->L_Leg_Info.Measure.Theta_dot);
//轮半径--0.055f---x--线速度
Control_Info->L_Leg_Info.Velocity.X     = Control_Info->L_Leg_Info.Velocity.W * 0.055f;
//求腿长变化率
Control_Info->L_Leg_Info.Sip_Leg_Length_dot   = Control_Info->L_Leg_Info.Y_J_Dot * arm_cos_f32(Control_Info->L_Leg_Info.Measure.Theta);
/*4. ​底盘速度融合​
采用一阶滞后滤波器融合预测值和测量值：
//估计底盘速度（Body）并进行滤波融合（Predict和Fusion）

预测速度 = 上一周期融合速度 + 加速度×Δt
融合速度 = 0.8×预测速度 + 0.2×当前测量速度---
滤波系数​：0.8（历史数据权重）和 0.2（新数据权重）
​物理意义​：在快速响应和噪声抑制之间取得平衡
*/
Control_Info->L_Leg_Info.Velocity.Body     		  = Control_Info->L_Leg_Info.Velocity.X;
//预测速度
Control_Info->L_Leg_Info.Predict_Velocity  		  = Control_Info->L_Leg_Info.Velocity.Fusion + Control_Info->Accel * 0.001f;
//融合速度
Control_Info->L_Leg_Info.Velocity.Fusion   		  = Control_Info->L_Leg_Info.Velocity.Predict* 0.8f + Control_Info->L_Leg_Info.Velocity.Body * 0.2f;
//底盘的实际速度（来自左腿测量）
Control_Info->L_Leg_Info.Measure.Chassis_Velocity = Control_Info->L_Leg_Info.Velocity.Fusion;
//右腿
  //右腿同理
	Control_Info->R_Leg_Info.Velocity.Wheel 	= -Chassis_Motor[1].Data.Velocity*(3.141593f/30.f)/15.f;

  	Control_Info->R_Leg_Info.Velocity.W     	= (Control_Info->R_Leg_Info.Velocity.Wheel - Control_Info->R_Leg_Info.Measure.Phi_dot + Control_Info->R_Leg_Info.Measure.Theta_dot);

	Control_Info->R_Leg_Info.Velocity.X     	= Control_Info->R_Leg_Info.Velocity.W * 0.055f;
	
	Control_Info->R_Leg_Info.Sip_Leg_Length_dot =  Control_Info->R_Leg_Info.Y_J_Dot * arm_cos_f32(Control_Info->R_Leg_Info.Measure.Theta);

	Control_Info->R_Leg_Info.Velocity.Body 		=    Control_Info->R_Leg_Info.Velocity.X;
	                                          //+ Control_Info->R_Leg_Info.VMC.L0 * Control_Info->R_Leg_Info.Measure.Theta_dot * arm_cos_f32(Control_Info->R_Leg_Info.Measure.Theta)
																						//+ Control_Info->R_Leg_Info.VMC.L0_dot * arm_sin_f32(Control_Info->R_Leg_Info.Measure.Theta);
	      
				
	Control_Info->R_Leg_Info.Velocity.Predict = Control_Info->R_Leg_Info.Velocity.Fusion + Control_Info->Accel*0.001f;	
				
	Control_Info->R_Leg_Info.Velocity.Fusion = 0.8f *	 Control_Info->R_Leg_Info.Velocity.Predict + 	Control_Info->R_Leg_Info.Velocity.Body * 0.2f;

  	Control_Info->R_Leg_Info.Measure.Chassis_Velocity = Control_Info->R_Leg_Info.Velocity.Fusion;
//------------------------------------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//计算整体底盘速度（取左右腿速度平均值）
  	Control_Info->Chassis_Velocity =   (Control_Info->L_Leg_Info.Measure.Chassis_Velocity +  Control_Info->R_Leg_Info.Measure.Chassis_Velocity)/2.f ;
//	7. ​底盘位置更新逻辑
	if(Control_Info->Target_Velocity == 0){//目标速度 == 0
   // 积分计算位置：位置 += 速度 × Δt
	  Control_Info->L_Leg_Info.Measure.Chassis_Position += Control_Info->Chassis_Velocity*0.001f  ;
	
  	Control_Info->R_Leg_Info.Measure.Chassis_Position += Control_Info->Chassis_Velocity*0.001f ;

	}else{
	    // 重置位置（速度控制模式下位置环不工作）
		  Control_Info->L_Leg_Info.Measure.Chassis_Position = 0 ;
	
    	Control_Info->R_Leg_Info.Measure.Chassis_Position = 0 ; 
	
	
	}	
//	6. ​底盘加速度计算
/*
Accel = [
    (-a_y + ω_z² × 0.155 - g × sin(Φ)) × cos(Φ)
    + (a_z - g × cos(Φ)) × sin(Φ)
]

分量解析​：
-a_y：Y轴加速度（传感器坐标系）
ω_z² × 0.155：向心加速度（0.155m为旋转半径）
g × sin(Φ)：重力分量在底盘平面投影
(a_z - g × cos(Φ)) × sin(Φ)：Z轴加速度扣除重力分量后的有效分量
*/			
 Control_Info->Accel =  (float) (( -INS_Info.Accel[1] + powf(INS_Info.Gyro[2],2)*0.155f) - GravityAccel * arm_sin_f32 (-INS_Info.Angle[2])) * arm_cos_f32 (-INS_Info.Angle[2]) + 
	                              (   INS_Info.Accel[2] - GravityAccel* arm_cos_f32 (-INS_Info.Angle[2])) * arm_sin_f32 (-INS_Info.Angle[2]) ; 		

//8. ​特殊状态处理（CHASSIS_WEAK）​​
//当底盘处于虚弱状态（未平衡状态）时：
 if(Control_Info->Chassis_Situation == CHASSIS_WEAK){
// 重置所有状态量为0 
   		Control_Info->L_Leg_Info.Measure.Phi = 0;
	 	  Control_Info->L_Leg_Info.Measure.Phi_dot = 0;
	  	Control_Info->L_Leg_Info.Measure.Chassis_Position = 0;
	  	Control_Info->L_Leg_Info.Measure.Chassis_Velocity = 0;
	 	  Control_Info->L_Leg_Info.Measure.Theta = 0;
	 	  Control_Info->L_Leg_Info.Measure.Theta_dot = 0;
 
	  	Control_Info->R_Leg_Info.Measure.Phi = 0;
	  	Control_Info->R_Leg_Info.Measure.Phi_dot = 0;
	  	Control_Info->R_Leg_Info.Measure.Chassis_Position = 0;
	  	Control_Info->R_Leg_Info.Measure.Chassis_Velocity = 0;
	  	Control_Info->R_Leg_Info.Measure.Theta = 0;
	  	Control_Info->R_Leg_Info.Measure.Theta_dot = 0;
 
	  	Control_Info->Chassis_Velocity = 0;
		//目的​：防止未初始化状态下的错误控制输出
 }



}




//=====================================================================================================================
//底盘移动控制
static void Chassis_Move_Control(Control_Info_Typedef *Control_Info){ 
//响应区

float K_Velocity = 0.001f;//前进响应快慢

float K_Brake = 0.002f;//刹车响应快慢
//前进后退
//移动指令============================================================================================================
if(remote_ctrl.rc.ch[3] != 0 ){//开始控制

Control_Info->Target_Velocity = f_Ramp_Calc(Control_Info->Target_Velocity,-remote_ctrl.rc.ch[3] * 0.00242,K_Velocity);

	//重置位置积分器
	//左右腿位置刷新
	Control_Info->L_Leg_Info.Measure.Chassis_Position = 0;
	Control_Info->R_Leg_Info.Measure.Chassis_Position = 0 ; 

}else if(remote_ctrl.rc.ch[3] == 0){//不移动了，开摆
	//平滑归零
	Control_Info->Target_Velocity = f_Ramp_Calc(Control_Info->Target_Velocity,0,K_Brake);//0.002f--慢刹车
}
//2.约束(行车不规范，亲人两行泪)
//此处限速1.6m/s
VAL_LIMIT(Control_Info->Target_Velocity,-1.6f,1.6f);

//=====================================================================================================================
//左右转
//滤波区
//旋转响应快慢
float K_Yaw_P = 0.2f;
//======================================================
// /*

// 主要是修改Yaw_Err的来源
// 将遥控器的摇杆值（范围：-660到660）代替DM_Yaw_Motor.Data.Position*/
 Control_Info->Yaw_Err =  f_Ramp_Calc(Control_Info->Yaw_Err , (remote_ctrl.rc.ch[2] * RemoteToDegrees),K_Yaw_P);
 
// 	//计算偏航角误差 Yaw_Err，通过将电机位置从弧度转换为角度后取负值
 	//Control_Info->Yaw_Err = 0.f - DM_Yaw_Motor.Data.Position * RadiansToDegrees ;
// 	//将偏航角误差限制在[-180~180]度
 	if (Control_Info->Yaw_Err >= 180.f) 		Control_Info->Yaw_Err -= 360.f;
 	else if (Control_Info->Yaw_Err <= -180.f)   Control_Info->Yaw_Err += 360.f;

// //yaw轴串级PID控制
// //上级：目标yaw轴角度偏差为0，输入Control_Info->Yaw_Err，输出：PID_Yaw[0].Output
 	PID_Calculate(&PID_Yaw[0], 0, Control_Info->Yaw_Err);
// //下级：目标yaw轴角速度为0，输入INS_Info.Yaw_Gyro，输出：PID_Yaw[1].Output
 	PID_Calculate(&PID_Yaw[1],PID_Yaw[0].Output,INS_Info.Yaw_Gyro);
// //转向控制	
// //将偏航转向力矩分别施加到左右腿上，方向相反实现转向。
 	Control_Info->L_Leg_Info.Moment.Turn_T =  PID_Yaw[1].Output;
 	Control_Info->R_Leg_Info.Moment.Turn_T = -PID_Yaw[1].Output;
}


//======================================================================================================================

//底盘高度控制
static void Chassis_Height_Control(Control_Info_Typedef *Control_Info){ 
	//切换底盘高度响应快慢
float K_Height = 0.00035f;
/*腿长控制*/
//首先，根据遥控器指令设置基础腿长

if(remote_ctrl.rc.s[1] == 1){//高腿长指令
	 Control_Info->L_Leg_Info.Base_Leg_Length = f_Ramp_Calc (Control_Info->L_Leg_Info.Base_Leg_Length,Control_Info->Base_Leg_Length_High,K_Height);
	 Control_Info->R_Leg_Info.Base_Leg_Length = f_Ramp_Calc (Control_Info->R_Leg_Info.Base_Leg_Length,Control_Info->Base_Leg_Length_High,K_Height);

//	Control_Info->L_Leg_Info.Base_Leg_Length = Control_Info->Base_Leg_Length_High;
//	Control_Info->R_Leg_Info.Base_Leg_Length = Control_Info->Base_Leg_Length_High;
}else {//低腿长指令

	 Control_Info->L_Leg_Info.Base_Leg_Length = f_Ramp_Calc (Control_Info->L_Leg_Info.Base_Leg_Length,Control_Info->Base_Leg_Length_Low,K_Height);
	 Control_Info->R_Leg_Info.Base_Leg_Length = f_Ramp_Calc (Control_Info->R_Leg_Info.Base_Leg_Length,Control_Info->Base_Leg_Length_Low,K_Height);
}
}
//底盘横滚控制
static void Chassis_Roll_Control(Control_Info_Typedef *Control_Info){ 
//========================================================================================================
//设置基础腿长后，引入横滚补偿，保持机体Roll轴平衡
/*机器人发生侧倾时通过调节腿长和推力来恢复平衡
其中，调节腿长的调节量受传感器测量的ROLL角度和几何关系的影响，
而推力补偿量受传感器测量的ROLL角度影响，采用PID控制。
*/
//首先，先定义方向
/*规定尾部剖面图为正视图，对于机体横滚角度方向：逆时针为正
对于机体因腿长差产生的横滚角方向：逆时针为正，
水平向右为零点方向

*/
//首先，当前左右实际腿长差：左腿 - 右腿
Control_Info->Roll.Length_Diff = Control_Info->L_Leg_Info.Sip_Leg_Length - Control_Info->R_Leg_Info.Sip_Leg_Length;
//2，当前机体横滚角,加一个一阶滤波
Control_Info->Roll.Angle     = f_Ramp_Calc(Control_Info->Roll.Angle, (-INS_Info.Angle[1] -0.0291f),0.003f);
//3，机体因腿长差而产生的横滚角度的正切值
Control_Info->Roll.Tan_Length_Diff_Angle = Control_Info->Roll.Length_Diff/Control_Info->Roll.Distance_Two_Wheel;
//4，当前机体横滚角的正切值
Control_Info->Roll.Tan_Angle = arm_sin_f32(Control_Info->Roll.Angle)/arm_cos_f32(Control_Info->Roll.Angle);
//5,机体接触坡的实际坡度，为机体横滚角 + 因腿长差产生的横滚角
//现在来求实际坡度角的正切值
Control_Info->Roll.Tan_Slope_Angle = (Control_Info->Roll.Tan_Angle + Control_Info->Roll.Tan_Length_Diff_Angle)  / (1.0f - (Control_Info->Roll.Tan_Angle * Control_Info->Roll.Tan_Length_Diff_Angle));

//来求左右横滚补偿角
Control_Info->L_Leg_Info.Roll_Leg_Length = (Control_Info->Roll.Tan_Slope_Angle * Control_Info->Roll.Distance_Two_Wheel)/2.0f;
Control_Info->R_Leg_Info.Roll_Leg_Length = -(Control_Info->Roll.Tan_Slope_Angle * Control_Info->Roll.Distance_Two_Wheel)/2.0f;
	 


	 //推力横滚补偿
	     Control_Info->L_Leg_Info.Moment.Roll_F =     -PID_Calculate(&PID_Leg_Roll_F,0.f,(Control_Info->Roll.Angle));
         Control_Info->R_Leg_Info.Moment.Roll_F =    PID_Calculate(&PID_Leg_Roll_F, 0.f,(Control_Info->Roll.Angle));

}

//腿长控制
static void Leg_Length_Control(Control_Info_Typedef *Control_Info){ 
float K_Length = 0.35f;
	//最后，计算最终期望的左右总腿长
Control_Info->L_Leg_Info.Total_Leg_Length =   f_Ramp_Calc (Control_Info->L_Leg_Info.Total_Leg_Length,( Control_Info->L_Leg_Info.Base_Leg_Length + Control_Info->L_Leg_Info.Roll_Leg_Length),K_Length);
Control_Info->R_Leg_Info.Total_Leg_Length =   f_Ramp_Calc  (Control_Info->R_Leg_Info.Total_Leg_Length , (Control_Info->R_Leg_Info.Base_Leg_Length + Control_Info->R_Leg_Info.Roll_Leg_Length),K_Length);

//善后，对左右腿长进行限幅	 
 	 VAL_LIMIT(Control_Info->L_Leg_Info.Total_Leg_Length,0.14,0.32);
 	 VAL_LIMIT(Control_Info->R_Leg_Info.Total_Leg_Length,0.14,0.32);


   Control_Info->L_Leg_Info.Moment.Leg_Length_F =  PID_Calculate(&PID_Leg_length_F[0],Control_Info->L_Leg_Info.Total_Leg_Length,Control_Info->L_Leg_Info.Sip_Leg_Length);
   Control_Info->R_Leg_Info.Moment.Leg_Length_F =  PID_Calculate(&PID_Leg_length_F[1],Control_Info->R_Leg_Info.Total_Leg_Length,Control_Info->R_Leg_Info.Sip_Leg_Length);


   
 //设置初始重力补偿值为 100。f
 Control_Info->R_Leg_Info.Gravity_Compensation = 42.f;
 Control_Info->L_Leg_Info.Gravity_Compensation = 42.f;
 //如果左腿处于支撑状态，则取消其滚动补偿，并提高重力补偿至 140。
 	if(Control_Info->L_Leg_Info.Support.Flag == 1){
	   
 		//Control_Info->L_Leg_Info.Moment.Roll_F = 0;
 	  //Control_Info->L_Leg_Info.Gravity_Compensation = 0.f;
 	}
 	//同理处理右腿支撑情况。
 	if(Control_Info->R_Leg_Info.Support.Flag == 1){
	
 		//Control_Info->R_Leg_Info.Moment.Roll_F = 0;
 		//Control_Info->R_Leg_Info.Gravity_Compensation = 0.f;

	
 	}

	
	
// 	//综合各部分力矩得到左右腿总的驱动力。
// 	//公式：F =  腿长补偿力（L_Leg_Info.Moment.Leg_Length_F）
// 	//         + 横滚补偿力（L_Leg_Info.Moment.Roll_F ）
// 	//         + 重力补偿（L_Leg_Info.Gravity_Compensation;）
// 	//分析：已知，重力的方向垂直地面向下，且Gravity_Compensation的值为正，故综合力的方向向下，
// 	//同重力方向
 	 Control_Info->L_Leg_Info.F = Control_Info->L_Leg_Info.Moment.Leg_Length_F+ Control_Info->L_Leg_Info.Moment.Roll_F  +  Control_Info->L_Leg_Info.Gravity_Compensation;  
 	 Control_Info->R_Leg_Info.F = Control_Info->R_Leg_Info.Moment.Leg_Length_F+ Control_Info->R_Leg_Info.Moment.Roll_F  +  Control_Info->R_Leg_Info.Gravity_Compensation;
	
//  	//同重力方向
//  	Control_Info->L_Leg_Info.F = Control_Info->L_Leg_Info.Moment.Leg_Length_F +  Control_Info->L_Leg_Info.Gravity_Compensation;  
//  	Control_Info->R_Leg_Info.F = Control_Info->R_Leg_Info.Moment.Leg_Length_F +  Control_Info->R_Leg_Info.Gravity_Compensation;

}


static void LQR_X_Update(Control_Info_Typedef *Control_Info){

//左腿
	Control_Info->L_Leg_Info.LQR_X[0] = (Control_Info->L_Leg_Info.Target.Theta            - Control_Info->L_Leg_Info.Measure.Theta);
	Control_Info->L_Leg_Info.LQR_X[1] = (Control_Info->L_Leg_Info.Target.Theta_dot        - Control_Info->L_Leg_Info.Measure.Theta_dot);
	Control_Info->L_Leg_Info.LQR_X[2] = (Control_Info->L_Leg_Info.Target.Chassis_Position - Control_Info->L_Leg_Info.Measure.Chassis_Position);
	Control_Info->L_Leg_Info.LQR_X[3] = (Control_Info->Target_Velocity                    - Control_Info->Chassis_Velocity);
	Control_Info->L_Leg_Info.LQR_X[4] = (Control_Info->L_Leg_Info.Target.Phi              - Control_Info->L_Leg_Info.Measure.Phi);
	Control_Info->L_Leg_Info.LQR_X[5] = (Control_Info->L_Leg_Info.Target.Phi_dot          - Control_Info->L_Leg_Info.Measure.Phi_dot);
//右腿
//右腿同理
	Control_Info->R_Leg_Info.LQR_X[0] = (Control_Info->R_Leg_Info.Target.Theta 			      - Control_Info->R_Leg_Info.Measure.Theta);
	Control_Info->R_Leg_Info.LQR_X[1] = (Control_Info->R_Leg_Info.Target.Theta_dot        - Control_Info->R_Leg_Info.Measure.Theta_dot);
	Control_Info->R_Leg_Info.LQR_X[2] = (Control_Info->R_Leg_Info.Target.Chassis_Position - Control_Info->R_Leg_Info.Measure.Chassis_Position) ;
	Control_Info->R_Leg_Info.LQR_X[3] = (Control_Info->Target_Velocity                    - Control_Info->Chassis_Velocity );
	Control_Info->R_Leg_Info.LQR_X[4] = (Control_Info->R_Leg_Info.Target.Phi              - Control_Info->R_Leg_Info.Measure.Phi);
	Control_Info->R_Leg_Info.LQR_X[5] = (Control_Info->R_Leg_Info.Target.Phi_dot          - Control_Info->R_Leg_Info.Measure.Phi_dot);




}




static void VMC_Measure_F_Tp_Calculate(Control_Info_Typedef *Control_Info){
//偏腿
	//左腿 
		//F    //公式：F=K(T2-T1)/A            //(T_左小腿 - T_左大腿) ps:公式相同，但左右腿在单独分析时，大腿的朝向不同，故需单独考虑，具体见整体模型简图
		Control_Info->L_Leg_Info.Measure.F  = (Control_Info->L_Leg_Info.Biased.K * (Control_Info->L_Leg_Info.Biased.T_Calf - Control_Info->L_Leg_Info.Biased.T_Thigh))/Control_Info->L_Leg_Info.A;
		//Tp  //公式：Tp=T1+T2
	//修改：改为负号
		Control_Info->L_Leg_Info.Measure.Tp = (Control_Info->L_Leg_Info.Biased.T_Thigh + Control_Info->L_Leg_Info.Biased.T_Calf);

	//右腿
		//F   //公式：F=K(T2-T1)/A  = （T2-T1）/(A/K)           //(T_右大腿 - T_右小腿)
		Control_Info->R_Leg_Info.Measure.F  = (Control_Info->R_Leg_Info.Biased.K * (Control_Info->R_Leg_Info.Biased.T_Thigh - Control_Info->R_Leg_Info.Biased.T_Calf))/Control_Info->R_Leg_Info.A;
		//Tp  //公式：Tp=T1+T2
	//2025.11.18--修改，Tp测量值改为负
		Control_Info->R_Leg_Info.Measure.Tp = (Control_Info->R_Leg_Info.Biased.T_Thigh + Control_Info->R_Leg_Info.Biased.T_Calf);
  											

//离地检测灯

	//当车平衡时
	if(Control_Info->Chassis_Situation == CHASSIS_BALANCE){
	//扭矩不变
	//左腿
	Control_Info->L_Leg_Info.Measure.Tp = Control_Info->L_Leg_Info.Measure.Tp;

	//求支持力
    Control_Info->L_Leg_Info.Support.FN =      Control_Info->L_Leg_Info.Measure.F  * arm_cos_f32(Control_Info->L_Leg_Info.Measure.Theta)+  (  (Control_Info->L_Leg_Info.Measure.Tp * arm_sin_f32(Control_Info->L_Leg_Info.Measure.Theta))
									                              /     Control_Info->L_Leg_Info.Sip_Leg_Length);
		
		
		
//右腿
    Control_Info->R_Leg_Info.Measure.Tp = Control_Info->R_Leg_Info.Measure.Tp;//注意极性

																					//注意：2025.11.15.12：54(Control_Info->R_Leg_Info.Measure.Tp*arm_sin_f32(Control_Info->R_Leg_Info.Measure.Theta)加了一个负号
	Control_Info->R_Leg_Info.Support.FN    =  Control_Info->R_Leg_Info.Measure.F * arm_cos_f32(Control_Info->R_Leg_Info.Measure.Theta)
										                        +(-(Control_Info->R_Leg_Info.Measure.Tp*arm_sin_f32(Control_Info->R_Leg_Info.Measure.Theta))/Control_Info->R_Leg_Info.Sip_Leg_Length);

		//检测标志
			//左腿
			Control_Info->L_Leg_Info.Support.Flag = (Control_Info->L_Leg_Info.Support.FN < 42.f);
			//右腿
			Control_Info->R_Leg_Info.Support.Flag = (Control_Info->R_Leg_Info.Support.FN < 42.f);
	//车不平衡时
	}else {
	      Control_Info->L_Leg_Info.Support.Flag = 0;//红灯
       Control_Info->R_Leg_Info.Support.Flag = 0;
	   Control_Info->L_Leg_Info.Support.FN   = 72.f;
	   Control_Info->R_Leg_Info.Support.FN   =72.f;


	}




}


//在VMC_Measure_F_Tp_Calculate得知现在车是否离地，之后根据支撑状态自适应调整控制策略，并生成最终的控制输出（力矩T和扭矩Tp
//不用改，并腿/偏腿通用
static void LQR_T_Tp_Calculate(Control_Info_Typedef *Control_Info){
//1. 支撑状态自适应调整	

	//当腿处于离地状态时（着地），​禁用水平方向控制​（轮子力矩T相关增益）
    //​部分禁用垂直方向控制（关节扭矩Tp相关增益）
	if(Control_Info->L_Leg_Info.Support.Flag == 1){

 		// 清零左腿LQR增益矩阵的第一行（力矩T相关）
		Control_Info->L_Leg_Info.LQR_K[0][0] = 0;
		Control_Info->L_Leg_Info.LQR_K[0][1] = 0; 
		Control_Info->L_Leg_Info.LQR_K[0][2] = 0;
		Control_Info->L_Leg_Info.LQR_K[0][3] = 0; 
		Control_Info->L_Leg_Info.LQR_K[0][4] = 0;
		Control_Info->L_Leg_Info.LQR_K[0][5] = 0; 
	

	}
	//右腿
	if(Control_Info->R_Leg_Info.Support.Flag == 1){
	
		Control_Info->R_Leg_Info.LQR_K[0][0] = 0;
		Control_Info->R_Leg_Info.LQR_K[0][1] = 0; 
		Control_Info->R_Leg_Info.LQR_K[0][2] = 0;
		Control_Info->R_Leg_Info.LQR_K[0][3] = 0; 
		Control_Info->R_Leg_Info.LQR_K[0][4] = 0;
		Control_Info->R_Leg_Info.LQR_K[0][5] = 0; 



	}

//2. LQR控制输出计算---------------------------------------------------------
//输出 = K · X
//K = 增益矩阵
//X = 状态误差向量
//--------------------------------------------------------------------------
//
	Control_Info->L_Leg_Info.LQR_Output[0][0] = Control_Info->L_Leg_Info.LQR_X[0]*Control_Info->L_Leg_Info.LQR_K[0][0];
	Control_Info->L_Leg_Info.LQR_Output[0][1] = Control_Info->L_Leg_Info.LQR_X[1]*Control_Info->L_Leg_Info.LQR_K[0][1];
	Control_Info->L_Leg_Info.LQR_Output[0][2] = Control_Info->L_Leg_Info.LQR_X[2]*Control_Info->L_Leg_Info.LQR_K[0][2];
	Control_Info->L_Leg_Info.LQR_Output[0][3] = Control_Info->L_Leg_Info.LQR_X[3]*Control_Info->L_Leg_Info.LQR_K[0][3];
	Control_Info->L_Leg_Info.LQR_Output[0][4] = Control_Info->L_Leg_Info.LQR_X[4]*Control_Info->L_Leg_Info.LQR_K[0][4];
	Control_Info->L_Leg_Info.LQR_Output[0][5] = Control_Info->L_Leg_Info.LQR_X[5]*Control_Info->L_Leg_Info.LQR_K[0][5];

  Control_Info->L_Leg_Info.Moment.Balance_T =   Control_Info->L_Leg_Info.LQR_Output[0][0] + Control_Info->L_Leg_Info.LQR_Output[0][1] + Control_Info->L_Leg_Info.LQR_Output[0][2]
                                              + Control_Info->L_Leg_Info.LQR_Output[0][3] + Control_Info->L_Leg_Info.LQR_Output[0][4] + Control_Info->L_Leg_Info.LQR_Output[0][5];

	
	Control_Info->R_Leg_Info.LQR_Output[0][0] = Control_Info->R_Leg_Info.LQR_X[0]*Control_Info->R_Leg_Info.LQR_K[0][0];
	Control_Info->R_Leg_Info.LQR_Output[0][1] = Control_Info->R_Leg_Info.LQR_X[1]*Control_Info->R_Leg_Info.LQR_K[0][1];
	Control_Info->R_Leg_Info.LQR_Output[0][2] = Control_Info->R_Leg_Info.LQR_X[2]*Control_Info->R_Leg_Info.LQR_K[0][2];
	Control_Info->R_Leg_Info.LQR_Output[0][3] = Control_Info->R_Leg_Info.LQR_X[3]*Control_Info->R_Leg_Info.LQR_K[0][3];
	Control_Info->R_Leg_Info.LQR_Output[0][4] = Control_Info->R_Leg_Info.LQR_X[4]*Control_Info->R_Leg_Info.LQR_K[0][4];
	Control_Info->R_Leg_Info.LQR_Output[0][5] = Control_Info->R_Leg_Info.LQR_X[5]*Control_Info->R_Leg_Info.LQR_K[0][5];	
	

	Control_Info->R_Leg_Info.Moment.Balance_T =   Control_Info->R_Leg_Info.LQR_Output[0][0] + Control_Info->R_Leg_Info.LQR_Output[0][1] + Control_Info->R_Leg_Info.LQR_Output[0][2]
                                              + Control_Info->R_Leg_Info.LQR_Output[0][3] + Control_Info->R_Leg_Info.LQR_Output[0][4] + Control_Info->R_Leg_Info.LQR_Output[0][5];


	Control_Info->L_Leg_Info.LQR_Output[1][0] = Control_Info->L_Leg_Info.LQR_X[0]*Control_Info->L_Leg_Info.LQR_K[1][0];
	Control_Info->L_Leg_Info.LQR_Output[1][1] = Control_Info->L_Leg_Info.LQR_X[1]*Control_Info->L_Leg_Info.LQR_K[1][1];
	Control_Info->L_Leg_Info.LQR_Output[1][2] = Control_Info->L_Leg_Info.LQR_X[2]*Control_Info->L_Leg_Info.LQR_K[1][2];
	Control_Info->L_Leg_Info.LQR_Output[1][3] = Control_Info->L_Leg_Info.LQR_X[3]*Control_Info->L_Leg_Info.LQR_K[1][3];
	Control_Info->L_Leg_Info.LQR_Output[1][4] = Control_Info->L_Leg_Info.LQR_X[4]*Control_Info->L_Leg_Info.LQR_K[1][4];
	Control_Info->L_Leg_Info.LQR_Output[1][5] = Control_Info->L_Leg_Info.LQR_X[5]*Control_Info->L_Leg_Info.LQR_K[1][5];

    Control_Info->L_Leg_Info.Moment.Balance_Tp = -( Control_Info->L_Leg_Info.LQR_Output[1][0] + Control_Info->L_Leg_Info.LQR_Output[1][1] + Control_Info->L_Leg_Info.LQR_Output[1][2]
                                               + Control_Info->L_Leg_Info.LQR_Output[1][3] + Control_Info->L_Leg_Info.LQR_Output[1][4] + Control_Info->L_Leg_Info.LQR_Output[1][5]);
 
	
	Control_Info->R_Leg_Info.LQR_Output[1][0] = Control_Info->R_Leg_Info.LQR_X[0]*Control_Info->R_Leg_Info.LQR_K[1][0];
	Control_Info->R_Leg_Info.LQR_Output[1][1] = Control_Info->R_Leg_Info.LQR_X[1]*Control_Info->R_Leg_Info.LQR_K[1][1];
	Control_Info->R_Leg_Info.LQR_Output[1][2] = Control_Info->R_Leg_Info.LQR_X[2]*Control_Info->R_Leg_Info.LQR_K[1][2];
	Control_Info->R_Leg_Info.LQR_Output[1][3] = Control_Info->R_Leg_Info.LQR_X[3]*Control_Info->R_Leg_Info.LQR_K[1][3];
	Control_Info->R_Leg_Info.LQR_Output[1][4] = Control_Info->R_Leg_Info.LQR_X[4]*Control_Info->R_Leg_Info.LQR_K[1][4];
	Control_Info->R_Leg_Info.LQR_Output[1][5] = Control_Info->R_Leg_Info.LQR_X[5]*Control_Info->R_Leg_Info.LQR_K[1][5];	
	
	Control_Info->R_Leg_Info.Moment.Balance_Tp =   (Control_Info->R_Leg_Info.LQR_Output[1][0] + Control_Info->R_Leg_Info.LQR_Output[1][1] + Control_Info->R_Leg_Info.LQR_Output[1][2] + Control_Info->R_Leg_Info.LQR_Output[1][3] + Control_Info->R_Leg_Info.LQR_Output[1][4] + Control_Info->R_Leg_Info.LQR_Output[1][5]);
  


}	
	
//另类目标控制，只不过这里的控制是写死的
//各种功能是以打补丁的方式添加到对关节电机，驱动轮电机的控制中

static void Comprehensive_F_Calculate(Control_Info_Typedef *Control_Info){


  // 更新PID_Leg_Coordinate，设置目标值为左脚和右脚的Theta差值，反馈值为左脚和右脚的Theta差值
 
  // 防劈叉PID控制
  // 更新PID_Leg_Coordinate，设置目标值为左脚和右脚的Theta差值，反馈值为左脚和右脚的Theta差值
     PID_Calculate(&PID_Leg_Coordinate, 0, Control_Info->L_Leg_Info.Measure.Theta - Control_Info->R_Leg_Info.Measure.Theta);
// //影响简化腿摆角力矩Tp
// //将腿部坐标调节力矩分配给左右腿，方向相反以实现平衡
 	Control_Info->L_Leg_Info.Moment.Leg_Coordinate_Tp = -PID_Leg_Coordinate.Output;
 	Control_Info->R_Leg_Info.Moment.Leg_Coordinate_Tp = -PID_Leg_Coordinate.Output;



//	
// // 	//驱动轮的转矩
// // 	//公式：T = 平衡力矩
 // 	//         +转向力矩
  	Control_Info->L_Leg_Info.T = Control_Info->L_Leg_Info.Moment.Balance_T  + Control_Info->L_Leg_Info.Moment.Turn_T ;
  	Control_Info->R_Leg_Info.T = Control_Info->R_Leg_Info.Moment.Balance_T  + Control_Info->R_Leg_Info.Moment.Turn_T ;

 // 	//简化腿的总转矩
 // 	//公式：Tp =  平衡转矩
 // 	//         + 双腿摆角补偿转矩
  	Control_Info->L_Leg_Info.Tp =   Control_Info->L_Leg_Info.Moment.Balance_Tp + Control_Info->L_Leg_Info.Moment.Leg_Coordinate_Tp;
  	Control_Info->R_Leg_Info.Tp =   Control_Info->R_Leg_Info.Moment.Balance_Tp + Control_Info->R_Leg_Info.Moment.Leg_Coordinate_Tp;
	
 	if(Control_Info->L_Leg_Info.Support.Flag == 1){
	   
 			Control_Info->L_Leg_Info.T = 0;
	
 	}
	
 	if(Control_Info->R_Leg_Info.Support.Flag == 1){
	
 			Control_Info->R_Leg_Info.T = 0;
	
	
 	}
//	
	
	
  	if(Control_Info->Chassis_Situation == CHASSIS_WEAK){
 
     Control_Info->L_Leg_Info.Tp = 0;
     Control_Info->R_Leg_Info.Tp = 0;
  	 Control_Info->L_Leg_Info.F = 0;
     Control_Info->R_Leg_Info.F = 0;	
  	 Control_Info->L_Leg_Info.T = 0;
     Control_Info->R_Leg_Info.T = 0;	
 
   }
/*
测试用：
1：简化模型参数到实际模型参数转换器
的测试
输入遥控器的值，转换到F和Tp


*/	



 	//    Control_Info->L_Leg_Info.Tp = remote_ctrl.rc.ch[2] *0.01f;
    //   Control_Info->R_Leg_Info.Tp = remote_ctrl.rc.ch[0] *0.01f;
 	//     Control_Info->L_Leg_Info.F  = remote_ctrl.rc.ch[3] *0.25f;
    //   Control_Info->R_Leg_Info.F  = remote_ctrl.rc.ch[1]  *0.25f;
//	  	 Control_Info->L_Leg_Info.T = 0;
//   Control_Info->R_Leg_Info.T = 0;	

}

static void Joint_Tourgue_Calculate(Control_Info_Typedef *Control_Info){
//输入：简化力和简化转矩
//转换器：2X2的雅克比矩阵
//输出：真实结构中各电机的力矩
//原理：
//中间量：
	float Current_max;
float Tourgue_max;
	Tourgue_max = 35.f;
	Current_max =10000;

//偏腿
//左腿
//0号电机，左小腿 对应T2 公式：T2 =（A/K）*F +(Tp/2.0f)
Control_Info->L_Leg_Info.SendValue.T_Calf = (Control_Info->L_Leg_Info.A * Control_Info->L_Leg_Info.F)/Control_Info->L_Leg_Info.Biased.K + (Control_Info->L_Leg_Info.Tp/2.0f);
//1号电机，左大腿 对应T1 公式：T1 =（-A/K）*F + (Tp/2.0f)
Control_Info->L_Leg_Info.SendValue.T_Thigh = (-Control_Info->L_Leg_Info.A * Control_Info->L_Leg_Info.F)/Control_Info->L_Leg_Info.Biased.K + (Control_Info->L_Leg_Info.Tp/2.0f);
//右腿
//2号电机，右大腿 对应T2 公式：T2 =（A/K）*F +(Tp/2.0f)
	//2025年11.12修改：互换
Control_Info->R_Leg_Info.SendValue.T_Thigh = (Control_Info->R_Leg_Info.A * Control_Info->R_Leg_Info.F)/Control_Info->R_Leg_Info.Biased.K + (Control_Info->R_Leg_Info.Tp/2.0f);
//3号电机，右小腿 对应T1 公式：T1 =（-A/K）*F + (Tp/2.0f)
Control_Info->R_Leg_Info.SendValue.T_Calf = (-Control_Info->R_Leg_Info.A * Control_Info->R_Leg_Info.F)/Control_Info->R_Leg_Info.Biased.K + (Control_Info->R_Leg_Info.Tp/2.0f);



// ////偏腿
// ////左腿
// ////0号电机，左小腿 对应T2 公式：T2 =（A/K）*F +(Tp/2.0f)
// Control_Info->L_Leg_Info.SendValue.T_Calf =  (Control_Info->L_Leg_Info.Tp/2.0f);
// ////1号电机，左大腿 对应T1 公式：T1 =（-A/K）*F + (Tp/2.0f)
// Control_Info->L_Leg_Info.SendValue.T_Thigh =  (Control_Info->L_Leg_Info.Tp/2.0f);
// ////右腿
// ////2号电机，右大腿 对应T2 公式：T2 =（A/K）*F +(Tp/2.0f)
// //	//2025年11.12修改：互换
// Control_Info->R_Leg_Info.SendValue.T_Thigh = (Control_Info->R_Leg_Info.Tp/2.0f);
// ////3号电机，右小腿 对应T1 公式：T1 =（-A/K）*F + (Tp/2.0f)
// Control_Info->R_Leg_Info.SendValue.T_Calf = (Control_Info->R_Leg_Info.Tp/2.0f);

//测试F 转换


// ////偏腿
//// ////左腿
//// ////0号电机，左小腿 对应T2 公式：T2 =（A/K）*F +(Tp/2.0f)
// Control_Info->L_Leg_Info.SendValue.T_Calf =   (-Control_Info->L_Leg_Info.A * Control_Info->L_Leg_Info.F)/Control_Info->L_Leg_Info.Biased.K;
//// ////1号电机，左大腿 对应T1 公式：T1 =（-A/K）*F + (Tp/2.0f)
// Control_Info->L_Leg_Info.SendValue.T_Thigh =  (Control_Info->L_Leg_Info.A * Control_Info->L_Leg_Info.F)/Control_Info->L_Leg_Info.Biased.K;
//// ////2号电机，右大腿 对应T2 公式：T2 =（A/K）*F +(Tp/2.0f)
// //	//2025年11.12修改：互换
// Control_Info->R_Leg_Info.SendValue.T_Thigh = (-Control_Info->R_Leg_Info.A * Control_Info->R_Leg_Info.F)/Control_Info->R_Leg_Info.Biased.K;
//// ////3号电机，右小腿 对应T1 公式：T1 =（-A/K）*F + (Tp/2.0f)
// Control_Info->R_Leg_Info.SendValue.T_Calf = (Control_Info->R_Leg_Info.A * Control_Info->R_Leg_Info.F)/Control_Info->R_Leg_Info.Biased.K;




//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------											 
//以下部分不用改，串并联通用部分
//轮子电机的电流值
//	 Control_Info->L_Leg_Info.SendValue.Current = (int16_t)( Control_Info->L_Leg_Info.T * 1000.f);
//	 Control_Info->R_Leg_Info.SendValue.Current = (int16_t)( -Control_Info->R_Leg_Info.T * 1000.f);

	 Control_Info->L_Leg_Info.SendValue.Current = (int16_t)( Control_Info->L_Leg_Info.T *1000.f);
	 Control_Info->R_Leg_Info.SendValue.Current = (int16_t)( -Control_Info->R_Leg_Info.T * 1000.f);

	 
	 VAL_LIMIT(Control_Info->L_Leg_Info.SendValue.Current,-Current_max,Current_max);
	 VAL_LIMIT(Control_Info->R_Leg_Info.SendValue.Current,-Current_max,Current_max); 
  
	//  VAL_LIMIT(Control_Info->L_Leg_Info.SendValue.T1,-54.f,54.f);
																
	  
	  VAL_LIMIT(Control_Info->L_Leg_Info.SendValue.T_Calf ,-Tourgue_max,Tourgue_max);
	  VAL_LIMIT(Control_Info->L_Leg_Info.SendValue.T_Thigh,-Tourgue_max,Tourgue_max);  
      VAL_LIMIT(Control_Info->R_Leg_Info.SendValue.T_Thigh,-Tourgue_max,Tourgue_max);
	  VAL_LIMIT(Control_Info->R_Leg_Info.SendValue.T_Calf ,-Tourgue_max,Tourgue_max);  			
																				
}