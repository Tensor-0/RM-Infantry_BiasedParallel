
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




/*说明文档
驱动轮电机：左腿轮：大疆3508电机--Id:0号电机//右腿轮：大疆3508电机--Id:1号电机
关节电机
//一些参考
1：单个关节电机转矩到7n/m时，轻微助力整个腿部可一直转（另一个电机失能状态）



*/


//变量区--------------------------------------------------------------------------------------
//========================================================================================

/*偏置并联腿LQR控制器增益矩阵
2025.10.31.第一次测试，
改动：单腿重量：3.40
      机体重量：8.75
效果：*/
float K11[6] = {0,-301.826001f	,350.951093f	,-262.443570f	,-4.166325f};
float K12[6] = {0,13.554556f	,-24.969778f	,-33.170803f	,0.273917f};
float K13[6] = {0,-221.195361f	,205.461024f	,-64.133850f	,-15.357114f};
float K14[6] = {0,-141.312669f	,140.168584f	,-59.602650f	,-10.846556f};
float K15[6] = {0,-1199.088787f	,1206.687560f	,-434.244457f	,65.252143f};
float K16[6] = {0,-58.739170f	,62.827610f		,-25.154270f	,4.849511f};
float K21[6] = {0,-166.093739f	,191.491484f	,-86.727247f	,26.913637f};
float K22[6] = {0,-0.505594f	,0.912968f		,0.147179f		,2.701292f};
float K23[6] = {0,-757.410310f	,762.051907f	,-274.104199f	,41.068039f};
float K24[6] = {0,-518.460403f	,519.668567f	,-186.549241f	,28.912702f};
float K25[6] = {0,1409.526386f	,-1309.572837f	,408.970786f	,96.799292f};
float K26[6] = {0,103.076627f	,-98.176298f	,31.960967f	,1.991248f};
//=========================================================================================



//并联腿
///* LQR控制器系数矩阵（通过MATLAB拟合获得） */
//float K11[6] = {0,-266.885577f,325.173670f  ,-247.616760f ,-1.462362f};
//float K12[6] = {0,14.857699f  ,-22.748998f  ,-26.338491f  ,0.302026f};
//float K13[6] = {0,-367.583527f,363.252641f  ,-125.208753f ,-8.205764f};
//float K14[6] = {0,-215.504823f,221.521582f  ,-91.470264f  ,-6.372200f};
//float K15[6] = {0,-619.617893f,751.574467f  ,-353.495425f ,80.695375f};
//float K16[6] = {0,-34.711750f ,46.475780f   ,-24.776425f  ,7.312116f};
//float K21[6] = {0,379.169790f ,-335.475183f ,83.668774f   ,17.041042f};
//float K22[6] = {0,48.175242f  ,-49.710524f  ,19.585459f   ,1.323766f};
//float K23[6] = {0,-170.787565f,253.798890f  ,-142.436138f ,36.898675f};
//float K24[6] = {0,-101.370548f,151.866239f  ,-87.113124f  ,24.965588f};
//float K25[6] = {0,1455.751136f,-1474.823176f,525.283723f  ,68.954698f};
//float K26[6] = {0,121.297246f ,-127.815723f ,48.450999f   ,0.941185f};


static float  PID_Leg_Length_F_Param[7] 	= {1500.f,1.f ,200000.f,0.f ,0  ,10.f,200.f}; //腿长PID(change)
static float  PID_Yaw_P_pama[7] 			= {4.4f  ,0.f ,60.f    ,0   ,0  ,200 ,500  }; // 偏航角位置PID
static float  PID_Yaw_V_pama[7] 			= {0.25f ,0   ,0.4f    ,0   ,0  ,200 ,70   }; // 偏航角速度PID
static float  PID_Leg_Coordinate_param[7]   = {300.f ,0.f ,20.0f   ,0.f ,0.f,0.f ,50   }; // 腿部协调PID

PID_Info_TypeDef PID_Leg_Coordinate;  // 腿部协调控制器
PID_Info_TypeDef PID_Leg_length_F[2]; // 左右腿长度控制器
PID_Info_TypeDef PID_Yaw[2];          // 偏航控制器（位置+速度）


//调试区----------------------------------------------------------------------------------------------


float Test_Theta = 0;                           //测试用的摆杆倾角--0度--摆杆垂直地面

//float Joint_Angle_offset_Num = 0.635f;          //机械安装偏移补偿(并联腿)

float Test_Vmc_Target_L0_Chassis_High   = 0.38f;//高底盘
float Test_Vmc_Target_L0_Chassis_Normal = 0.17f;//正常高度底盘
//整活用
//float Test_Vmc_Target_L0_L              = 0;//左腿目标腿长
//float Test_Vmc_Target_L0_R              = 0;//右腿目标腿长

Control_Info_Typedef Control_Info ={
//腿的机械参数，固定值--如果是新车，这一块要改
.L_Leg_Info = {
	// .VMC ={
	// 	.L1 = 0.15f,//大腿长度（m）
	// 	.L4 = 0.15f,//另一边大腿（m）
	// 	.L2 = 0.28f,//小腿
	// 	.L3 = 0.28f,//另一条小腿
	// 	.L5 = 0.161f,//髋关节间距（m）
	// },
	// .Gravity_Compensation = 110.f,//重力补偿（N）
	.Thigh_Comp_Angle = 0.0f,//大腿摆角补偿
	.Calf_Comp_Angle  = 0.0f,//小腿摆角补偿
	.Biased = {
		.L_Thigh_Link = 0.215f,//大腿连杆，连杆直驱--动力长后杆长度（AH）
		.L_Calf_Link  = 0.098f,//小腿连杆(仿生机器人领域标准术语）动力短前杆长度（AD）,连杆传动
		.K = 0.4558139f,//连杆长度比K=AD/AH

		
	},//偏置并联

},//单腿测试



//偏置并联



};   


//函数区-------------------------------------------------------------------------------------------
static void Control_Init(Control_Info_Typedef *Control_Info);
/*清零所有状态量/设置默认PID参数/配置初始工作模式/分配内存资源/系统启动时调用一次*/
static void Check_Low_Voltage_Beep(Control_Info_Typedef *Control_Info);//1.检查电池（低电压警告）
static void Mode_Update(Control_Info_Typedef *Control_Info);           //2.模式更新
static void Joint_Angle_Offset(Control_Info_Typedef *Control_Info);    //3.看腿的形状（读关节角度，并映射到VMC坐标系）
static void VMC_Calculate(Control_Info_Typedef *Control_Info);         //4.知道形状后，连杆正运动学解算，求简化腿腿长L0和摆角Phi0，轮子速度
static void LQR_K_Update(Control_Info_Typedef *Control_Info);          //5.得到简化腿长后，更新LQR控制器的增益矩阵K

static void Measure_Update(Control_Info_Typedef *Control_Info);        //6.更新传感器数据和目标值
static void Target_Update(Control_Info_Typedef *Control_Info);
static void LQR_X_Update(Control_Info_Typedef *Control_Info);			//7.更新状态向量差值，为LQR控制器u = -K·X提供输入向量（值）
static void VMC_Measure_F_Tp_Calculate(Control_Info_Typedef *Control_Info);//8.求简化腿的顶力（F）和扭矩(Tp)

static void LQR_T_Tp_Calculate(Control_Info_Typedef *Control_Info);		//9.自适应的LQR，根据是否在空中来限制LQR的作用

static void Comprehensive_F_Calculate(Control_Info_Typedef *Control_Info);//10.综合计算
static void Joint_Tourgue_Calculate(Control_Info_Typedef *Control_Info);//11.实际向电机发送数据的计算



TickType_t Control_Task_SysTick = 0;


void Control_Task(void const * argument)
{
  /*只跑一次的*/
  
 	Control_Init(&Control_Info);
	
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
		//上级函数
		Target_Update(&Control_Info);
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
        // USART_Vofa_Justfloat_Transmit(Control_Info.R_Leg_Info.Measure.Chassis_Velocity,Control_Info.L_Leg_Info.Measure.Chassis_Velocity,0,0);

		osDelayUntil(&Control_Task_SysTick,1);
  }
}
  /* USER CODE END Control_Task */
 static uint32_t Tick = 0;
//不用改，串并联通用======================================================================================
static void Check_Low_Voltage_Beep(Control_Info_Typedef *Control_Info){
 
	 Control_Info->VDC = USER_ADC_Voltage_Update();
 	
	 if(Control_Info->VDC < 22.f){//如果电压低于22V
	    Tick++;
		// 触发蜂鸣器报警模式（0.1s开/0.1s关，循环3次）
		 if(Tick < 100){
		    TIM12->CCR2 = 1000;
		 }else if(Tick > 100 && Tick <200){
		 	TIM12->CCR2 = 0;
		 }else if(Tick > 200 && Tick < 300){
		 	TIM12->CCR2 = 1000;
		 }else if(Tick > 300 && Tick < 400){
			TIM12->CCR2 = 0;
		 }else if(Tick > 400 && Tick < 500){
			TIM12->CCR2 = 1000;
		 }else if(Tick > 500 && Tick < 600){
			TIM12->CCR2 = 0;
     }else if(Tick > 1000){
			 Tick = 0;
		 }
		}else if(Control_Info->VDC >= 22.f){
	 
			TIM12->CCR2 = 0;
			
	  }
  }
//PID初始化
static void Control_Init(Control_Info_Typedef *Control_Info){
	//腿长PID初始化
    PID_Init(&PID_Leg_length_F[0],PID_POSITION,PID_Leg_Length_F_Param);
    PID_Init(&PID_Leg_length_F[1],PID_POSITION,PID_Leg_Length_F_Param);
	// 初始化偏航PID控制器（位置环+速度环）
	PID_Init(&PID_Yaw[0],PID_POSITION,PID_Yaw_P_pama);
	PID_Init(&PID_Yaw[1],PID_POSITION,PID_Yaw_V_pama);
	// 初始化腿部协调PID控制器
	PID_Init(&PID_Leg_Coordinate, PID_POSITION, PID_Leg_Coordinate_param);
}



//模式更新
static void Mode_Update(Control_Info_Typedef *Control_Info){

	//当遥控器在线
	if(remote_ctrl.rc.s[1] == 3 || remote_ctrl.rc.s[1]){
		//打开初始化flag
		Control_Info->Init.IF_Begin_Init = 1;
	//当遥控器离线
	}else{
		//关闭初始化flag，	 
		Control_Info->Init.IF_Begin_Init = 0;
    if(Control_Info->Chassis_Situation == CHASSIS_BALANCE) Control_Info->Chassis_Situation = CHASSIS_WEAK; 
     } 
		 
//初始化flag打开后，（并且底盘虚弱）
if(Control_Info->Init.IF_Begin_Init == 1 && Control_Info->Chassis_Situation == CHASSIS_WEAK){	
	
    //关节初始化flag关闭    
   if(Control_Info->Init.Joint_Init.IF_Joint_Init == 0){  
	   	//关节电机到达安全位置（小车放在地上时，四个关节电机的位置）
			 //先检查关节电机是否到达安全位置
			Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[0] = (DM_8009_Motor[0].Data.Position > -0.64f );//有点意思
			Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[1] = (DM_8009_Motor[1].Data.Position < 0.64f ) ;
			Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[2] = (DM_8009_Motor[2].Data.Position < 0.64f ) ;
			Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[3] = (DM_8009_Motor[3].Data.Position > -0.64f );
			 
			Control_Info->Init.Joint_Init.IF_Joint_Init =   (Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[0] &&
		 													 Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[1] &&
		 													 Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[2] &&
							 								 Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[3]);  
			//重置底盘位置（把当前位置当做原点）		
			Control_Info->L_Leg_Info.Measure.Chassis_Position = 0 ;
    		Control_Info->R_Leg_Info.Measure.Chassis_Position = 0 ; 

	//如果所有关节电机到达安全位置
	}else if(Control_Info->Init.Joint_Init.IF_Joint_Init == 1){
		//进入平衡状态		
		 Control_Info->Chassis_Situation = CHASSIS_BALANCE;
		//重置底盘位置（把当前位置当做原点）
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



//确定连杆腿的形状（读取关节电机的弧度值，速度）
//打一个补丁，在这个函数顺便再读取关节电机的扭矩，方便分清各电机对应腿部什么部位
static void Joint_Angle_Offset(Control_Info_Typedef *Control_Info){
//偏置=====================================================================================================
/*
我们这样设计:
1：以逆时针为正方向，水平向右为X轴，垂直向上为Y轴，
为了方便，将机器人的腿部连杆结构放在一二象限内

2：电机与连杆是通过链条连接的，且，连杆上存在机械限位
在之后的控制中，我们需要知道连杆和电机角度（弧度值）的映射关系，
在这里，为了方便，我们先将连杆转到最低的限位，（蹲在地上），
然后重置关节电机的零点。
3：
（不同与并联腿，偏置腿可以全周转动，
也就是说，并联腿的连杆活动范围是固定的（比如无法转到头上），
而偏置腿上的限位只是限制了大小连杆的相对角度，大小连杆本身可以自由转动，
这导致一个问题，
也就是给关节电机定零点时，
你没法完全保证大小连杆各自和水平线的夹角是相等的
故定零点时要注意，
要么摆姿势让夹角相等，
要么定好零点之后分别计算大小连杆和关节电机零点的相对角度（弧度制））
4：要求什么
左右腿大小动力杆（共四根）与各自对应的关节电机（共四个）之间的映射关系
在求之前，先要完成的处理是
（0.1）设置好关节电机的ID和模式
（0.2）配对大小动力杆和关节电机
（1）已经合理的重置好了关节电机的零点，重置零点时，最好书面记入一下当时机器人腿部的姿势
（2）重置好零点后，记入大小动力杆转到水平线时，关节电机的弧度值，
5：开始映射

*/
//左腿
	// //左小腿摆角与0号关节电机的映射(摆角=补偿角度+电机角度)
	// Control_Info->L_Leg_Info.Biased.Calf_Angle 		 	= Control_Info->L_Leg_Info.Calf_Comp_Angle	   +(DM_8009_Motor[0].Data.Position);
	// //左大腿摆角与1号关节电机的映射（摆角=PI-补偿角度+电机角度）
	// Control_Info->L_Leg_Info.Biased.Thigh_Angle 		 = (PI-Control_Info->L_Leg_Info.Thigh_Comp_Angle)+(DM_8009_Motor[1].Data.Position);

//测试用：假设，当腿完全收缩时-也就是到达最低机械限位，时，大腿和小腿连杆可以张到180度（实际是180.03度）

    // //左小腿摆角与0号关节电机的映射(摆角=-电机角度)
	 Control_Info->L_Leg_Info.Biased.Calf_Angle 		 = DM_8009_Motor[0].Data.Position;
	// //左大腿摆角与1号关节电机的映射（摆角=电机角度）
	 Control_Info->L_Leg_Info.Biased.Thigh_Angle 		 =   DM_8009_Motor[1].Data.Position;
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
//右腿
	// //右大腿摆角和2号关节电机的映射(同左腿)
	// Control_Info->R_Leg_Info.Biased.Thigh_Angle 		 = (PI -Control_Info->R_Leg_Info.Thigh_Comp_Angle)+(DM_8009_Motor[2].Data.Position);
	// //右小腿摆角和3号关节电机的映射
	// Control_Info->R_Leg_Info.Biased.Calf_Angle  		 = Control_Info->R_Leg_Info.Calf_Comp_Angle		+(DM_8009_Motor[3].Data.Position);



	//测试用
	//测试用：假设，当腿完全收缩时-也就是到达最低机械限位，时，大腿和小腿连杆可以张到180度（实际是180.03度）

	// //右大腿摆角和2号关节电机的映射(同左腿)
	 Control_Info->R_Leg_Info.Biased.Thigh_Angle 		 =  DM_8009_Motor[2].Data.Position;
	// //右小腿摆角和3号关节电机的映射
	 Control_Info->R_Leg_Info.Biased.Calf_Angle  		 =   DM_8009_Motor[3].Data.Position;
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

//  //五连杆并联结构--------------------------------------------------------------------------------------------------
// 	//我们这样设计：设关节电机在下限位处为电机的零点
//  //左腿
// 	//连杆角度
//  	Control_Info->L_Leg_Info.VMC.Phi1 		= (PI + Joint_Angle_offset_Num)+(DM_8009_Motor[0].Data.Position);
//  	Control_Info->L_Leg_Info.VMC.Phi4 		= DM_8009_Motor[1].Data.Position - Joint_Angle_offset_Num;
// 	//电机速度
// 	Control_Info->L_Leg_Info.VMC.Phi1_dot 	= DM_8009_Motor[0].Data.Velocity;
// 	Control_Info->L_Leg_Info.VMC.Phi4_dot 	= DM_8009_Motor[1].Data.Velocity;
// //右腿
// 	//连杆角度
// 	  Control_Info->R_Leg_Info.VMC.Phi1 	= (3.141593f + 0.635f) + (DM_8009_Motor[3].Data.Position );
//       Control_Info->R_Leg_Info.VMC.Phi4 	= DM_8009_Motor[2].Data.Position - 0.635f;
// 	 //速度 
// 	  Control_Info->R_Leg_Info.VMC.Phi1_dot = DM_8009_Motor[3].Data.Velocity;
// 	  Control_Info->R_Leg_Info.VMC.Phi4_dot = DM_8009_Motor[2].Data.Velocity;

}

//更新一下
/*
整个运控的控制思路是：


*/
/*形状确定，开始计算其简化模型
--求解虚拟腿长L0，为了更新不同腿长下的雅可比矩阵

求解虚拟腿的角度，为了更新状态空间方程

//求解轮子的X轴和Y轴上的速度（两条腿）

//最主要的改的地方*/
static void VMC_Calculate(Control_Info_Typedef *Control_Info){
//偏置并联结构===============================================================================================================================
//这个部分是将真实复杂的小车结构，转换为简化的二阶倒立摆的预处理
    //1:求解虚拟腿的角度，为了更新状态空间方程
	//2:求解虚拟腿长L0，为了更新不同腿长下的雅可比矩阵
	//3:求解轮子的X轴和Y轴上的速度（两条腿）
//左腿
    //中间变量=====================================================================================================================================
	//a,b,简化表达用
	Control_Info->L_Leg_Info.a = Control_Info->L_Leg_Info.Biased.L_Calf_Link;
	Control_Info->L_Leg_Info.b = Control_Info->L_Leg_Info.Biased.L_Thigh_Link;
	//左夹角差半值=（左小腿摆角-左大腿摆角）/2
	Control_Info->L_Leg_Info.M = (Control_Info->L_Leg_Info.Biased.Calf_Angle - Control_Info->L_Leg_Info.Biased.Thigh_Angle)/2.f;
	//左夹角和半差=（左小腿摆角+左大腿摆角）/2
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
	Control_Info->R_Leg_Info.M = (Control_Info->R_Leg_Info.Biased.Thigh_Angle - Control_Info->R_Leg_Info.Biased.Calf_Angle)/2.f;
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

 //五连杆并联结构================================================================================================================================
/* 中间计算变量（优化用）
 A0, B0, C0;       // 二次方程系数
 LBD_2, LBD;       // BD距离平方/实际值
 Sqrt_Cache;       // 平方根缓存*/


//先建模，再简化


 //左腿
	//建模------------------------------------------------------------------------------------------------------------------------------
		
		//1.先计算膝盖坐标（B点和D点坐标）

// 			//L1-->B点坐标	
// 			Control_Info->L_Leg_Info.VMC.X_B = Control_Info->L_Leg_Info.VMC.L1 * arm_cos_f32(Control_Info->L_Leg_Info.VMC.Phi1);
// 			Control_Info->L_Leg_Info.VMC.Y_B = Control_Info->L_Leg_Info.VMC.L1 * arm_sin_f32(Control_Info->L_Leg_Info.VMC.Phi1);
// 			//L5 and L4-->D点坐标
// 			Control_Info->L_Leg_Info.VMC.X_D = Control_Info->L_Leg_Info.VMC.L4 * arm_cos_f32(Control_Info->L_Leg_Info.VMC.Phi4) + (Control_Info->L_Leg_Info.VMC.L5);
// 			Control_Info->L_Leg_Info.VMC.Y_D = Control_Info->L_Leg_Info.VMC.L4 * arm_sin_f32(Control_Info->L_Leg_Info.VMC.Phi4);
		
// 		//2.计算BD向量

// 		     Control_Info->L_Leg_Info.VMC.X_D_X_B = Control_Info->L_Leg_Info.VMC.X_D - Control_Info->L_Leg_Info.VMC.X_B;
// 		     Control_Info->L_Leg_Info.VMC.Y_D_Y_B = Control_Info->L_Leg_Info.VMC.Y_D - Control_Info->L_Leg_Info.VMC.Y_B;
		
// 		//3.计算BD距离的平方

// 		    Control_Info->L_Leg_Info.VMC.LBD_2 = Control_Info->L_Leg_Info.VMC.X_D_X_B * Control_Info->L_Leg_Info.VMC.X_D_X_B + Control_Info->L_Leg_Info.VMC.Y_D_Y_B * Control_Info->L_Leg_Info.VMC.Y_D_Y_B;
		
// 		//4.求膝关节角度Phi2

// 			//先求二次方程系数A0, B0, C0
// 	 		Control_Info->L_Leg_Info.VMC.A0 = 2 *Control_Info->L_Leg_Info.VMC.L2 * Control_Info->L_Leg_Info.VMC.X_D_X_B;
// 			Control_Info->L_Leg_Info.VMC.B0 = 2 * Control_Info->L_Leg_Info.VMC.L2* Control_Info->L_Leg_Info.VMC.Y_D_Y_B;
// 			Control_Info->L_Leg_Info.VMC.C0 =    (Control_Info->L_Leg_Info.VMC.L2*Control_Info->L_Leg_Info.VMC.L2)
// 												-(Control_Info->L_Leg_Info.VMC.L3*Control_Info->L_Leg_Info.VMC.L3)
// 												+(Control_Info->L_Leg_Info.VMC.LBD_2);
// 			//再求Sqrt_Cache 平方根缓存
// 			arm_sqrt_f32(Control_Info->L_Leg_Info.VMC.A0*Control_Info->L_Leg_Info.VMC.A0 + Control_Info->L_Leg_Info.VMC.B0*Control_Info->L_Leg_Info.VMC.B0 - Control_Info->L_Leg_Info.VMC.C0*Control_Info->L_Leg_Info.VMC.C0,&Control_Info->L_Leg_Info.VMC.Sqrt_Cache);
// 			//开始求角度Phi2
// 			Control_Info->L_Leg_Info.VMC.Phi2 = 2 * atan2f(Control_Info->L_Leg_Info.VMC.B0 + Control_Info->L_Leg_Info.VMC.Sqrt_Cache,Control_Info->L_Leg_Info.VMC.A0+Control_Info->L_Leg_Info.VMC.C0);
		
// 		//5.知道小腿张开多少，就可以开始求脚的坐标（C点）
		
// 		Control_Info->L_Leg_Info.VMC.X_C= Control_Info->L_Leg_Info.VMC.X_B + Control_Info->L_Leg_Info.VMC.L2 * arm_cos_f32(Control_Info->L_Leg_Info.VMC.Phi2);
// 		Control_Info->L_Leg_Info.VMC.Y_C= Control_Info->L_Leg_Info.VMC.Y_B + Control_Info->L_Leg_Info.VMC.L2 * arm_sin_f32(Control_Info->L_Leg_Info.VMC.Phi2);
		
// 		//6.先补充另一小腿张开的角度Phi3--先求脚的坐标再求Phi3,计算更方便
		
// 		Control_Info->L_Leg_Info.VMC.Phi3 = atan2f(Control_Info->L_Leg_Info.VMC.Y_C - Control_Info->L_Leg_Info.VMC.Y_D,Control_Info->L_Leg_Info.VMC.X_C - Control_Info->L_Leg_Info.VMC.X_D);
	
// //简化-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------	
		
		
// 		//7.现在，完全确定连杆的形状，开始简化它

// 			//简化后的腿长L0
// 			arm_sqrt_f32(powf(Control_Info->L_Leg_Info.VMC.X_C - Control_Info->L_Leg_Info.VMC.L5/2,2) 
// 							+ Control_Info->L_Leg_Info.VMC.Y_C *Control_Info->L_Leg_Info.VMC.Y_C,
// 							& Control_Info->L_Leg_Info.VMC.L0 );
// 			//腿倾斜的角度
// 			Control_Info->L_Leg_Info.VMC.Phi0 = atan2f(Control_Info->L_Leg_Info.VMC.Y_C,Control_Info->L_Leg_Info.VMC.X_C - (Control_Info->L_Leg_Info.VMC.L5/2));
		
// 		//8.开始计算脚的速度（雅可比矩阵）
// 			//一些中间变量（简化表达用）
// 			static float Phi1_2,Phi3,Phi2,Sin_Phi2_3,Phi3_4;
// 			Phi1_2  = 	Control_Info->L_Leg_Info.VMC.Phi1 - Control_Info->L_Leg_Info.VMC.Phi2;
// 			Phi3 	=  	Control_Info->L_Leg_Info.VMC.Phi3;
// 			Phi2 	=  	Control_Info->L_Leg_Info.VMC.Phi2;
// 			Phi3_4  =  	Control_Info->L_Leg_Info.VMC.Phi3 - Control_Info->L_Leg_Info.VMC.Phi4;
// 			Sin_Phi2_3 = arm_sin_f32( Control_Info->L_Leg_Info.VMC.Phi2 - Control_Info->L_Leg_Info.VMC.Phi3);
// 			//X方向速度
// 			Control_Info->L_Leg_Info.VMC.X_C_dot = 		((Control_Info->L_Leg_Info.VMC.L1 * arm_sin_f32(Phi1_2) * arm_sin_f32(Phi3)) / (Sin_Phi2_3)) * Control_Info->L_Leg_Info.VMC.Phi1_dot
// 													  + ((Control_Info->L_Leg_Info.VMC.L4 * arm_sin_f32(Phi3_4) * arm_sin_f32(Phi2)) / (Sin_Phi2_3)) * Control_Info->L_Leg_Info.VMC.Phi4_dot;
// 			//// Y方向速度									
// 			Control_Info->L_Leg_Info.VMC.Y_C_dot  = 	-((Control_Info->L_Leg_Info.VMC.L1 * arm_sin_f32(Phi1_2) * arm_cos_f32(Phi3)) / (Sin_Phi2_3)) * Control_Info->L_Leg_Info.VMC.Phi1_dot
// 													  + -((Control_Info->L_Leg_Info.VMC.L4 * arm_sin_f32(Phi3_4) * arm_cos_f32(Phi2)) / (Sin_Phi2_3)) * Control_Info->L_Leg_Info.VMC.Phi4_dot;
// //右腿
   
//   // 右腿类似计算 ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// 		Control_Info->R_Leg_Info.VMC.X_B =   Control_Info->R_Leg_Info.VMC.L1 * arm_cos_f32(Control_Info->R_Leg_Info.VMC.Phi1);
// 		Control_Info->R_Leg_Info.VMC.Y_B =   Control_Info->R_Leg_Info.VMC.L1 * arm_sin_f32(Control_Info->R_Leg_Info.VMC.Phi1);
// 		Control_Info->R_Leg_Info.VMC.X_D =  (Control_Info->R_Leg_Info.VMC.L5) + Control_Info->R_Leg_Info.VMC.L4 * arm_cos_f32(Control_Info->R_Leg_Info.VMC.Phi4);
// 		Control_Info->R_Leg_Info.VMC.Y_D =   Control_Info->R_Leg_Info.VMC.L4 * arm_sin_f32(Control_Info->R_Leg_Info.VMC.Phi4);
	
// 		Control_Info->R_Leg_Info.VMC.X_D_X_B = Control_Info->R_Leg_Info.VMC.X_D - Control_Info->R_Leg_Info.VMC.X_B;
// 		Control_Info->R_Leg_Info.VMC.Y_D_Y_B = Control_Info->R_Leg_Info.VMC.Y_D - Control_Info->R_Leg_Info.VMC.Y_B;
// 		Control_Info->R_Leg_Info.VMC.LBD_2 = Control_Info->R_Leg_Info.VMC.X_D_X_B*Control_Info->R_Leg_Info.VMC.X_D_X_B + Control_Info->R_Leg_Info.VMC.Y_D_Y_B*Control_Info->R_Leg_Info.VMC.Y_D_Y_B;
	  
// 	    Control_Info->R_Leg_Info.VMC.A0 =  2 *Control_Info->R_Leg_Info.VMC.L2  * Control_Info->R_Leg_Info.VMC.X_D_X_B;
// 		Control_Info->R_Leg_Info.VMC.B0 =  2 *Control_Info->R_Leg_Info.VMC.L2  * Control_Info->R_Leg_Info.VMC.Y_D_Y_B;
// 		Control_Info->R_Leg_Info.VMC.C0 =   (Control_Info->R_Leg_Info.VMC.L2 * Control_Info->R_Leg_Info.VMC.L2) 
// 	                                    + (Control_Info->R_Leg_Info.VMC.LBD_2) - (Control_Info->R_Leg_Info.VMC.L3 * Control_Info->R_Leg_Info.VMC.L3);		
// 	    arm_sqrt_f32(Control_Info->R_Leg_Info.VMC.A0*Control_Info->R_Leg_Info.VMC.A0 + Control_Info->R_Leg_Info.VMC.B0*Control_Info->R_Leg_Info.VMC.B0 - Control_Info->R_Leg_Info.VMC.C0*Control_Info->R_Leg_Info.VMC.C0,&Control_Info->R_Leg_Info.VMC.Sqrt_Cache);
// 	    Control_Info->R_Leg_Info.VMC.Phi2 =2 * atan2f(Control_Info->R_Leg_Info.VMC.B0 + Control_Info->R_Leg_Info.VMC.Sqrt_Cache,Control_Info->R_Leg_Info.VMC.A0 + Control_Info->R_Leg_Info.VMC.C0);
// 		Control_Info->R_Leg_Info.VMC.X_C  =Control_Info->R_Leg_Info.VMC.X_B + Control_Info->R_Leg_Info.VMC.L2 * arm_cos_f32(Control_Info->R_Leg_Info.VMC.Phi2);
// 		Control_Info->R_Leg_Info.VMC.Y_C  =Control_Info->R_Leg_Info.VMC.Y_B + Control_Info->R_Leg_Info.VMC.L2 * arm_sin_f32(Control_Info->R_Leg_Info.VMC.Phi2);
		
// 	    Control_Info->R_Leg_Info.VMC.Phi3 = atan2f(Control_Info->R_Leg_Info.VMC.Y_C - Control_Info->R_Leg_Info.VMC.Y_D,Control_Info->R_Leg_Info.VMC.X_C - Control_Info->R_Leg_Info.VMC.X_D);
// 		arm_sqrt_f32(powf(Control_Info->R_Leg_Info.VMC.X_C -Control_Info->R_Leg_Info.VMC.L5/2,2) + Control_Info->R_Leg_Info.VMC.Y_C*Control_Info->R_Leg_Info.VMC.Y_C,&Control_Info->R_Leg_Info.VMC.L0);
// 		Control_Info->R_Leg_Info.VMC.Phi0 = atan2f(Control_Info->R_Leg_Info.VMC.Y_C,Control_Info->R_Leg_Info.VMC.X_C - (Control_Info->R_Leg_Info.VMC.L5/2));

// 		Phi1_2 = 	Control_Info->R_Leg_Info.VMC.Phi1 - Control_Info->R_Leg_Info.VMC.Phi2;
// 		Phi3 =  Control_Info->R_Leg_Info.VMC.Phi3;
// 		Phi2 =  Control_Info->R_Leg_Info.VMC.Phi2;
// 		Phi3_4 =  Control_Info->R_Leg_Info.VMC.Phi3 - Control_Info->R_Leg_Info.VMC.Phi4;

// 		Sin_Phi2_3 = arm_sin_f32( Control_Info->R_Leg_Info.VMC.Phi2 - Control_Info->R_Leg_Info.VMC.Phi3);	

//     	Control_Info->R_Leg_Info.VMC.X_C_dot = 	 (( Control_Info->R_Leg_Info.VMC.L1 * arm_sin_f32(Phi1_2) * arm_sin_f32(Phi3)) / (Sin_Phi2_3)) * Control_Info->R_Leg_Info.VMC.Phi1_dot
// 																			     + (( Control_Info->R_Leg_Info.VMC.L4 * arm_sin_f32(Phi3_4) * arm_sin_f32(Phi2)) / (Sin_Phi2_3)) * Control_Info->R_Leg_Info.VMC.Phi4_dot;
// 		Control_Info->R_Leg_Info.VMC.Y_C_dot = 	 -((Control_Info->R_Leg_Info.VMC.L1 * arm_sin_f32(Phi1_2) * arm_cos_f32(Phi3)) / (Sin_Phi2_3)) * Control_Info->R_Leg_Info.VMC.Phi1_dot
// 																				   + -((Control_Info->R_Leg_Info.VMC.L4 * arm_sin_f32(Phi3_4) * arm_cos_f32(Phi2)) / (Sin_Phi2_3)) * Control_Info->R_Leg_Info.VMC.Phi4_dot;



}

//根据简化后的腿长，更新增益矩阵K----不用改
static void LQR_K_Update(Control_Info_Typedef *Control_Info){
	//简化表达
	float L_L0; 
	//L_L0= Control_Info->L_Leg_Info.VMC.L0;(并腿)
L_L0= Control_Info->L_Leg_Info.Sip_Leg_Length;//偏腿

/*左腿LQR增益计算（六状态量）计算每个增益元素的公式为：
//K = a * L0^3 + b * L0^2 + c * L0 + d

powf(变量，指数)：eg powf(L_L0,3)表示L_L0的立方

其中，a, b, c, d 是预先通过拟合得到的系数，这些系数存储在数组K11, K12, ..., K26中。
注意，这些数组有6个元素（索引0到5），但这里只用到了索引1到4（即K11[1]到K11[4]），
而索引0未使用（从代码中看，K11[0]被初始化为0，但没有被使用）
。
这种设计使LQR控制器能够自适应腿长变化​：
1. 当腿伸长时（L0增大），系统惯性矩增加，需要更大的控制增益
2. 当腿缩短时（L0减小），系统响应更快，需要减小控制增益
3. 多项式系数通过系统辨识和优化算法预先确定，确保在全工作范围内保持最优性能
这种自适应策略显著提高了双足机器人在不同工作状态（如行走、站立、抬腿）下的控制稳定性和鲁棒性。
*/	
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
	 //R_L0= Control_Info->R_Leg_Info.VMC.L0;(并腿)
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




/**
  * @brief 更新传感器测量值
  * @param Control_Info 控制信息结构体
  * 
  * 融合IMU、电机编码器等数据，计算关键状态
  * 陀螺仪数据来自底盘的达妙开发板，底盘的倾斜角和角速度
  * 
  * 主要步骤包括：

1.从IMU读取底盘倾斜角和角速度，并转换到左右腿结构体。
2.计算左右腿虚拟腿的倾斜角（Theta）和倾斜角速度（Theta_dot）。
3.计算轮速、角速度、线速度。
4.计算虚拟腿长的变化率（L0_dot）。
5.估计底盘速度（Body）并进行滤波融合（Predict和Fusion）。
6.计算整体底盘速度（取左右腿速度平均值）。
7.根据目标速度更新底盘位置（如果目标速度为0则积分，否则重置为0）。
8.计算底盘加速度（考虑向心加速度和重力分量）。
9.如果底盘处于虚弱状态（未平衡），则重置所有状态为0。
  */
static void Measure_Update(Control_Info_Typedef *Control_Info){
//填状态
	//左腿
		//身体平衡
	//机体水平倾角
		Control_Info->L_Leg_Info.Measure.Phi       = -INS_Info.Angle[2];//注意极性
		Control_Info->L_Leg_Info.Measure.Phi_dot   = -INS_Info.Gyro[0];
		//腿的姿势
		//Control_Info->L_Leg_Info.Measure.Theta     = 	((PI/2) - Control_Info->L_Leg_Info.VMC.Phi0) - Control_Info->L_Leg_Info.Measure.Phi;
		Control_Info->L_Leg_Info.Measure.Theta     = 	((PI/2) - Control_Info->L_Leg_Info.Sip_Leg_Angle) - Control_Info->L_Leg_Info.Measure.Phi;
		
		//Control_Info->L_Leg_Info.Measure.Theta_dot = 			 (Control_Info->L_Leg_Info.VMC.X_C_dot * arm_cos_f32( - Control_Info->L_Leg_Info.Measure.Theta)
		//											  		   +  Control_Info->L_Leg_Info.VMC.Y_C_dot * arm_sin_f32( - Control_Info->L_Leg_Info.Measure.Theta))
		//											  		   /  Control_Info->L_Leg_Info.VMC.L0;
		Control_Info->L_Leg_Info.Measure.Theta_dot = 			 (Control_Info->L_Leg_Info.X_J_Dot * arm_cos_f32( - Control_Info->L_Leg_Info.Measure.Theta)
													  		   +  Control_Info->L_Leg_Info.Y_J_Dot * arm_sin_f32( - Control_Info->L_Leg_Info.Measure.Theta))
													  		   /  Control_Info->L_Leg_Info.Sip_Leg_Length;
	
		//右腿
	// 右腿姿态（使用相同IMU数据）	
		Control_Info->R_Leg_Info.Measure.Phi 		= -INS_Info.Angle[2];
		Control_Info->R_Leg_Info.Measure.Phi_dot 	= -INS_Info.Gyro[0];
		Control_Info->R_Leg_Info.Measure.Theta 		= (Control_Info->R_Leg_Info.Sip_Leg_Angle -(PI/2)) - Control_Info->R_Leg_Info.Measure.Phi;
		Control_Info->R_Leg_Info.Measure.Theta_dot  = ( -Control_Info->R_Leg_Info.X_J_Dot     * arm_cos_f32(-Control_Info->R_Leg_Info.Measure.Theta)
                                               		+  Control_Info->R_Leg_Info.Y_J_Dot       * arm_sin_f32(-Control_Info->R_Leg_Info.Measure.Theta))
	                                             	/  Control_Info->R_Leg_Info.Sip_Leg_Length;	
	
/*
 虚拟腿倾斜角 = (π/2 - 虚拟腿安装角) - 底盘倾斜角
Theta_L = (π/2 - Phi0_L) - Phi_L;

 虚拟腿倾斜角速度 = [X方向速度·cos(Theta) + Y方向速度·sin(Theta)] / 腿长L0
Theta_dot_L = [X_C_dot·cos(-Theta_L) + Y_C_dot·sin(-Theta_L)] / L0_L;
关键变量​：
Phi0：虚拟腿安装角度（固定几何参数）
X_C_dot, Y_C_dot：足端速度（从VMC计算获得）
*/	

//3. ​轮速数据处理
/*
// 左轮转速转换（RPM → rad/s → m/s）
左轮角速度 = 电机原始数据 × (π/30) / 减速比;
左轮线速度 = 左轮角速度 × 轮半径;
*/
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
	Control_Info->L_Leg_Info.Measure.Chassis_Position += Control_Info->Chassis_Velocity*0.001f ;
	
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






//功能：根据遥控器通道3（速度控制）和开关s[1]（腿长模式）更新目标值*/
//它主要处理两种控制指令：移动速度控制和腿部高度控制。
static void Target_Update(Control_Info_Typedef *Control_Info){
/*1，移动逻辑
两种模式，速度/位置模式
移动靠遥控器给速度值，不控制就自动变回位置模式，也就是在原地保持平衡，哪也不去。
*/
if(remote_ctrl.rc.ch[3] != 0 ){//开始控制
//关键参数​：
//0.031：摇杆值到速度的转换系数（满摇杆对应约±1.6 m/s）
//0.0013：加速斜坡斜率（加速响应较快）
//0.002：减速斜坡斜率（减速响应较缓
//平滑
	Control_Info->Target_Velocity = f_Ramp_Calc(Control_Info->Target_Velocity,remote_ctrl.rc.ch[3] * 0.031,0.0013f);
//沉默位置模式
	//重置位置积分器
	//左右腿位置刷新
	Control_Info->L_Leg_Info.Measure.Chassis_Position = 0;
	Control_Info->R_Leg_Info.Measure.Chassis_Position = 0 ; 
}else if(remote_ctrl.rc.ch[3] == 0){//不移动了，开摆
	//平滑归零
	Control_Info->Target_Velocity = f_Ramp_Calc(Control_Info->Target_Velocity,0,0.002f);//0.002f--慢刹车
}
//2.约束(行车不规范，亲人两行泪)
//此处限速1.6m/s
VAL_LIMIT(Control_Info->Target_Velocity,-1.6f,1.6f);

//3.腿高控制（雄起/正常）
//雄起
if(remote_ctrl.rc.ch[1] == 1){
//目标腿长0,38米
Control_Info->L_Leg_Info.Target_Sip_Leg_Length = Test_Vmc_Target_L0_Chassis_High;
//右
Control_Info->R_Leg_Info.Target_Sip_Leg_Length =  Test_Vmc_Target_L0_Chassis_High;
//正常
}else {
//目标0.17米
Control_Info->L_Leg_Info.Target_Sip_Leg_Length = Test_Vmc_Target_L0_Chassis_Normal;
 Control_Info->R_Leg_Info.Target_Sip_Leg_Length =Test_Vmc_Target_L0_Chassis_Normal;
		}


}


/*该函数实现了：
​状态反馈​：提供当前系统状态与理想状态的偏差
​控制基础​：为LQR控制器 u = -K·X 提供输入向量
​多目标协调​：平衡位置保持、速度跟踪和姿态稳定多个控制目标
​双环控制​：
内环：虚拟腿角度/速度控制（索引0-1）
外环：底盘位置/速度控制（索引2-3*/
static void LQR_X_Update(Control_Info_Typedef *Control_Info){
/*L_Leg_Info.LQR_X[0] = (目标θ - 测量θ)
L_Leg_Info.LQR_X[1] = (目标dθ/dt - 测量dθ/dt)
L_Leg_Info.LQR_X[2] = (目标位置 - 测量位置)
L_Leg_Info.LQR_X[3] = (系统目标速度 - 底盘测量速度)
L_Leg_Info.LQR_X[4] = (目标φ - 测量φ)
L_Leg_Info.LQR_X[5] = (目标dφ/dt - 测量dφ/dt)*/
//左腿
	Control_Info->L_Leg_Info.LQR_X[0] = (Control_Info->L_Leg_Info.Target.Theta            - Control_Info->L_Leg_Info.Measure.Theta);
	Control_Info->L_Leg_Info.LQR_X[1] = (Control_Info->L_Leg_Info.Target.Theta_dot        - Control_Info->L_Leg_Info.Measure.Theta_dot);
	Control_Info->L_Leg_Info.LQR_X[2] = (Control_Info->L_Leg_Info.Target.Chassis_Position - Control_Info->L_Leg_Info.Measure.Chassis_Position);
	Control_Info->L_Leg_Info.LQR_X[3] = (Control_Info->Target_Velocity                    - Control_Info->Chassis_Velocity);
	Control_Info->L_Leg_Info.LQR_X[4] = (Control_Info->L_Leg_Info.Target.Phi              - Control_Info->L_Leg_Info.Measure.Phi);
	Control_Info->L_Leg_Info.LQR_X[5] = (Control_Info->L_Leg_Info.Target.Phi_dot          - Control_Info->L_Leg_Info.Measure.Phi_dot);
//右腿
//右腿同理
	Control_Info->R_Leg_Info.LQR_X[0] = (Control_Info->R_Leg_Info.Target.Theta 			  - Control_Info->R_Leg_Info.Measure.Theta);
	Control_Info->R_Leg_Info.LQR_X[1] = (Control_Info->R_Leg_Info.Target.Theta_dot        - Control_Info->R_Leg_Info.Measure.Theta_dot);
	Control_Info->R_Leg_Info.LQR_X[2] = (Control_Info->R_Leg_Info.Target.Chassis_Position - Control_Info->R_Leg_Info.Measure.Chassis_Position) ;
	Control_Info->R_Leg_Info.LQR_X[3] = (Control_Info->Target_Velocity                    - Control_Info->Chassis_Velocity );
	Control_Info->R_Leg_Info.LQR_X[4] = (Control_Info->R_Leg_Info.Target.Phi              - Control_Info->R_Leg_Info.Measure.Phi);
	Control_Info->R_Leg_Info.LQR_X[5] = (Control_Info->R_Leg_Info.Target.Phi_dot          - Control_Info->R_Leg_Info.Measure.Phi_dot);




}


/*函数功能
1.​计算虚拟腿的推力（F）和扭矩（Tp）​​：
读关节电机的扭矩数据，结合机器人的几何参数（如杆长、角度等），计算出作用在虚拟腿上的顶力（F）和扭矩（Tp）。
分别计算左腿和右腿的F和Tp。

2.离地检测
​在平衡状态下计算支撑力（FN）和设置支撑标志​：
当底盘处于平衡状态（CHASSIS_BALANCE）时，
根据计算出的F和Tp，结合虚拟腿的倾斜角度（Theta）和长度（L0），
计算支撑力（FN）。
如果支撑力FN小于50N，则设置支撑标志（Flag）为1（表示腿处于支撑状态），否则为0。
非平衡状态下，默认躺在地上
支撑标志被重置为0，并给支撑力FN一个默认值（100N）。

逆运动学计算
求解由F和Tp到T1和T2的矩阵的逆矩阵
得到从T1和T2到F和Tp的映射
F= K(T2−T1​)/A,Tp =T1+T2

参数说明
A= (a*t*sinM)/S
​，其中：

M= 
θ_1−θ_2

t=acosϕ+S

a=AD=0.098m

b=DC=0.1176m

K= 98/215为给定比值

逆矩阵存在的条件是 A不为0且 K不为0，这通常当 θ_1不等于θ_2
​
 时成立。
*/

static void VMC_Measure_F_Tp_Calculate(Control_Info_Typedef *Control_Info){
//偏腿
	//左腿 
		//F    //公式：F=K(T2-T1)/A            //(T_左小腿 - T_左大腿) ps:公式相同，但左右腿在单独分析时，大腿的朝向不同，故需单独考虑，具体见整体模型简图
		Control_Info->L_Leg_Info.Measure.F  = (Control_Info->L_Leg_Info.Biased.K * (Control_Info->L_Leg_Info.Biased.T_Calf - Control_Info->L_Leg_Info.Biased.T_Thigh))/Control_Info->L_Leg_Info.A;
		//Tp  //公式：Tp=T1+T2
		Control_Info->L_Leg_Info.Measure.Tp = Control_Info->L_Leg_Info.Biased.T_Thigh + Control_Info->L_Leg_Info.Biased.T_Calf;

	//右腿
		//F   //公式：F=K(T2-T1)/A  = （T2-T1）/(A/K)           //(T_右大腿 - T_右小腿)
		Control_Info->R_Leg_Info.Measure.F  = (Control_Info->R_Leg_Info.Biased.K * (Control_Info->R_Leg_Info.Biased.T_Thigh - Control_Info->R_Leg_Info.Biased.T_Calf))/Control_Info->R_Leg_Info.A;
		//Tp  //公式：Tp=T1+T2
		Control_Info->R_Leg_Info.Measure.Tp = Control_Info->R_Leg_Info.Biased.T_Thigh + Control_Info->R_Leg_Info.Biased.T_Calf;

//并联腿	
// //左腿
// 	Control_Info->L_Leg_Info.Measure.F  = (((DM_8009_Motor[1].Data.Torque    * arm_cos_f32((Control_Info->L_Leg_Info.VMC.Phi0 - Control_Info->L_Leg_Info.VMC.Phi3)))
// 										/   (Control_Info->L_Leg_Info.VMC.L4 * arm_sin_f32((Control_Info->L_Leg_Info.VMC.Phi3 - (Control_Info->L_Leg_Info.VMC.Phi4)))))
// 									    -  ((DM_8009_Motor[0].Data.Torque    * arm_cos_f32((Control_Info->L_Leg_Info.VMC.Phi0 - Control_Info->L_Leg_Info.VMC.Phi2)))
// 									    /   (Control_Info->L_Leg_Info.VMC.L1 * arm_sin_f32((Control_Info->L_Leg_Info.VMC.Phi1 - (Control_Info->L_Leg_Info.VMC.Phi2))))));
		
// 	Control_Info->L_Leg_Info.Measure.Tp = Control_Info->L_Leg_Info.VMC.L0  
// 										* (((DM_8009_Motor[0].Data.Torque    * arm_sin_f32((Control_Info->L_Leg_Info.VMC.Phi0 - Control_Info->L_Leg_Info.VMC.Phi2)))
//                                         /   (Control_Info->L_Leg_Info.VMC.L1 * arm_sin_f32((Control_Info->L_Leg_Info.VMC.Phi1 - (Control_Info->L_Leg_Info.VMC.Phi2))))) 
// 	                                    -  ((DM_8009_Motor[1].Data.Torque    * arm_sin_f32((Control_Info->L_Leg_Info.VMC.Phi0 -  Control_Info->L_Leg_Info.VMC.Phi3)))
//                                         /   (Control_Info->L_Leg_Info.VMC.L4 * arm_sin_f32((Control_Info->L_Leg_Info.VMC.Phi3 - (Control_Info->L_Leg_Info.VMC.Phi4))))));
// //右腿									
// 	Control_Info->R_Leg_Info.Measure.F = (((DM_8009_Motor[2].Data.Torque * arm_cos_f32(( Control_Info->R_Leg_Info.VMC.Phi0 -  Control_Info->R_Leg_Info.VMC.Phi3)))/
//                                          ( Control_Info->L_Leg_Info.VMC.L4 *  arm_sin_f32 (( Control_Info->R_Leg_Info.VMC.Phi3 - (Control_Info->R_Leg_Info.VMC.Phi4)))))      -
// 	                                     ((DM_8009_Motor[3].Data.Torque * arm_cos_f32(( Control_Info->R_Leg_Info.VMC.Phi0 -  Control_Info->R_Leg_Info.VMC.Phi2)))/
//                                          ( Control_Info->L_Leg_Info.VMC.L1 * arm_sin_f32 ((Control_Info->R_Leg_Info.VMC.Phi1 - (Control_Info->R_Leg_Info.VMC.Phi2))))));

//     Control_Info->R_Leg_Info.Measure.Tp= Control_Info->R_Leg_Info.VMC.L0*(((DM_8009_Motor[3].Data.Torque * arm_sin_f32(( Control_Info->R_Leg_Info.VMC.Phi0 -  Control_Info->R_Leg_Info.VMC.Phi2)))/
//                                          ( Control_Info->L_Leg_Info.VMC.L1 *  arm_sin_f32 (( Control_Info->R_Leg_Info.VMC.Phi1 - (Control_Info->R_Leg_Info.VMC.Phi2)))))      -
// 	                                    ((DM_8009_Motor[2].Data.Torque * arm_sin_f32(( Control_Info->R_Leg_Info.VMC.Phi0 -  Control_Info->R_Leg_Info.VMC.Phi3)))/
//                                          ( Control_Info->L_Leg_Info.VMC.L4 * arm_sin_f32 ((Control_Info->R_Leg_Info.VMC.Phi3 - (Control_Info->R_Leg_Info.VMC.Phi4))))));
    											

//离地检测灯

	//当车平衡时
	if(Control_Info->Chassis_Situation == CHASSIS_BALANCE){
	//扭矩不变
	//左腿
	Control_Info->L_Leg_Info.Measure.Tp = Control_Info->L_Leg_Info.Measure.Tp;

	//求支持力
    Control_Info->L_Leg_Info.Support.FN =      Control_Info->L_Leg_Info.Measure.F  * arm_cos_f32(Control_Info->L_Leg_Info.Measure.Theta)
										+ ( - (Control_Info->L_Leg_Info.Measure.Tp * arm_sin_f32(Control_Info->L_Leg_Info.Measure.Theta))
									    /      Control_Info->L_Leg_Info.Sip_Leg_Length);
//右腿
    Control_Info->R_Leg_Info.Measure.Tp = -Control_Info->R_Leg_Info.Measure.Tp;//注意极性


	Control_Info->R_Leg_Info.Support.FN =  Control_Info->R_Leg_Info.Measure.F * arm_cos_f32(Control_Info->R_Leg_Info.Measure.Theta)
										+((Control_Info->R_Leg_Info.Measure.Tp*arm_sin_f32(Control_Info->R_Leg_Info.Measure.Theta))/Control_Info->R_Leg_Info.Sip_Leg_Length);

		//检测标志
			//左腿
			Control_Info->L_Leg_Info.Support.Flag = (Control_Info->L_Leg_Info.Support.FN < 50.f);
			//右腿
			Control_Info->R_Leg_Info.Support.Flag = (Control_Info->R_Leg_Info.Support.FN < 50.f);
	//车不平衡时
	}else {
	   Control_Info->L_Leg_Info.Support.Flag = 0;//红灯
       Control_Info->R_Leg_Info.Support.Flag = 0;
	   Control_Info->L_Leg_Info.Support.FN   = 100.f;
	   Control_Info->R_Leg_Info.Support.FN   =100.f;


	}




}


//在VMC_Measure_F_Tp_Calculate得知现在车是否离地，之后根据支撑状态自适应调整控制策略，并生成最终的控制输出（力矩T和扭矩Tp
//不用改，并腿/偏腿通用
static void LQR_T_Tp_Calculate(Control_Info_Typedef *Control_Info){
//1. 支撑状态自适应调整	

	//当腿处于支撑状态时（着地），​禁用水平方向控制​（轮子力矩T相关增益）
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

		Control_Info->R_Leg_Info.LQR_K[1][2] = 0;  
		Control_Info->R_Leg_Info.LQR_K[1][3] = 0;  
		Control_Info->R_Leg_Info.LQR_K[1][4] = 0;  
		Control_Info->R_Leg_Info.LQR_K[1][5] = 0; 
	
	
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
	
	Control_Info->R_Leg_Info.Moment.Balance_Tp =   Control_Info->R_Leg_Info.LQR_Output[1][0] + Control_Info->R_Leg_Info.LQR_Output[1][1] + Control_Info->R_Leg_Info.LQR_Output[1][2]
                                               + Control_Info->R_Leg_Info.LQR_Output[1][3] + Control_Info->R_Leg_Info.LQR_Output[1][4] + Control_Info->R_Leg_Info.LQR_Output[1][5];
  


}	
	
static void Comprehensive_F_Calculate(Control_Info_Typedef *Control_Info){

	
	Control_Info->Yaw_Err = 0.f - DM_Yaw_Motor.Data.Position * RadiansToDegrees ;
	
	if (Control_Info->Yaw_Err >= 180.f) Control_Info->Yaw_Err -= 360.f;
	else if (Control_Info->Yaw_Err <= -180.f) Control_Info->Yaw_Err += 360.f;

  PID_Calculate(&PID_Yaw[0], 0, Control_Info->Yaw_Err);
	PID_Calculate(&PID_Yaw[1],PID_Yaw[0].Output,INS_Info.Yaw_Gyro);
 
  PID_Calculate(&PID_Leg_Coordinate, 0, Control_Info->L_Leg_Info.Measure.Theta - Control_Info->R_Leg_Info.Measure.Theta);

	Control_Info->L_Leg_Info.Moment.Leg_Coordinate_Tp = -PID_Leg_Coordinate.Output;
	Control_Info->R_Leg_Info.Moment.Leg_Coordinate_Tp = -PID_Leg_Coordinate.Output;
	
	Control_Info->L_Leg_Info.Moment.Roll_F = -(INS_Info.Roll_Angle + 0.4f) * 50.f;
	Control_Info->R_Leg_Info.Moment.Roll_F =  (INS_Info.Roll_Angle + 0.4f) * 50.f;
	
	
	Control_Info->L_Leg_Info.Moment.Turn_T =  PID_Yaw[1].Output;
	Control_Info->R_Leg_Info.Moment.Turn_T = -PID_Yaw[1].Output;
	
  Control_Info->L_Leg_Info.Moment.Leg_Length_F =  PID_Calculate(&PID_Leg_length_F[0],Control_Info->L_Leg_Info.Target_Sip_Leg_Length,Control_Info->L_Leg_Info.Sip_Leg_Length);
  Control_Info->R_Leg_Info.Moment.Leg_Length_F =  PID_Calculate(&PID_Leg_length_F[1],Control_Info->R_Leg_Info.Target_Sip_Leg_Length,Control_Info->R_Leg_Info.Sip_Leg_Length);


		   Control_Info->R_Leg_Info.Gravity_Compensation = 100.f;
	     Control_Info->L_Leg_Info.Gravity_Compensation = 100.f;


	if(Control_Info->L_Leg_Info.Support.Flag == 1){
	   
		Control_Info->L_Leg_Info.Moment.Roll_F = 0;
	  Control_Info->L_Leg_Info.Gravity_Compensation = 140.f;
	}
	
	if(Control_Info->R_Leg_Info.Support.Flag == 1){
	
		Control_Info->R_Leg_Info.Moment.Roll_F = 0;
		Control_Info->R_Leg_Info.Gravity_Compensation = 140.f;

	
	}
	
	
	
	Control_Info->L_Leg_Info.F = Control_Info->L_Leg_Info.Moment.Leg_Length_F  + Control_Info->L_Leg_Info.Moment.Roll_F  +  Control_Info->L_Leg_Info.Gravity_Compensation;  
	Control_Info->R_Leg_Info.F = Control_Info->R_Leg_Info.Moment.Leg_Length_F  + Control_Info->R_Leg_Info.Moment.Roll_F  +  Control_Info->R_Leg_Info.Gravity_Compensation;
	
	
	Control_Info->L_Leg_Info.T = (Control_Info->L_Leg_Info.Moment.Balance_T  );//+ Control_Info->L_Leg_Info.Moment.Turn_T) ;
	Control_Info->R_Leg_Info.T = (Control_Info->R_Leg_Info.Moment.Balance_T  );//+ Control_Info->R_Leg_Info.Moment.Turn_T) ;

	
	Control_Info->L_Leg_Info.Tp =   Control_Info->L_Leg_Info.Moment.Balance_Tp + Control_Info->L_Leg_Info.Moment.Leg_Coordinate_Tp;
	Control_Info->R_Leg_Info.Tp =   Control_Info->R_Leg_Info.Moment.Balance_Tp + Control_Info->R_Leg_Info.Moment.Leg_Coordinate_Tp;
	
	if(Control_Info->L_Leg_Info.Support.Flag == 1){
	   
			Control_Info->L_Leg_Info.T = 0;
	
	}
	
	if(Control_Info->R_Leg_Info.Support.Flag == 1){
	
			Control_Info->R_Leg_Info.T = 0;
	
	
	}
	
	
	
	if(Control_Info->Chassis_Situation == CHASSIS_WEAK){
 
   Control_Info->L_Leg_Info.Tp = 0;
   Control_Info->R_Leg_Info.Tp = 0;
	 Control_Info->L_Leg_Info.F = 0;
   Control_Info->R_Leg_Info.F = 0;	
	 Control_Info->L_Leg_Info.T = 0;
   Control_Info->R_Leg_Info.T = 0;	
 
 }
	
	
}
//这个才是串并联腿实际的独特的函数转换框架，
//整个运控的逻辑是控制机器人运动时把它看成一个二阶独轮车
//目标层的数据都是基于二阶独轮车，只有这个函数才是把来自二阶独轮车的数据转化为真实的关节电机的力矩
//对应韭菜大佬的五连杆运动学解算和VMC-分析力学
//输入：上位平衡车模型的延杆伸缩力generate的力F和扭矩Tp
//输出：实际偏置并联结构的关节电机的力矩T1和T2

static void Joint_Tourgue_Calculate(Control_Info_Typedef *Control_Info){
//输入：简化力和简化转矩
//转换器：2X2的雅克比矩阵
//输出：真实结构中各电机的力矩
//原理：
//中间量：


//偏腿
//左腿
//0号电机，左小腿 对应T2 公式：T2 =（A/K）*F +(Tp/2.0f)
Control_Info->L_Leg_Info.SendValue.T_Calf = (Control_Info->L_Leg_Info.A * Control_Info->L_Leg_Info.F)/Control_Info->L_Leg_Info.Biased.K + (Control_Info->L_Leg_Info.Tp/2.0f);
//1号电机，左大腿 对应T1 公式：T1 =（-A/K）*F + (Tp/2.0f)
Control_Info->L_Leg_Info.SendValue.T_Thigh = (-Control_Info->L_Leg_Info.A * Control_Info->L_Leg_Info.F)/Control_Info->L_Leg_Info.Biased.K + (Control_Info->L_Leg_Info.Tp/2.0f);
//右腿
//2号电机，右大腿 对应T2 公式：T2 =（A/K）*F +(Tp/2.0f)
Control_Info->R_Leg_Info.SendValue.T_Thigh = (Control_Info->R_Leg_Info.A * Control_Info->R_Leg_Info.F)/Control_Info->R_Leg_Info.Biased.K + (Control_Info->R_Leg_Info.Tp/2.0f);
//3号电机，右小腿 对应T1 公式：T1 =（-A/K）*F + (Tp/2.0f)
Control_Info->R_Leg_Info.SendValue.T_Calf = (-Control_Info->R_Leg_Info.A * Control_Info->R_Leg_Info.F)/Control_Info->R_Leg_Info.Biased.K + (Control_Info->R_Leg_Info.Tp/2.0f);

//五连杆并联结构转换器--------------------------------------------------------------------------------------------------
// 	 float Phi2_3,Phi0_3,Phi1_2,Phi0_2,Phi3_4;
   
// 	  Phi2_3=  ( Control_Info->L_Leg_Info.VMC.Phi2 -  Control_Info->L_Leg_Info.VMC.Phi3);
//    Phi0_3=  ( Control_Info->L_Leg_Info.VMC.Phi0 -  Control_Info->L_Leg_Info.VMC.Phi3);
//    Phi1_2=  ((Control_Info->L_Leg_Info.VMC.Phi1) - Control_Info->L_Leg_Info.VMC.Phi2);
//    Control_Info->L_Leg_Info.SendValue.T1 = -(((Control_Info->L_Leg_Info.VMC.L1 * arm_sin_f32(Phi1_2))*(Control_Info->L_Leg_Info.F * Control_Info->L_Leg_Info.VMC.L0 * arm_sin_f32(Phi0_3)
// 	                                         +   Control_Info->L_Leg_Info.Tp * arm_cos_f32(Phi0_3))) / (Control_Info->L_Leg_Info.VMC.L0*arm_sin_f32(Phi2_3)));
	
//    Phi0_2 =  ( Control_Info->L_Leg_Info.VMC.Phi0 -  Control_Info->L_Leg_Info.VMC.Phi2);
//    Phi3_4 =  ( Control_Info->L_Leg_Info.VMC.Phi3 - (Control_Info->L_Leg_Info.VMC.Phi4));
//    Control_Info->L_Leg_Info.SendValue.T2=  -(((Control_Info->L_Leg_Info.VMC.L4 * arm_sin_f32(Phi3_4))*(Control_Info->L_Leg_Info.F * Control_Info->L_Leg_Info.VMC.L0 * arm_sin_f32(Phi0_2)
// 	                                         +  Control_Info->L_Leg_Info.Tp  * arm_cos_f32(Phi0_2))) / (Control_Info->L_Leg_Info.VMC.L0*arm_sin_f32(Phi2_3)));
	
// 	 Phi2_3=  ( Control_Info->R_Leg_Info.VMC.Phi2 -  Control_Info->R_Leg_Info.VMC.Phi3);
//    Phi0_3=  ( Control_Info->R_Leg_Info.VMC.Phi0 -  Control_Info->R_Leg_Info.VMC.Phi3);
//    Phi1_2=  ((Control_Info->R_Leg_Info.VMC.Phi1) - Control_Info->R_Leg_Info.VMC.Phi2);
//    Control_Info->R_Leg_Info.SendValue.T1 = -(((Control_Info->R_Leg_Info.VMC.L1 * arm_sin_f32(Phi1_2))*(Control_Info->R_Leg_Info.F * Control_Info->R_Leg_Info.VMC.L0 * arm_sin_f32(Phi0_3)
// 	                                         +  Control_Info->R_Leg_Info.Tp * arm_cos_f32(Phi0_3))) /  (Control_Info->R_Leg_Info.VMC.L0*arm_sin_f32(Phi2_3)));
   
// 	 Phi0_2 =  ( Control_Info->R_Leg_Info.VMC.Phi0 -  Control_Info->R_Leg_Info.VMC.Phi2);
//    Phi3_4 =  ( Control_Info->R_Leg_Info.VMC.Phi3 - (Control_Info->R_Leg_Info.VMC.Phi4));
//    Control_Info->R_Leg_Info.SendValue.T2=  -(((Control_Info->R_Leg_Info.VMC.L4 * arm_sin_f32(Phi3_4))*(Control_Info->R_Leg_Info.F * Control_Info->R_Leg_Info.VMC.L0 * arm_sin_f32(Phi0_2)
// 	                                         +  Control_Info->R_Leg_Info.Tp  * arm_cos_f32(Phi0_2))) /  (Control_Info->R_Leg_Info.VMC.L0*arm_sin_f32(Phi2_3)));

/*
偏置并联结构转换器



*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------											 
//以下部分不用改，串并联通用部分
//轮子电机的电流值
	 Control_Info->L_Leg_Info.SendValue.Current = (int16_t)( Control_Info->L_Leg_Info.T * 1000.f);
	 Control_Info->R_Leg_Info.SendValue.Current = (int16_t)( -Control_Info->R_Leg_Info.T * 1000.f);
	 
	 VAL_LIMIT(Control_Info->L_Leg_Info.SendValue.Current,-14000,14000);
	 VAL_LIMIT(Control_Info->R_Leg_Info.SendValue.Current,-14000,14000); 
  
	//  VAL_LIMIT(Control_Info->L_Leg_Info.SendValue.T1,-54.f,54.f);
	//  VAL_LIMIT(Control_Info->L_Leg_Info.SendValue.T2,-54.f,54.f);  
    //  VAL_LIMIT(Control_Info->R_Leg_Info.SendValue.T1,-54.f,54.f);
	//  VAL_LIMIT(Control_Info->R_Leg_Info.SendValue.T2,-54.f,54.f);  			
																		
	  
	  VAL_LIMIT(Control_Info->L_Leg_Info.SendValue.T_Calf,-54.f,54.f);
	  VAL_LIMIT(Control_Info->L_Leg_Info.SendValue.T_Thigh,-54.f,54.f);  
      VAL_LIMIT(Control_Info->R_Leg_Info.SendValue.T_Thigh,-54.f,54.f);
	  VAL_LIMIT(Control_Info->R_Leg_Info.SendValue.T_Calf,-54.f,54.f);  			
																				
}