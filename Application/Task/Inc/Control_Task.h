

#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"

typedef enum 
{
  CONTROL_OFF,//关机
  CONTROL_REMOTE,//遥控器控制
 
}Control_Mode_e;//控制模式

typedef enum
{
  CHASSIS_WEAK,//底盘虚弱（未初始化）
  CHASSIS_BALANCE,//平衡
  //CHASSIS_SLIP,
  Chassis_Situation_NUM,

}Chassis_Situation_e;//底盘状态


typedef enum 
{
CHASSIS_FRONT,//front-前进
CHASSIS_SIDE,//side-侧移-?
CHASSIS_SPIN,//spin-旋转
CHASSIS_MODE_NUM,

}Chassis_Mode_e;//底盘模式


typedef enum 
{
  LEG_LENGTH_NORMAL,//正常腿长
  LEG_LENGTH_HIGH,//高腿长
}Leg_Length_Mode_e;


typedef struct //腿的信息
{
		float Fusion_Velocity;		//融合速度
		float Predict_Velocity;		//预测速度
		float W_Velocity;//			角速度
		float LQR_K[2][6];//    	LQR控制器增益矩阵
		float LQR_X[6];//       	LQR状态向量(哈工程建模)-未来计划更新为上交建模
		float LQR_Output[2][6];//	LQR输出
    	float Gravity_Compensation;//重力补偿？
	
  struct//VMC (virtual model control)与五连杆运动学解算
  {
	//1，五连杆的机械参数(物理参数，固定不变)
		float L1,L2,L3,L4,L5;
	//2，虚拟腿参数（随时间变化）
		float L0;          		// 当前虚拟腿长度
		float L0_dot;       	// 腿长变化率
		float Last_L0;      	// 上一周期腿长（差分计算）
		float Target_L0;    	// 目标腿长（遥控设定）
	//3. 关节角度（输入状态）
		float Phi1,Phi4;// 		大腿关节角度（电机测量）
	//4. 计算关节角度（运动学解算）		
		float Phi2, Phi3;     	// 膝关节角度（解算得到）
		float Phi0;           	// 虚拟腿角度（计算输出）
		float Last_Phi0;      	// 上一周期虚拟腿角度
	//5. 角度变化率：
		float Phi1_dot, Phi4_dot; // 左右腿大腿关节角速度（由关节电机测量得到）
		float Phi0_dot;           // 虚拟腿旋转角速度（由Phi0差分或通过雅可比计算得到）
	//6. 几何关系（运动学计算）	
		float X_D_X_B, Y_D_Y_B;  // BD向量分量
		float X_B, Y_B;          // B点坐标
		float X_D, Y_D;          // D点坐标
		float X_C, Y_C;          // C点（足端）坐标
	//7. 运动学解算（足端状态）
		float X_C_dot, Y_C_dot;  // 足端速度
	//8. 中间计算变量（优化用）
		float A0, B0, C0;      // 二次方程系数
		float LBD_2, LBD;       // BD距离平方/实际值
		float Sqrt_Cache;       // 平方根缓存
  }VMC;
	
	struct
    {
	float Phi;//             目标底盘倾斜角
	float Phi_dot;//         目标底盘倾斜角速度
    float Chassis_Position;//目标底盘位置
    float Chassis_Velocity;//目标底盘速度
    float Theta;//           目标虚拟腿倾斜角      
    float Theta_dot;//       目标虚拟腿倾斜角速度
	}Target;//X状态向量的目标值
  
  struct
  {
		
	float Phi;//				IMU直接测量
	float Phi_dot;//			IMU陀螺仪
	float Last_Phi;/*​功能​：1-差分计算角速度（备用）2-滤波处理历史数据--​更新规则​：每周期结束时存储当前Phi*/
    float Chassis_Position;//计算方式​：position += velocity * Δt  // 速度积分
    float Chassis_Velocity;//轮速计+IMU融合滤波
	float Theta;//计算方式​：Theta = (π/2 - Phi0) - Phi  // 几何关系
    float Last_Theta;
    float Theta_dot;

	float F;//​物理意义​：沿虚拟腿方向的推力（N）​计算方式​：关节扭矩逆动力学解算
	float Tp;//​计算方式​：Tp =L0×(关节扭矩分量差)
  }Measure;
	
	struct
  {
	float FN;//物理意义​：腿部在垂直方向上施加给地面的力（牛顿)-判断腿部是否接触地面;计算重心分配
	float P;//压力值--应用场景​：打滑检测（压力突变）地形识别（压力分布
	bool Flag;//是否处于支撑状态--控制作用​：1.切换控制模式（支撑腿/摆动腿）--2.调整LQR增益
 
  }Support;
	
	struct{
	
	float Balance_Tp;/*
					    平衡扭矩
						​物理意义​：用于维持机器人平衡的髋关节扭矩
​						 计算来源​：LQR控制器输出
​						 控制目标​：
						抵抗底盘倾斜（φ）
						稳定虚拟腿角度（θ）*/
	float Leg_Coordinate_Tp;/*腿部协调扭矩
​						物理意义​：协调双腿运动的同步扭矩
​						​计算来源​：PID控制器（PID_Leg_Coordinate）
​						控制目标​：
					   保持双腿对称运动
					   防止"劈叉"现象
​						典型值​：±5 N·m（根据双腿角度差）*/
	float Balance_T;/*平衡力矩
​						物理意义​：维持位置平衡的轮毂驱动力矩
​						计算来源​：LQR控制器输出
​						控制目标​：
​						抗干扰
​						位置保持*/
	float Turn_T;/*转向力矩
​						物理意义​：实现机器人转向的差动力矩
​						计算来源​：偏航PID控制器（PID_Yaw）*/
	float Roll_F;/*滚转力
​						物理意义​：抵抗左右倾斜的补偿力
​						计算方式​：Roll_F = -(Roll_Angle + 0.4°) × 50 N/°(暂定)
​						控制目标​：
						平衡左右腿负载
						防止侧向倾倒
​						特殊处理​：支撑腿禁用此分量
*/
	float Leg_Length_F;/*腿长控制力
​						物理意义​：维持目标腿长的推力
​						计算来源​：腿长PID控制器（PID_Leg_length_F）
​						控制目标​：
​						跟踪目标腿长（0.17m/0.38m）
​						适应不同地形高度
​						重力补偿​：与重力补偿力协同工作*/
	}Moment;/*
​模式自适应​：
工作模式	激活分量
平衡模式	Balance_T/Tp
行走模式	Turn_T + Leg_Coordinate_Tp
高腿模式	Leg_Length_F主导*/
  
 struct{
	float Wheel;
	float W;//计算方式​：W = Wheel - dφ + dθ  
	float X;//X = W × 轮半径  // 0.055m  特性​：瞬时位移量--需积分得总位移--系统图位置​：状态3（x）的原始值
	float Body;//车体速度
					/*物理意义​：底盘中心的线速度
					​计算方式:Body = X  // 基础版本?
 					高级版本包含运动学补偿：
					Body = X + L0×dθ×cosθ + L0_dot×sinθ*/
	float Predict;//目的​：补偿传感器延迟--提高系统响应速度
						//​计算方式​：Predict = Fusion + Accel × Δt  // Δt=0.001s​
	float Fusion; //​计算方式​：Fusion = 0.8×Predict + 0.2×Body  // 一阶滞后滤波--系统图位置​：状态4（dx）的最终值
	}Velocity;/*
1. 状态反馈
x˙=Fusion→状态4输入
2. 前馈控制
控制量=Kp×(目标速度−Fusion)+Kf×目标速度
3. 安全监控
异常情况	检测方式
打滑	Wheel↑ 但 Fusion↓
卡死	Wheel=0 但 Body≠0
过速	Fusion > 安全阈值
*/
	
	struct{
	
   	 float T1;
	 float T2;
	 int16_t Current;
	}SendValue;
	
	float T;
			//总力矩计算方式​：T = Balance_T + Turn_T
			/*	分量	物理意义	控制目标
			Balance_T	平衡力矩	维持位置稳定
			Turn_T	    转向力矩	实现转向控制*/
	float Tp;
			//总扭矩​计算方式​：Tp = Balance_Tp + Leg_Coordinate_Tp
			/*组成分量​：
				分量	物理意义	控制目标
			Balance_Tp	平衡扭矩	稳定躯干姿态
	Leg_Coordinate_Tp	协调扭矩	同步双腿运动*/
	float F;
			//总推力 F = Leg_Length_F + Roll_F + Gravity_Compensation
				/*组成分量​：
				分量	物理意义	控制目标
		Leg_Length_F	腿长控制力	维持目标腿长
				Roll_F	滚转力	    抵抗侧倾
Gravity_Compensation	重力补偿	抵消自重*/

/*模式适应性
工作模式	T主导	Tp主导	F主导
平衡模式	60%		30%		10%
行走模式	40%		40%		20%
高抬腿		20%		30%		50%
转向模式	70%		20%		10%
(暂定)*/
}Leg_Info_Typedef;
//-------------------------------------------------------------------------

typedef struct{
    
	
	Control_Mode_e   Control_Mode;			// 当前控制模式
	Chassis_Situation_e  Chassis_Situation; // 底盘状态
    Chassis_Mode_e Chassis_Mode;			// 底盘运动模式
    Leg_Length_Mode_e  Leg_Length_Mode;		// 腿部长度模式
  	float VDC;								// 电压值
	 //初始化
	struct
  { 
		bool IF_Begin_Init;//(change)
						/*​控制逻辑​：
							if(遥控器指令 == 初始化命令) {
    						Begin_Init = true;
							}*/
		bool Motor_Enable;
						/*初始化完成前：强制禁用（false）
						初始化完成后：自动使能（true）*/
		//关节初始化
		struct{
		 bool IF_Joint_Reduction_Flag[4];//四关节减速到位标志(change)
							 /*判定条件​：
		 					if(关节角度 < 安全阈值) {
    						Joint_Reduction_Flag[i] = true;
							}*/
		 bool IF_Joint_Init;//关节初始化完成标志
							 /*​判定逻辑​：
							if(所有Joint_Reduction_Flag == true) {
    						IF_Joint_Init = true;
								}*/
		}Joint_Init;
		//平衡初始化	
		struct{
			bool IF_Balance_Init;
			uint16_t Balance_Right_Cnt;
							/*功能​：平衡右转计数器
								​用途​：记录右转调整次数
								防止振荡（超过阈值强制完成）
​								典型阈值​：10-20次*/
		}Balance_Init;

        // 偏航初始化 
	   struct{
			bool IF_Yaw_Init; // 偏航初始化完成标志
			uint16_t Yaw_Right_Cnt; // 偏航右转计数
		}Yaw_Init;
		
		
	 struct{
		bool IF_Velocity_Init;
							/*初始化内容​：
							轮速计零偏校准
							速度滤波器重置
							里程计清零*/
		uint16_t Velocity_Cnt;//速度初始化计时器

	 }Velocity_Init;//速度初始化
	 				/*while(Velocity_Cnt < 100) { // 100ms初始化
    				采集速度样本;
    					计算零偏;
    				Velocity_Cnt++;
					}*/
		
	}Init;/*
状态互锁​：
前序系统未完成，后续系统不启动
任一子系统失败则终止整个流程
​断电记忆​：
关键状态非易失存储
异常重启时恢复进度
异常处理机制
异常类型	检测方式					处理措施
关节卡死	关节标志超时未置位			尝试反向运动
平衡失败	Balance_Right_Cnt超限		切换弱平衡模式
偏航异常	传感器数据超范围			使用IMU备份
速度漂移	零偏过大					重新校准*/

// 电机在线状态
	struct{
		bool Enable_Flag;
	}Motor_Online;
	
	
	
//左右腿信息	
  	Leg_Info_Typedef L_Leg_Info;

	Leg_Info_Typedef R_Leg_Info;
	
	float Yaw_Err;			//当前偏航角与目标偏航角的差值-转向控制的主要输入
							//决定转向力矩大小
							//	单位​：弧度（rad）
	float Target_Yaw;		// 目标偏航角

	float Accel;			// 加速度
	float Fusion_Velocity;	// 融合速度
	float Chassis_Velocity; // 底盘速度
							/*物理意义​：底盘中心的实际移动速度
							​测量方式​：轮速计主导
							简单积分计算*/
	float Predict_Velocity; // 预测速度
	float Target_Velocity;  // 目标速度
	float K; 				// 控制器增益

 
  // 偏航控制
struct{
    float Angle;   // 偏航角
	float Velocity;// 偏航速度
    float Err;     // 误差
	 }Yaw;
 
	
/*		
	struct
  {
		bool Begin_Climb_Flag;
		bool Climb_Flag;
		uint16_t Cnt;
    bool Climb_Complete_Flag;
  }Climb;
	
	struct
  {
    uint16_t Slip_Cnt;
		bool Slip_Flag;
		
  }Slip;
*/
}Control_Info_Typedef;
	

extern Control_Info_Typedef Control_Info;

#endif //CONTROL_TASK_H




