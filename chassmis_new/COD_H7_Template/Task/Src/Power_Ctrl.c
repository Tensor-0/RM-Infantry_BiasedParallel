#include "Power_Ctrl.h"
RLS_Info_TypeDef RLS_Power_Info;

void PowerCtrl_Init(PowerCtrl_Typedef *PowerCtrl_Info, Chassis_Tpye_e Type, float Lamda, float P, float PowerCtrl_Param[POWERCTRL_TYPE_NUM])
{
    if (Type == MACUNUM)
    {
        RLS_Init(&RLS_Power_Info, 2, 1, Lamda, P);
        PowerCtrl_Info->Param.K1 = PowerCtrl_Param[0];
        PowerCtrl_Info->Param.K2 = PowerCtrl_Param[1];
        PowerCtrl_Info->Param.K3 = PowerCtrl_Param[2];
        PowerCtrl_Info->Param.Err_Lower = PowerCtrl_Param[3];
        PowerCtrl_Info->Param.Err_Upper = PowerCtrl_Param[4];
        RLS_Power_Info.Data.W[0] = PowerCtrl_Info->Param.K1;
        RLS_Power_Info.Data.W[1] = PowerCtrl_Info->Param.K2;
    }
    else if (Type == STEER)
    {
        RLS_Init(&RLS_Power_Info, 4, 1, Lamda, P);
        PowerCtrl_Info[0].Param.K1 = PowerCtrl_Param[0];
        PowerCtrl_Info[0].Param.K2 = PowerCtrl_Param[1];
        PowerCtrl_Info[0].Param.K3 = PowerCtrl_Param[2];
        PowerCtrl_Info[0].Param.Err_Lower = PowerCtrl_Param[3];
        PowerCtrl_Info[0].Param.Err_Upper = PowerCtrl_Param[4];
        PowerCtrl_Info[1].Param.K1 = PowerCtrl_Param[0];
        PowerCtrl_Info[1].Param.K2 = PowerCtrl_Param[1];
        PowerCtrl_Info[1].Param.K3 = PowerCtrl_Param[2];
        PowerCtrl_Info[1].Param.Err_Lower = PowerCtrl_Param[3];
        PowerCtrl_Info[1].Param.Err_Upper = PowerCtrl_Param[4];
        RLS_Power_Info.Data.W[0] = PowerCtrl_Info[0].Param.K1;
        RLS_Power_Info.Data.W[1] = PowerCtrl_Info[0].Param.K2;
        RLS_Power_Info.Data.W[2] = PowerCtrl_Info[1].Param.K1;
        RLS_Power_Info.Data.W[3] = PowerCtrl_Info[1].Param.K2;
    }
}
void PowerCtrl(PowerCtrl_Typedef *PowerCtrl_Info, PID_Info_TypeDef *Pid, DJI_Motor_Info_Typedef *Chassis_Motor)
{
    for (int i = 0; i < 4; i++)
    {
        PowerCtrl_Info->Target.Omiga[i] = (Chassis_Motor[i].Data.velocity / 9.55f);    // 转速转角速度
        PowerCtrl_Info->Target.Torque[i] = (Chassis_Motor[i].Data.current * 3.662e-4); // 电流转转矩
        PowerCtrl_Info->Target.Omiga_2[i] = powf(PowerCtrl_Info->Target.Omiga[i], 2.f);
        PowerCtrl_Info->Target.Torque_2[i] = powf(PowerCtrl_Info->Target.Torque[i], 2.f);
        PowerCtrl_Info->Target.Power_In[i] = (PowerCtrl_Info->Param.K1 * PowerCtrl_Info->Target.Torque_2[i] + PowerCtrl_Info->Param.K2 * PowerCtrl_Info->Target.Omiga_2[i] + PowerCtrl_Info->Target.Omiga[i] * PowerCtrl_Info->Target.Torque[i]); // 根据模型预测单个电机的功率
    }

    PowerCtrl_Info->Sum.Torque2_Sum = PowerCtrl_Info->Target.Torque_2[0] + PowerCtrl_Info->Target.Torque_2[1] + PowerCtrl_Info->Target.Torque_2[2] + PowerCtrl_Info->Target.Torque_2[3];                          // 计算输出总力矩平方和
    PowerCtrl_Info->Sum.Omiga2_Sum = PowerCtrl_Info->Target.Omiga_2[0] + PowerCtrl_Info->Target.Omiga_2[1] + PowerCtrl_Info->Target.Omiga_2[2] + PowerCtrl_Info->Target.Omiga_2[3];                               // 计算输出总角速度平方和
    PowerCtrl_Info->Sum.Power_Sum = PowerCtrl_Info->Target.Power_In[0] + PowerCtrl_Info->Target.Power_In[1] + PowerCtrl_Info->Target.Power_In[2] + PowerCtrl_Info->Target.Power_In[3] + PowerCtrl_Info->Param.K3; // 计算实际功率

    RLS_Power_Info.Data.X[0] = PowerCtrl_Info->Sum.Torque2_Sum;
    RLS_Power_Info.Data.X[1] = PowerCtrl_Info->Sum.Omiga2_Sum;
    RLS_Power_Info.Data.U[0] = PowerCtrl_Info->Sum.Power_Sum;
    RLS_Power_Info.Data.Y[0] = Referee_Info.power_heat_data.chassis_power;
    // RLS输入参数更新
    RLS_Update(&RLS_Power_Info);
    PowerCtrl_Info->Param.K1 = RLS_Power_Info.Data.W[0];
    PowerCtrl_Info->Param.K2 = RLS_Power_Info.Data.W[1];
    // RLS输出参数更新

    PowerCtrl_Info->Sum.Power_Sum = PowerCtrl_Info->Target.Power_In[0] + PowerCtrl_Info->Target.Power_In[1] + PowerCtrl_Info->Target.Power_In[2] + PowerCtrl_Info->Target.Power_In[3] + PowerCtrl_Info->Param.K3; // 根据更新的参数，重新计算实际功率

    for (int i = 0; i < 4; i++)
        PowerCtrl_Info->Err[i] = Pid[i].Err[0];
    // 输入PID误差
    PowerCtrl_Info->Sum.Err_Sum = fabsf(PowerCtrl_Info->Err[FL]) + fabsf(PowerCtrl_Info->Err[FB]) + fabsf(PowerCtrl_Info->Err[RB]) + fabsf(PowerCtrl_Info->Err[RL]);
    // 计算PID误差和
    if (PowerCtrl_Info->Sum.Err_Sum > PowerCtrl_Info->Param.Err_Upper)
        PowerCtrl_Info->K = 1;
    else if (PowerCtrl_Info->Sum.Err_Sum < PowerCtrl_Info->Param.Err_Lower)
        PowerCtrl_Info->K = 0;
    else
        PowerCtrl_Info->K = (PowerCtrl_Info->Sum.Err_Sum - PowerCtrl_Info->Param.Err_Lower) / (PowerCtrl_Info->Param.Err_Upper - PowerCtrl_Info->Param.Err_Lower);
    // 计算分配因子K

    for (int i = 0; i < 4; i++)
    {
        PowerCtrl_Info->Menbership[i] = (PowerCtrl_Info->K * (fabs(PowerCtrl_Info->Err[i]) / PowerCtrl_Info->Sum.Err_Sum) + (1 - PowerCtrl_Info->K) * (fabs(PowerCtrl_Info->Target.Power_In[i]) / PowerCtrl_Info->Sum.Power_Sum)); // 港科大P分配
        PowerCtrl_Info->Power_Limit[i] = PowerCtrl_Info->Menbership[i] * PowerCtrl_Info->Power_Max;
    }

    if (PowerCtrl_Info->Sum.Power_Sum >= PowerCtrl_Info->Power_Max)
    {
        for (int i = 0; i < 4; i++)
        {
            PowerCtrl_Info->A = PowerCtrl_Info->Param.K1;
            PowerCtrl_Info->B = PowerCtrl_Info->Target.Omiga[i];
            PowerCtrl_Info->C = PowerCtrl_Info->Target.Omiga_2[i] * PowerCtrl_Info->Param.K2 + PowerCtrl_Info->Param.K3 * 0.25 - PowerCtrl_Info->Power_Limit[i];
            PowerCtrl_Info->Delta = powf(PowerCtrl_Info->B, 2.f) - 4 * PowerCtrl_Info->A * PowerCtrl_Info->C;
            // 二次方程系数带入
            if (isnan(PowerCtrl_Info->Delta) == 1 || isinf(PowerCtrl_Info->Delta) == 1)
                PowerCtrl_Info->Delta = 0;

            if (PowerCtrl_Info->Delta >= 0)
            {
                PowerCtrl_Info->Sqrt = sqrtf(PowerCtrl_Info->Delta);
                if (Pid[i].Output >= 0)
                {
                    PowerCtrl_Info->Torque[i] = (-PowerCtrl_Info->B + PowerCtrl_Info->Sqrt) / (2 * PowerCtrl_Info->A);
                    PowerCtrl_Info->Output[i] = (PowerCtrl_Info->Torque[i] / 3.662-4);
                }
                else
                {
                    PowerCtrl_Info->Torque[i] = (-PowerCtrl_Info->B - PowerCtrl_Info->Sqrt) / (2 * PowerCtrl_Info->A);
                    PowerCtrl_Info->Output[i] = (PowerCtrl_Info->Torque[i] / 3.662e-4);
                }
            } // 有实数解
            else
            {
                if (Pid[i].Output >= 0)
                {
                    PowerCtrl_Info->Torque[i] = (-PowerCtrl_Info->B) / (2 * PowerCtrl_Info->A);
                    PowerCtrl_Info->Output[i] = (PowerCtrl_Info->Torque[i] / 3.662e-4);
                }
                else
                {
                    PowerCtrl_Info->Torque[i] = (PowerCtrl_Info->B) / (2 * PowerCtrl_Info->A);
                    PowerCtrl_Info->Output[i] = (PowerCtrl_Info->Torque[i] / 3.662e-4);
                }
            } // 无实数解
        }
    }
    else
        for (int i = 0; i < 4; i++)
            PowerCtrl_Info->Output[i] = Pid[i].Output;
}
void PowerCtrl_Steer(PowerCtrl_Typedef *PowerCtrl_Info, PID_Info_TypeDef *Pid_Course, PID_Info_TypeDef *Pid_Travel, DJI_Motor_Info_Typedef *Motor_Course, DJI_Motor_Info_Typedef *Motor_Travel)
{
    if (PowerCtrl_Info->Chassis_Type == STEER)
    {
        for (int i = 0; i < 4; i++)
        {
            PowerCtrl_Info[0].Target.Omiga[i] = (Motor_Course[i].Data.velocity / 9.55f);
            PowerCtrl_Info[0].Target.Torque[i] = ((Motor_Course[i].Data.current) * 8.697e-4);
            PowerCtrl_Info[0].Target.Omiga_2[i] = powf(PowerCtrl_Info[0].Target.Omiga[i], 2.f);
            PowerCtrl_Info[0].Target.Torque_2[i] = powf(PowerCtrl_Info[0].Target.Torque[i], 2.f);

            PowerCtrl_Info[0].Target.Power_In[i] = (PowerCtrl_Info[0].Param.K1 * PowerCtrl_Info[0].Target.Torque_2[i] + PowerCtrl_Info[0].Param.K2 * PowerCtrl_Info[0].Target.Omiga_2[i] + PowerCtrl_Info[0].Target.Omiga[i] * PowerCtrl_Info[0].Target.Torque[i]);

            PowerCtrl_Info[1].Target.Omiga[i] = (Motor_Travel[i].Data.velocity / 9.55f);
            PowerCtrl_Info[1].Target.Torque[i] = ((Motor_Travel[i].Data.current) * 8.697e-4);
            PowerCtrl_Info[1].Target.Omiga_2[i] = powf(PowerCtrl_Info[1].Target.Omiga[i], 2.f);
            PowerCtrl_Info[1].Target.Torque_2[i] = powf(PowerCtrl_Info[1].Target.Torque[i], 2.f);

            PowerCtrl_Info[1].Target.Power_In[i] = (PowerCtrl_Info[1].Param.K1 * PowerCtrl_Info[1].Target.Torque_2[i] + PowerCtrl_Info[1].Param.K2 * PowerCtrl_Info[1].Target.Omiga_2[i] + PowerCtrl_Info[1].Target.Omiga[i] * PowerCtrl_Info[1].Target.Torque[i]);
        }

        PowerCtrl_Info[0].Sum.Torque2_Sum = PowerCtrl_Info[0].Target.Torque_2[0] + PowerCtrl_Info[0].Target.Torque_2[1] + PowerCtrl_Info[0].Target.Torque_2[2] + PowerCtrl_Info[0].Target.Torque_2[3];
        PowerCtrl_Info[0].Sum.Omiga2_Sum = PowerCtrl_Info[0].Target.Omiga_2[0] + PowerCtrl_Info[0].Target.Omiga_2[1] + PowerCtrl_Info[0].Target.Omiga_2[2] + PowerCtrl_Info[0].Target.Omiga_2[3];
        PowerCtrl_Info[0].Sum.Power_Sum = PowerCtrl_Info[0].Target.Power_In[0] + PowerCtrl_Info[0].Target.Power_In[1] + PowerCtrl_Info[0].Target.Power_In[2] + PowerCtrl_Info[0].Target.Power_In[3] + PowerCtrl_Info[0].Param.K3;
        PowerCtrl_Info[0].Power_Allin = PowerCtrl_Info[0].Sum.Power_Sum;
        PowerCtrl_Info[0].Power_Max = 0.8 * (Referee_Info.robot_status.chassis_power_limit);
        VAL_LIMIT(PowerCtrl_Info[0].Power_Allin, -PowerCtrl_Info[0].Power_Max, PowerCtrl_Info[0].Power_Max);

        PowerCtrl_Info[1].Sum.Torque2_Sum = PowerCtrl_Info[1].Target.Torque_2[0] + PowerCtrl_Info[1].Target.Torque_2[1] + PowerCtrl_Info[1].Target.Torque_2[2] + PowerCtrl_Info[1].Target.Torque_2[3];
        PowerCtrl_Info[1].Sum.Omiga2_Sum = PowerCtrl_Info[1].Target.Omiga_2[0] + PowerCtrl_Info[1].Target.Omiga_2[1] + PowerCtrl_Info[1].Target.Omiga_2[2] + PowerCtrl_Info[1].Target.Omiga_2[3];
        PowerCtrl_Info[1].Sum.Power_Sum = PowerCtrl_Info[1].Target.Power_In[0] + PowerCtrl_Info[1].Target.Power_In[1] + PowerCtrl_Info[1].Target.Power_In[2] + PowerCtrl_Info[1].Target.Power_In[3] + PowerCtrl_Info[1].Param.K3;
        PowerCtrl_Info[1].Power_Allin = PowerCtrl_Info[1].Sum.Power_Sum;
        PowerCtrl_Info[1].Power_Max = Referee_Info.robot_status.chassis_power_limit - fabs(PowerCtrl_Info[0].Power_Allin);
        VAL_LIMIT(PowerCtrl_Info[1].Power_Allin, -PowerCtrl_Info[1].Power_Max, PowerCtrl_Info[1].Power_Max);

        RLS_Power_Info.Data.U[0] = PowerCtrl_Info[0].Power_Allin + PowerCtrl_Info[1].Power_Allin;
        RLS_Power_Info.Data.Y[0] = Referee_Info.power_heat_data.chassis_power;
        RLS_Power_Info.Data.X[0] = PowerCtrl_Info[0].Sum.Torque2_Sum;
        RLS_Power_Info.Data.X[1] = PowerCtrl_Info[0].Sum.Omiga2_Sum;
        RLS_Power_Info.Data.X[2] = PowerCtrl_Info[1].Sum.Torque2_Sum;
        RLS_Power_Info.Data.X[3] = PowerCtrl_Info[1].Sum.Omiga2_Sum;
        RLS_Update(&RLS_Power_Info);

        PowerCtrl_Info[0].Param.K1 = RLS_Power_Info.Data.W[0];
        PowerCtrl_Info[0].Param.K2 = RLS_Power_Info.Data.W[1];
        PowerCtrl_Info[1].Param.K1 = RLS_Power_Info.Data.W[2];
        PowerCtrl_Info[1].Param.K2 = RLS_Power_Info.Data.W[3];
        PowerCtrl_Info[0].Sum.Err_Sum = fabsf(Pid_Course[FL].Err[0]) + fabsf(Pid_Course[FB].Err[0]) + fabsf(Pid_Course[RB].Err[0]) + fabsf(Pid_Course[RL].Err[0]);
        PowerCtrl_Info[1].Sum.Err_Sum = fabsf(Pid_Travel[FL].Err[0]) + fabsf(Pid_Travel[FB].Err[0]) + fabsf(Pid_Travel[RB].Err[0]) + fabsf(Pid_Travel[RL].Err[0]);

        for (int i = 0; i < 4; i++)
            PowerCtrl_Info[0].Err[i] = Pid_Course[i].Err[0];
        // 输入PID误差
        PowerCtrl_Info[0].Sum.Err_Sum = fabsf(PowerCtrl_Info[0].Err[FL]) + fabsf(PowerCtrl_Info[0].Err[FB]) + fabsf(PowerCtrl_Info[0].Err[RB]) + fabsf(PowerCtrl_Info[0].Err[RL]);
        // 计算PID误差和
        if (PowerCtrl_Info[0].Sum.Err_Sum > PowerCtrl_Info[0].Param.Err_Upper)
            PowerCtrl_Info[0].K = 1;
        else if (PowerCtrl_Info[0].Sum.Err_Sum < PowerCtrl_Info[0].Param.Err_Lower)
            PowerCtrl_Info[0].K = 0;
        else
        PowerCtrl_Info[0].K = (PowerCtrl_Info[0].Sum.Err_Sum - PowerCtrl_Info[0].Param.Err_Lower) / (PowerCtrl_Info[0].Param.Err_Upper - PowerCtrl_Info[0].Param.Err_Lower);

        for (int i = 0; i < 4; i++)
            PowerCtrl_Info[1].Err[i] = Pid_Travel[i].Err[0];
        // 输入PID误差
        PowerCtrl_Info[1].Sum.Err_Sum = fabsf(PowerCtrl_Info[1].Err[FL]) + fabsf(PowerCtrl_Info[1].Err[FB]) + fabsf(PowerCtrl_Info[1].Err[RB]) + fabsf(PowerCtrl_Info[1].Err[RL]);
        // 计算PID误差和
        if (PowerCtrl_Info[1].Sum.Err_Sum > PowerCtrl_Info[1].Param.Err_Upper)
            PowerCtrl_Info[1].K = 1;
        else if (PowerCtrl_Info[0].Sum.Err_Sum < PowerCtrl_Info[1].Param.Err_Lower)
            PowerCtrl_Info[1].K = 0;
        else
            PowerCtrl_Info[1].K = (PowerCtrl_Info[1].Sum.Err_Sum - PowerCtrl_Info[1].Param.Err_Lower) / (PowerCtrl_Info[1].Param.Err_Upper - PowerCtrl_Info[1].Param.Err_Lower);
        // 计算分配因子K

        for (int i = 0; i < 4; i++)
        {
        PowerCtrl_Info[0].Menbership[i] = (PowerCtrl_Info[0].K * (fabs(PowerCtrl_Info[0].Err[i]) / PowerCtrl_Info[0].Sum.Err_Sum) + (1 - PowerCtrl_Info[0].K) * (fabs(PowerCtrl_Info[0].Target.Power_In[i]) / PowerCtrl_Info[0].Sum.Power_Sum));
            VAL_LIMIT(PowerCtrl_Info[0].Menbership[i], 0, 1);
        PowerCtrl_Info[1].Menbership[i] = (PowerCtrl_Info[1].K * (fabs(PowerCtrl_Info[1].Err[i]) / PowerCtrl_Info[1].Sum.Err_Sum) + (1 - PowerCtrl_Info[1].K) * (fabs(PowerCtrl_Info[1].Target.Power_In[i]) / PowerCtrl_Info[1].Sum.Power_Sum));
            VAL_LIMIT(PowerCtrl_Info[1].Menbership[i], 0, 1);
        PowerCtrl_Info[0].Power_Limit[i] = PowerCtrl_Info[0].Menbership[i] * PowerCtrl_Info[0].Power_Allin; // 港科大P分配
        PowerCtrl_Info[1].Power_Limit[i] = PowerCtrl_Info[1].Menbership[i] * PowerCtrl_Info[1].Power_Allin; // 港科大P分配
        }

        if (PowerCtrl_Info[0].Sum.Power_Sum >= PowerCtrl_Info[0].Power_Max)
        {

            for (int i = 0; i < 4; i++)
            {
                PowerCtrl_Info[0].A = PowerCtrl_Info[0].Param.K1;
                PowerCtrl_Info[0].B = PowerCtrl_Info[0].Target.Omiga[i];
                PowerCtrl_Info[0].C = PowerCtrl_Info[0].Target.Omiga_2[i] * PowerCtrl_Info[0].Param.K2 + PowerCtrl_Info[0].Param.K3 - PowerCtrl_Info[0].Power_Limit[i];
                PowerCtrl_Info[0].Delta = powf(PowerCtrl_Info[0].B, 2.f) - 4 * PowerCtrl_Info[0].A * PowerCtrl_Info[0].C;
                if (isnan(PowerCtrl_Info[0].Delta) == 1 || isinf(PowerCtrl_Info[0].Delta) == 1)
                    PowerCtrl_Info[0].Delta = 0;
                if (PowerCtrl_Info[0].Delta >= 0)
                {
                    PowerCtrl_Info[0].Sqrt = sqrtf(PowerCtrl_Info[0].Delta);
                    if (Pid_Course[i].Output >= 0)
                    {
                        PowerCtrl_Info[0].Torque[i] = (-PowerCtrl_Info[0].B + PowerCtrl_Info[0].Sqrt) / (2 * PowerCtrl_Info[0].A);
                        PowerCtrl_Info[0].Output[i] = (PowerCtrl_Info[0].Torque[i] / 8.697e-4);
                    }
                    else
                    {
                        PowerCtrl_Info[0].Torque[i] = (-PowerCtrl_Info[0].B - PowerCtrl_Info[0].Sqrt) / (2 * PowerCtrl_Info[0].A);
                        PowerCtrl_Info[0].Output[i] = (PowerCtrl_Info[0].Torque[i] / 8.697e-4);
                    }
                }
                else
                {
                        PowerCtrl_Info[0].Torque[i] = (-PowerCtrl_Info[0].B) / (2 * PowerCtrl_Info[0].A);
                        PowerCtrl_Info[0].Output[i] = (PowerCtrl_Info[0].Torque[i] / 8.697e-4);
                }
            }
        }
        else
            for (int i = 0; i < 4; i++)
                PowerCtrl_Info->Output[i] = Pid_Course[i].Output;

        if (PowerCtrl_Info[1].Sum.Power_Sum >= PowerCtrl_Info[1].Power_Max)
        {
            for (int i = 0; i < 4; i++)
            {
                PowerCtrl_Info[1].A = PowerCtrl_Info[1].Param.K1;
                PowerCtrl_Info[1].B = PowerCtrl_Info[1].Target.Omiga[i];
                PowerCtrl_Info[1].C = PowerCtrl_Info[1].Target.Omiga_2[i] * PowerCtrl_Info[1].Param.K2 + PowerCtrl_Info[1].Param.K3 - PowerCtrl_Info[1].Power_Limit[i];
                PowerCtrl_Info[1].Delta = powf(PowerCtrl_Info[1].B, 2.f) - 4 * PowerCtrl_Info[1].A * PowerCtrl_Info[1].C;

                if (isnan(PowerCtrl_Info[1].Delta) == 1 || isinf(PowerCtrl_Info[1].Delta) == 1)
                    PowerCtrl_Info[1].Delta = 0;
                if (PowerCtrl_Info[1].Delta >= 0)
                {
                    PowerCtrl_Info[1].Sqrt = sqrtf(PowerCtrl_Info[1].Delta);
                    if (Pid_Travel[i].Output >= 0)
                    {
                        PowerCtrl_Info[1].Torque[i] = (-PowerCtrl_Info[1].B + PowerCtrl_Info[1].Sqrt) / (2 * PowerCtrl_Info[1].A);
                        Pid_Travel[i].Output = (PowerCtrl_Info[1].Torque[i] / 8.697e-4);
                    }
                    else
                    {
                        PowerCtrl_Info[1].Torque[i] = (-PowerCtrl_Info[1].B - PowerCtrl_Info[1].Sqrt) / (2 * PowerCtrl_Info[1].A);
                        Pid_Travel[i].Output = (PowerCtrl_Info[1].Torque[i] / 8.697e-4);
                    }
                }
                else
                {
                        PowerCtrl_Info[1].Torque[i] = (-PowerCtrl_Info[1].B) / (2 * PowerCtrl_Info[1].A);
                        Pid_Travel[i].Output = (PowerCtrl_Info[1].Torque[i] / 8.697e-4);
                }
            }
        }
        else
            for (int i = 0; i < 4; i++)
                Pid_Travel[i].Output = Pid_Travel[i].Output;
    }
}