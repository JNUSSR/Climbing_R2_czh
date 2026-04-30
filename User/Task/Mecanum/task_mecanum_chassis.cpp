#include "task_mecanum_chassis.h"

#include "crt_wheelchassis.h"
#include "alg_slope.h"

extern CAN_HandleTypeDef hcan1;

static Class_Chassis_Mecanum g_task_mecanum_chassis;
static Class_Slope g_slope_vx;
static Class_Slope g_slope_vy;
static Class_Slope g_slope_vw;

static float g_cmd_vx = 0.0f;
static float g_cmd_vy = 0.0f;
static float g_cmd_vw = 0.0f;

#define CHASSIS_VELOCITY_INCREASE_STEP 0.002f
#define CHASSIS_VELOCITY_DECREASE_STEP 0.003f
#define CHASSIS_ANGULAR_INCREASE_STEP  0.003f
#define CHASSIS_ANGULAR_DECREASE_STEP  0.005f

void Task_Mecanum_Chassis_Init(void)
{
    g_slope_vx.Init(CHASSIS_VELOCITY_INCREASE_STEP, CHASSIS_VELOCITY_DECREASE_STEP, Slope_First_REAL);
    g_slope_vy.Init(CHASSIS_VELOCITY_INCREASE_STEP, CHASSIS_VELOCITY_DECREASE_STEP, Slope_First_REAL);
    g_slope_vw.Init(CHASSIS_ANGULAR_INCREASE_STEP, CHASSIS_ANGULAR_DECREASE_STEP, Slope_First_REAL);

    for (int i = 0; i < 4; i++)
    {
        if (i == 2)
        {
            g_task_mecanum_chassis.Chassis_Motor[i].PID_Omega.Init(85.3f, 97.3f, 0.0f, 0.0f, 10000.0f, 12000.0f);
        }
        else
        {
            g_task_mecanum_chassis.Chassis_Motor[i].PID_Omega.Init(83.3f, 97.3f, 0.0f, 0.0f, 10000.0f, 12000.0f);
        }
    }

    g_task_mecanum_chassis.Chassis_Motor[0].Init(&hcan1, CAN_Motor_ID_0x205, Control_Method_OMEGA, 19.0f, 20.0f);
    g_task_mecanum_chassis.Chassis_Motor[1].Init(&hcan1, CAN_Motor_ID_0x206, Control_Method_OMEGA, 19.0f, 20.0f);
    g_task_mecanum_chassis.Chassis_Motor[2].Init(&hcan1, CAN_Motor_ID_0x207, Control_Method_OMEGA, 19.0f, 20.0f);
    g_task_mecanum_chassis.Chassis_Motor[3].Init(&hcan1, CAN_Motor_ID_0x208, Control_Method_OMEGA, 19.0f, 20.0f);

    Task_Mecanum_Chassis_StopImmediate();
}

void Task_Mecanum_Chassis_SetMotion(float vx, float vy, float vw)
{
    g_cmd_vx = vx;
    g_cmd_vy = vy;
    g_cmd_vw = vw;
}

void Task_Mecanum_Chassis_StopImmediate(void)
{
    Task_Mecanum_Chassis_SetMotion(0.0f, 0.0f, 0.0f);

    g_slope_vx.Set_Now_Real(0.0f);
    g_slope_vy.Set_Now_Real(0.0f);
    g_slope_vw.Set_Now_Real(0.0f);
    g_slope_vx.Set_Target(0.0f);
    g_slope_vy.Set_Target(0.0f);
    g_slope_vw.Set_Target(0.0f);

    g_task_mecanum_chassis.Set_Target_Velocity_X(0.0f);
    g_task_mecanum_chassis.Set_Target_Velocity_Y(0.0f);
    g_task_mecanum_chassis.Set_Target_Omega(0.0f);
}

void Task_Mecanum_Chassis_Task1ms(void)
{
    g_slope_vx.Set_Now_Real(g_slope_vx.Get_Out());
    g_slope_vy.Set_Now_Real(g_slope_vy.Get_Out());
    g_slope_vw.Set_Now_Real(g_slope_vw.Get_Out());

    g_slope_vx.Set_Target(g_cmd_vx);
    g_slope_vy.Set_Target(g_cmd_vy);
    g_slope_vw.Set_Target(g_cmd_vw);

    g_slope_vx.TIM_Calculate_PeriodElapsedCallback();
    g_slope_vy.TIM_Calculate_PeriodElapsedCallback();
    g_slope_vw.TIM_Calculate_PeriodElapsedCallback();

    g_task_mecanum_chassis.Set_Target_Velocity_X(g_slope_vx.Get_Out());
    g_task_mecanum_chassis.Set_Target_Velocity_Y(g_slope_vy.Get_Out());
    g_task_mecanum_chassis.Set_Target_Omega(g_slope_vw.Get_Out());
    g_task_mecanum_chassis.TIM_1ms_Control_PeriodElapsedCallback();

    g_task_mecanum_chassis.Chassis_Motor[0].TIM_PID_PeriodElapsedCallback();
    g_task_mecanum_chassis.Chassis_Motor[1].TIM_PID_PeriodElapsedCallback();
    g_task_mecanum_chassis.Chassis_Motor[2].TIM_PID_PeriodElapsedCallback();
    g_task_mecanum_chassis.Chassis_Motor[3].TIM_PID_PeriodElapsedCallback();
}
