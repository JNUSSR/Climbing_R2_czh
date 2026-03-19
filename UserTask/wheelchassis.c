#include "wheelchassis.h"

#include <math.h>

#include "alg_pid.h"
#include "alg_slope.h"
#include "dvc_motor.h"

// 全局变量定义
MotionControl G_Motion; // 运动控制结构体
Class_Motor_C620 G_Chassis_Motor[4]; // 4个底盘电机对象 (M3508 + C620)
Chassis_Params G_Chassis_Params; // 运动参数

// 斜率规划器对象（用于速度平滑）
Class_Slope Slope_Vx;
Class_Slope Slope_Vy;
Class_Slope Slope_Vw;

extern TIM_HandleTypeDef htim6;
extern CAN_HandleTypeDef hcan1;

// 静态函数声明
static void wheelChassis_Motor_Init(void);
static void wheelChassis_Slope_Init(void);
static void wheelChassis_MotorSpeedControl(float *ExpectSpeed);

#define SPEED_DEADZONE 0.03f // 速度死区

// 底盘参数定义
#define HALF_TRACK_WIDTH 0.243f // 左右轮中心距的一半 (车宽/2)   480mm 485.56mm
#define HALF_WHEEL_BASE 0.195f // 前后轮中心距的一半 (车长/2) 390mm
#define WHEEL_RADIUS 0.076f       // 轮子半径 (m) 直径152mm
#define CHASSIS_L 0.315f      // 底盘旋转中心到轮子中心的距离 (m) 630mm
#define SQRT2_DIV_2 0.70710678f    // √2/2

// 斜率规划参数（每个控制周期的最大变化量）
#define VELOCITY_INCREASE_STEP 0.002f  // 加速步长 (2.0 m/s² * 0.001s = 0.002 m/s)
#define VELOCITY_DECREASE_STEP 0.003f  // 减速步长 (2.0 m/s² * 0.001s = 0.002 m/s)
#define ANGULAR_INCREASE_STEP 0.003f   // 角速度加速步长 (3.0 rad/s² * 0.001s)
#define ANGULAR_DECREASE_STEP 0.005f   // 角速度减速步长 (5.0 rad/s² * 0.001s)


/**
 * @brief 轮式底盘模块总初始化函数。
 *
 * 功能：
 * - 配置并启动CAN通信，包括滤波器和中断。
 * - 调用内部初始化函数，设置所有PID控制器参数和底盘物理参数（如加减速、半径等）。
 * - 将底盘初始状态设置为等待（Wait）。
 *
 * 调用时机：
 * - 在主程序（如main.c）的初始化阶段，所有依赖的硬件（如CAN1）初始化之后，主循环开始之前调用一次。
 *
 * 注意事项：
 * - 这是一个必须调用的函数，它确保了底盘模块所有功能的正常运行前提。
 * - 依赖于CubeMX生成的 `hcan1` 句柄。
 */
void wheelChassis_Init(void)
{
    // 初始化4个底盘电机 (M3508 + C620), CAN ID: 0x201-0x204
    wheelChassis_Motor_Init();
    
    // 初始化斜率规划器
    wheelChassis_Slope_Init();
    
    // 初始化运动控制结构体
    G_Motion.Vx = 0.0f;
    G_Motion.Vy = 0.0f;
    G_Motion.Vw = 0.0f;
}

/**
 * @brief 初始化底盘4个电机对象
 * 
 * 功能：
 * - 初始化4个M3508电机（使用C620电调）
 * - 配置电机CAN ID（0x201-0x204）
 * - 配置电机控制模式为速度环控制
 * - 设置速度环PID参数
 */
void wheelChassis_Motor_Init(void)
{
    // 配置速度环PID参数（所有电机使用相同参数）
    // PID_Omega.Init(Kp, Ki, Kd, Kf, I_Max, Out_Max)
    for (int i = 0; i < 4; i++)
    {
        if ( i == 2 )
        {
            G_Chassis_Motor[i].PID_Omega.Init(85.3f, 97.3f, 0.0f, 0.0f, 10000.0f, 12000.0f);
            continue;
        }
        else
        {
            G_Chassis_Motor[i].PID_Omega.Init(83.3f, 97.3f, 0.0f, 0.0f, 10000.0f, 12000.0f);
        }    
    }
    
    // 初始化4个底盘电机对象 (M3508 + C620)
    // CAN ID: 0x201-0x204，控制模式：速度环
    G_Chassis_Motor[0].Init(&hcan1, CAN_Motor_ID_0x205, Control_Method_OMEGA, 19.0f, 20.0f);
    G_Chassis_Motor[1].Init(&hcan1, CAN_Motor_ID_0x206, Control_Method_OMEGA, 19.0f, 20.0f);
    G_Chassis_Motor[2].Init(&hcan1, CAN_Motor_ID_0x207, Control_Method_OMEGA, 19.0f, 20.0f);
    G_Chassis_Motor[3].Init(&hcan1, CAN_Motor_ID_0x208, Control_Method_OMEGA, 19.0f, 20.0f);
}

/**
 * @brief 初始化斜坡规划器
 * 
 * 功能：
 * - 为Vx, Vy, Vw分别初始化斜坡规划器
 * - 设置加速和减速步长
 * - 使用真实值优先模式（Slope_First_REAL）
 */
void wheelChassis_Slope_Init(void)
{
    // 初始化 Vx 斜坡规划器
    Slope_Vx.Init(VELOCITY_INCREASE_STEP, VELOCITY_DECREASE_STEP, Slope_First_REAL);
    
    // 初始化 Vy 斜坡规划器
    Slope_Vy.Init(VELOCITY_INCREASE_STEP, VELOCITY_DECREASE_STEP, Slope_First_REAL);
    
    // 初始化 Vw 斜坡规划器（角速度可以设置更快的响应）
    Slope_Vw.Init(ANGULAR_INCREASE_STEP, ANGULAR_DECREASE_STEP, Slope_First_REAL);
}

/**
 * @brief 设置4个电机的目标速度
 *
 * 功能：
 * - 使用dvc_motor库的接口设置各电机目标速度
 * - 电机内部PID会自动计算控制量
 *
 * @param ExpectSpeed 期望速度数组（rad/s）
 */
void wheelChassis_MotorSpeedControl(float *ExpectSpeed)
{
    for (int i = 0; i < 4; i++)
    {
        G_Chassis_Motor[i].Set_Target_Omega(ExpectSpeed[i]);
    }
}

float wheelChassis_AbsoluteValue(float num)
{
    if (num < 0)
    {
        return -num;
    }
    return num;
}

/**
 * @brief 设置底盘运动速度
 * 
 * @param vx X方向速度 (m/s)
 * @param vy Y方向速度 (m/s)
 * @param vw 旋转角速度 (rad/s)
 */
void wheelChassis_SetMotion(float vx, float vy, float vw)
{
    G_Motion.Vx = vx;
    G_Motion.Vy = vy;
    G_Motion.Vw = vw;
}

/**
 * @brief 底盘运动控制函数
 * 
 * 功能：
 * - 根据期望的底盘运动速度（Vx, Vy, Vw）
 * - 计算每个全向轮轮组轮的目标转速
 * - 设置电机目标速度
 * 
 * @param motion 期望的底盘运动速度
 */
void wheelChassis_MotionControl(MotionControl motion)
{
    float ExpectSpeed[4];  // 单位：rad/s

    float k = HALF_TRACK_WIDTH + HALF_WHEEL_BASE;  // 旋转解算系数 k = (L/2 + W/2)

    ExpectSpeed[0] = ((motion.Vx - motion.Vy - motion.Vw * k)) / WHEEL_RADIUS;
    
    // 右前轮 (motor_chassis[1])
    ExpectSpeed[1] = (-(motion.Vx + motion.Vy + motion.Vw * k)) / WHEEL_RADIUS;
    
    // 左后轮 (motor_chassis[2])
    ExpectSpeed[2] = (motion.Vx + motion.Vy - motion.Vw * k) / WHEEL_RADIUS;
    
    // 右后轮 (motor_chassis[3])
    ExpectSpeed[3] = (motion.Vx - motion.Vy + motion.Vw * k) / WHEEL_RADIUS;
    // 全向轮运动学逆解 rad/s
    // ExpectSpeed[0] = -motion.Vy - motion.Vx - motion.Vw;
    // ExpectSpeed[1] = +motion.Vy - motion.Vx - motion.Vw;
    // ExpectSpeed[2] = +motion.Vy + motion.Vx - motion.Vw;
    // ExpectSpeed[3] = -motion.Vy + motion.Vx - motion.Vw;
    
    // 设置电机目标速度（dvc_motor内部会进行PID控制）
    wheelChassis_MotorSpeedControl(ExpectSpeed);
}

/**
 * @brief 底盘控制任务主函数
 * 
 * 功能：
 * - 接收上位机发来的目标速度指令
 * - 通过斜坡规划器进行速度平滑处理
 * - 执行底盘运动控制
 * 
 * @param motion 期望的底盘运动速度 (Vx, Vy单位m/s, Vw单位rad/s)
 * 
 * 调用频率：1kHz（与TIMESTEP匹配）
 */
void wheelChassis_Task(MotionControl motion)
{
    // 使用上一次的输出作为当前真实值，避免规划器从0开始
    Slope_Vx.Set_Now_Real(Slope_Vx.Get_Out());
    Slope_Vy.Set_Now_Real(Slope_Vy.Get_Out());
    Slope_Vw.Set_Now_Real(Slope_Vw.Get_Out());
    
    // 设置斜坡规划器的目标值
    Slope_Vx.Set_Target(motion.Vx);
    Slope_Vy.Set_Target(motion.Vy);
    Slope_Vw.Set_Target(motion.Vw);
    
    // 执行斜坡规划计算
    Slope_Vx.TIM_Calculate_PeriodElapsedCallback();
    Slope_Vy.TIM_Calculate_PeriodElapsedCallback();
    Slope_Vw.TIM_Calculate_PeriodElapsedCallback();
    
    // 获取平滑后的速度
    MotionControl motion_smooth;
    motion_smooth.Vx = Slope_Vx.Get_Out();
    motion_smooth.Vy = Slope_Vy.Get_Out();
    motion_smooth.Vw = Slope_Vw.Get_Out();
    
    // 使用平滑后的速度执行控制
    wheelChassis_MotionControl(motion_smooth);
}

/**
 * @brief 立即停车：清零斜坡和电机目标
 *
 * 用于状态切换时快速撤销底盘残留速度，避免离开 WAIT_TRIGGER 后轮子继续转。
 */
void wheelChassis_StopImmediate(void)
{
    MotionControl stop = {0.0f, 0.0f, 0.0f};

    // 直接把斜坡状态和目标都置零，避免缓慢回零造成“拖尾”
    Slope_Vx.Set_Now_Real(0.0f);
    Slope_Vy.Set_Now_Real(0.0f);
    Slope_Vw.Set_Now_Real(0.0f);
    Slope_Vx.Set_Target(0.0f);
    Slope_Vy.Set_Target(0.0f);
    Slope_Vw.Set_Target(0.0f);

    // 下发零速目标
    wheelChassis_MotionControl(stop);
}


