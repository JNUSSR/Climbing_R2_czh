#ifndef __WHEELCHASSIS_H
#define __WHEELCHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "can.h"

// #define PI 3.1415926
#define TIMESTEP 0.001f //时间步长，和定时中断频率有关
#define REMOVE_K 0.02 //底盘控制值和遥控通道值的比例关系

typedef struct
{
    float Chassis_R;
    float Wheel_R;
    float Acceleration;
    float Deceleration;
}Chassis_Params;

typedef struct
{
    float Vx;  // X方向速度 (m/s)
    float Vy;  // Y方向速度 (m/s)
    float Vw;  // 旋转角速度 (rad/s)
}MotionControl;

typedef struct {
    // 滤波器系数
    float b0, b1, b2;
    float a1, a2;

    // 状态变量
    float x1, x2;  // 前两次输入
    float y1, y2;  // 前两次输出
} SecondOrderFilter;


void wheelChassis_Init(void);
void wheelChassis_Task(MotionControl motion);
void wheelChassis_SetMotion(float vx, float vy, float vw);
// 立即停车：清零斜坡规划和4个底盘电机目标速度
void wheelChassis_StopImmediate(void);

void wheelChassis_MotionControl(MotionControl motion);
void wheelChassis_WithFilter_Task(MotionControl motion);

#ifdef __cplusplus
}
#endif

// C++ 专用头文件（仅在 C++ 模式下包含）
#ifdef __cplusplus
#include "drv_math.h"
#include "alg_pid.h"
#include "dvc_motor.h"
#endif

#endif
