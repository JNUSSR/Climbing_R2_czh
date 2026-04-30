#ifndef TEST_FEEDBACK_CLIMBINGTASK2_H
#define TEST_FEEDBACK_CLIMBINGTASK2_H

#include "drv_can.h"

#ifdef __cplusplus
#include "climbing_controller.h"
extern "C" {
#endif

void ClimbingTask(void);
// CAN 接收分发到 climbing 控制器
void Climbing_CAN_Rx_Dispatch(Struct_CAN_Rx_Buffer *Rx_Buffer);

// 兼容旧接口: 初始化 + 1ms 主控制
void Climbing_Task_Init(void);
void Climbing_Task_Entry(void);
void Climbing_Next_Step(void);

// 自动流程接口
void Climbing_Auto_Start(void);
void Climbing_Auto_Start_20cm(void);
void Climbing_Auto_Start_40cm(void);
void Climbing_Prepare_40cm(void);
void Climbing_Auto_Start_From_Touch_20cm(void);
void Climbing_Auto_Start_From_Touch_40cm(void);
void Climbing_Descend_Auto_Start(void);
void Climbing_Descend_Auto_Start_20cm(void);
void Climbing_Init_Pose_Start(void);
void Climbing_Auto_Task_1ms(void);
uint8_t Climbing_Is_Auto_Running(void);

// 手动调试接口
void Climbing_Manual_Next(void);
void Climbing_Descend_Manual_Next(void);
void Climbing_Manual_Reset(void);
#ifdef __cplusplus
ClimbingState_e Climbing_Get_State(void);
#endif

#ifdef __cplusplus
}

ClimbingController &Climbing_Get_Controller(void);

#endif

#endif // TEST_FEEDBACK_CLIMBINGTASK2_H
