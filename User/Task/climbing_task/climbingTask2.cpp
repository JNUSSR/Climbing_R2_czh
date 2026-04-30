#include "climbingTask2.h"

#include "cmsis_os2.h"
#include "main.h"

ClimbingController climbingCtrl;

ClimbingController &Climbing_Get_Controller(void)
{
    return climbingCtrl;
}

void Climbing_CAN_Rx_Dispatch(Struct_CAN_Rx_Buffer *Rx_Buffer)
{
    climbingCtrl.CAN_RxCallback(Rx_Buffer->Header.StdId, Rx_Buffer->Data);
}

void ClimbingTask(void)
{
    // 与 docking 一致: 任务入口里完成一次初始化
    climbingCtrl.Init(&hcan1);
    for (;;)
    {
        // 1ms 周期: 先推进自动状态, 再执行主控制
        climbingCtrl.AutoTask1ms();
        climbingCtrl.TaskEntry1ms();
        osDelay(1);
    }
}

void Climbing_Task_Init(void)
{
    // 兼容旧调用方式(保留给历史代码)
    climbingCtrl.Init(&hcan1);
}

void Climbing_Task_Entry(void)
{
    // 兼容旧调用方式(保留给历史代码)
    climbingCtrl.TaskEntry1ms();
}

void Climbing_Next_Step(void)
{
    climbingCtrl.NextStep();
}

void Climbing_Auto_Start(void)
{
    climbingCtrl.AutoStart40cm();
}

void Climbing_Auto_Start_20cm(void)
{
    climbingCtrl.AutoStart20cm();
}

void Climbing_Auto_Start_40cm(void)
{
    climbingCtrl.AutoStart40cm();
}

void Climbing_Descend_Auto_Start(void)
{
    climbingCtrl.DescendAutoStart20cm();
}

void Climbing_Descend_Auto_Start_20cm(void)
{
    climbingCtrl.DescendAutoStart20cm();
}

void Climbing_Init_Pose_Start(void)
{
    climbingCtrl.InitPoseStart();
}

void Climbing_Auto_Task_1ms(void)
{
    climbingCtrl.AutoTask1ms();
}

uint8_t Climbing_Is_Auto_Running(void)
{
    return climbingCtrl.IsAutoRunning();
}

void Climbing_Manual_Next(void)
{
    climbingCtrl.ManualNext();
}

void Climbing_Descend_Manual_Next(void)
{
    climbingCtrl.DescendManualNext();
}

void Climbing_Manual_Reset(void)
{
    climbingCtrl.ManualReset();
}

void Climbing_Manual_Goto(ClimbingState_e state)
{
    climbingCtrl.ManualGoto(state);
}

void Climbing_Emergency_Stop(void)
{
    climbingCtrl.EmergencyStop();
}

void Climbing_Set_Chassis_External_Control(uint8_t enable)
{
    climbingCtrl.SetChassisExternalControl(enable);
}

ClimbingState_e Climbing_Get_State(void)
{
    return climbingCtrl.GetState();
}
