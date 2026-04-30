#ifndef TASK_MECANUM_CHASSIS_H
#define TASK_MECANUM_CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

void Task_Mecanum_Chassis_Init(void);
void Task_Mecanum_Chassis_SetMotion(float vx, float vy, float vw);
void Task_Mecanum_Chassis_StopImmediate(void);
void Task_Mecanum_Chassis_Task1ms(void);

#ifdef __cplusplus
}
#endif

#endif
