/**
 * @file host_communication_task.h
 * @brief 上位机通信任务 - 适配 drv_uart 版本
 */

#ifndef __HOST_COMMUNICATION_TASK_H
#define __HOST_COMMUNICATION_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"      // 引入 huart6 定义
#include "cmsis_os.h"  // FreeRTOS 相关
/* Exported variables --------------------------------------------------------*/

/**
 * @brief 电机目标速度全局变量 (rad/s)
 * @note 由 VOFA+ 指令修改，电机任务读取
 */
extern float g_motor_3508_target_speed;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化通信任务 (注册串口回调)
 */
void Host_Communication_Task_Init(void);

/**
 * @brief 通信任务主循环
 */
void Host_Communication_Task(void *argument);

/**
 * @brief 发送波形数据到VOFA+
 */
void Host_Send_Waveform(void);

#ifdef __cplusplus
}
#endif

#endif /* __HOST_COMMUNICATION_TASK_H */
