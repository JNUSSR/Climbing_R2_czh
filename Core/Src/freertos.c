/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "climbingTask2.h"
#include "host_communication_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for TaskSerial */
osThreadId_t TaskSerialHandle;
const osThreadAttr_t TaskSerial_attributes = {
  .name = "TaskSerial",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TaskClimbing */
osThreadId_t TaskClimbingHandle;
const osThreadAttr_t TaskClimbing_attributes = {
  .name = "TaskClimbing",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern void J4310_Task(void *argument);
extern void DJI_M3508_Task(void *argument);
/* USER CODE END FunctionPrototypes */

void StartTaskSerial(void *argument);
void StartTaskClimbing(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of TaskSerial */
  TaskSerialHandle = osThreadNew(StartTaskSerial, NULL, &TaskSerial_attributes);

  /* creation of TaskClimbing */
  TaskClimbingHandle = osThreadNew(StartTaskClimbing, NULL, &TaskClimbing_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartTaskSerial */
/**
  * @brief  Function implementing the TaskSerial thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTaskSerial */
void StartTaskSerial(void *argument)
{
  /* USER CODE BEGIN StartTaskSerial */
  /* Infinite loop */
  Host_Communication_Task(argument);
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskSerial */
}

/* USER CODE BEGIN Header_StartTaskClimbing */
/**
* @brief Function implementing the TaskClimbing thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskClimbing */
void StartTaskClimbing(void *argument)
{
  /* USER CODE BEGIN StartTaskClimbing */
  // 攀爬任务主循环已封装到 ClimbingTask() 内部
  ClimbingTask();
  /* USER CODE END StartTaskClimbing */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

