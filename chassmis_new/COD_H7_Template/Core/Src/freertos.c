/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "SEGGER_SYSVIEW_FreeRTOS.h"

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
osThreadId StartINS_TaskHandle;
uint32_t StartINS_TaskBuffer[ 1024 ];
osStaticThreadDef_t StartINS_TaskControlBlock;
osThreadId StartCAN_TaskHandle;
uint32_t StartCAN_TaskBuffer[ 1024 ];
osStaticThreadDef_t StartCAN_TaskControlBlock;
osThreadId StartControl_TaskHandle;
uint32_t StartControlBuffer[ 1024 ];
osStaticThreadDef_t StartControlControlBlock;
osThreadId StartMPC_TaskHandle;
uint32_t StartMPC_TaskBuffer[ 1024 ];
osStaticThreadDef_t StartMPC_TaskControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void INS_Task(void const * argument);
void CAN_Task(void const * argument);
void Control_Task(void const * argument);
void MPC_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* definition and creation of StartINS_Task */
  osThreadStaticDef(StartINS_Task, INS_Task, osPriorityHigh, 0, 1024, StartINS_TaskBuffer, &StartINS_TaskControlBlock);
  StartINS_TaskHandle = osThreadCreate(osThread(StartINS_Task), NULL);

  /* definition and creation of StartCAN_Task */
  osThreadStaticDef(StartCAN_Task, CAN_Task, osPriorityNormal, 0, 1024, StartCAN_TaskBuffer, &StartCAN_TaskControlBlock);
  StartCAN_TaskHandle = osThreadCreate(osThread(StartCAN_Task), NULL);

  /* definition and creation of StartControl_Task */
  osThreadStaticDef(StartControl_Task, Control_Task, osPriorityAboveNormal, 0, 1024, StartControlBuffer, &StartControlControlBlock);
  StartControl_TaskHandle = osThreadCreate(osThread(StartControl_Task), NULL);

  /* definition and creation of StartMPC_Task */
  osThreadStaticDef(StartMPC_Task, MPC_Task, osPriorityBelowNormal, 0, 1024, StartMPC_TaskBuffer, &StartMPC_TaskControlBlock);
  StartMPC_TaskHandle = osThreadCreate(osThread(StartMPC_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_INS_Task */
/**
  * @brief  Function implementing the StartINS thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_INS_Task */
__weak void INS_Task(void const * argument)
{
  /* USER CODE BEGIN INS_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END INS_Task */
}

/* USER CODE BEGIN Header_CAN_Task */
/**
* @brief Function implementing the StartCAN thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Task */
__weak void CAN_Task(void const * argument)
{
  /* USER CODE BEGIN CAN_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END CAN_Task */
}

/* USER CODE BEGIN Header_Control_Task */
/**
* @brief Function implementing the StartControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Control_Task */
__weak void Control_Task(void const * argument)
{
  /* USER CODE BEGIN Control_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Control_Task */
}

/* USER CODE BEGIN Header_MPC_Task */
/**
* @brief Function implementing the StartMPC_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MPC_Task */
__weak void MPC_Task(void const * argument)
{
  /* USER CODE BEGIN MPC_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END MPC_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
