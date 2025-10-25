/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
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
#include "fdcan_bsp.h"
#include "dji_motor.h"
#include "cybergear_motor.h"
#include "Moter.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include <stdlib.h>
#include "trajectory_planner.h"
#include "task_rtos.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define buf_size 256
#define TASK_NONE     0   
#define TASK_1 1   
#define TASK_2 2   
#define TASK_3 3 
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t rx_buffer[buf_size]; 
volatile int16_t sys = TASK_NONE; 
Task3Position_t position[3];
#define POSITION_TOLERANCE  50       // DJI电机位置容差（编码器值）
#define CHASSIS_TOLERANCE   0.01f    // 底盘电机位置容差（rad）
#define WAIT_TIMEOUT_MS     5000     // 超时时间（ms）
#define SUCTION_DELAY_MS    500      // 吸盘动作延迟

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for Task */
osThreadId_t TaskHandle;
const osThreadAttr_t Task_attributes = {
  .name = "Task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
  .name = "myTask04",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for task3position */
osMessageQueueId_t task3positionHandle;
const osMessageQueueAttr_t task3position_attributes = {
  .name = "task3position"
};
/* Definitions for motorMutex */
osMutexId_t motorMutexHandle;
const osMutexAttr_t motorMutex_attributes = {
  .name = "motorMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Task0(void *argument);
void ZhuzhouMonitor(void *argument);
void DabiMonitor(void *argument);
void ChassisMonitor(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	fdcan_bsp_init();
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, buf_size);
	trajectory_planner_init();
    dji_motors_init();
	cybergear_motors_init();

	fdcan_bsp_start(&hfdcan1);
	fdcan_bsp_start(&hfdcan2);

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of motorMutex */
  motorMutexHandle = osMutexNew(&motorMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of task3position */
  task3positionHandle = osMessageQueueNew (16, sizeof(uint16_t), &task3position_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task */
  TaskHandle = osThreadNew(Task0, NULL, &Task_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(ZhuzhouMonitor, NULL, &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(DabiMonitor, NULL, &myTask03_attributes);

  /* creation of myTask04 */
  myTask04Handle = osThreadNew(ChassisMonitor, NULL, &myTask04_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Task0 */
/**
  * @brief  Function implementing the Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Task0 */
__weak void Task0(void *argument)
{
  /* USER CODE BEGIN Task0 */
	
  /* Infinite loop */
  for(;;)
  {
	  
    osDelay(1);
  }
  /* USER CODE END Task0 */
}

/* USER CODE BEGIN Header_ZhuzhouMonitor */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ZhuzhouMonitor */
__weak void ZhuzhouMonitor(void *argument)
{
  /* USER CODE BEGIN ZhuzhouMonitor */
  /* Infinite loop */
  for(;;)
  {
	  
    osDelay(1);
  }
  /* USER CODE END ZhuzhouMonitor */
}

/* USER CODE BEGIN Header_DabiMonitor */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DabiMonitor */
__weak void DabiMonitor(void *argument)
{
  /* USER CODE BEGIN DabiMonitor */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END DabiMonitor */
}

/* USER CODE BEGIN Header_ChassisMonitor */
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ChassisMonitor */
__weak void ChassisMonitor(void *argument)
{
  /* USER CODE BEGIN ChassisMonitor */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ChassisMonitor */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void  HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == &huart1)
  {
    rx_buffer[Size]='\0';
      printf("7");
      if(rx_buffer[0] == 'a')
      {
          printf("a");
      sys = TASK_1; 
      }
        else if(rx_buffer[0] == 'b')
        {
      sys = TASK_2; 
                      printf("b");

        }
        else if(rx_buffer[0] == 'c')
        {
      sys = TASK_3; 
                      printf("c");

        }
      else if(rx_buffer[0] == 'F') 
        {
                    float number[20];
                    sscanf((char*)rx_buffer,"F%fA%fS%fA%fL%fA%f",&number[0],&number[1],&number[2],&number[3],&number[4],&number[5]);
        position[0].radius=number[0];
        position[0].angle=number[1];
        position[1].radius=number[2];
        position[1].angle=number[3];
        position[2].radius=number[4];
        position[2].angle=number[5];
        send_task3_positions(position,3);
                              printf("f");

        }
      HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, buf_size);
  
}
    }
/* USER CODE END Application */

