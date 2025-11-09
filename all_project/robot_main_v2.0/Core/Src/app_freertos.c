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
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include <stdlib.h>
#include "trajectory_planner.h"
#include "task_rtos.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define buf_size 			256
#define TASK_NONE     0   
#define TASK_1 				1   
#define TASK_2 				2   
#define TASK_3 				3 

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t rx_buffer[buf_size]; 
volatile int16_t sys = TASK_NONE; 
TestPosition_t test = { 620.0f, 145.0f, 0.0f, 0.0f, 0 };
/* USER CODE END Variables */
/* Definitions for Task */
osThreadId_t TaskHandle;
const osThreadAttr_t Task_attributes = {
  .name = "Task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
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

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	fdcan_bsp_init();
	HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buffer, buf_size);
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
  task3positionHandle = osMessageQueueNew (16, sizeof(Task3Position_t), &task3position_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task */
  TaskHandle = osThreadNew(Task0, NULL, &Task_attributes);

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

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void  HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
		// 确保是来自 huart1 的中断
    if (huart == &huart5)
    {
        // 1. 将数据包末尾添加 null 终止符，以便 sscanf 等字符串函数安全解析
        rx_buffer[Size] = '\0';

        // 2. 解析指令
        // 指令 'a': 启动任务一
        if (rx_buffer[0] == 'a')
        {
            sys = TASK_1;
        }
        // 指令 'b': 启动任务二
        else if (rx_buffer[0] == 'b')
        {
            sys = TASK_2;
        }
        // 指令 'c': 启动任务三
        else if (rx_buffer[0] == 'c')
        {
            sys = TASK_3;
        }
        // 指令 'F': 任务三随机坐标 (格式: F<r1>A<a1>S<r2>A<a2>L<r3>A<a3>)
        else if (rx_buffer[0] == 'F')
        {
            // 将变量定义在最小作用域 (回调函数内部)
            int32_t parsed_numbers[6]; // 存储解析出的6个整数
            Task3Position_t local_positions[3]; // 存储转换后的3个坐标

            // 尝试解析字符串
            // 格式: F(radius1)A(angle1)S(radius2)A(angle2)L(radius3)A(angle3)
            int parse_count = sscanf((char*)rx_buffer, "F%dA%dS%dA%dL%dA%d",
                                     &parsed_numbers[0], &parsed_numbers[1],
                                     &parsed_numbers[2], &parsed_numbers[3],
                                     &parsed_numbers[4], &parsed_numbers[5]);

            // 确保成功解析了所有 6 个数字
            if (parse_count == 6)
            {
                // 将解析的整数转换为浮点数并填充结构体
                local_positions[0].radius = (float)parsed_numbers[0];
                local_positions[0].angle  = (float)parsed_numbers[1];
                local_positions[1].radius = (float)parsed_numbers[2];
                local_positions[1].angle  = (float)parsed_numbers[3];
                local_positions[2].radius = (float)parsed_numbers[4];
                local_positions[2].angle  = (float)parsed_numbers[5];

                // 将解析到的坐标发送到消息队列 (在中断中，使用 0 超时)
                send_task3_positions(local_positions, 3);
            }
            else
            {
                // 可选：添加解析失败的错误处理日志
                // printf("UART Parse Error for Task 3 data\r\n");
            }
        }
				// 指令 'T': 仅调试时使用，输入四个坐标传入test结构体
				// 格式: T H<height> R<radius> C<chassis_angle> S<suction_angle> X<suction_switch>
				if	(rx_buffer[0] == 'T')
				{
						int parsed_numbers[5]; // 存储解析出的5个整数
						// 尝试解析字符串
            // 格式: T H(height) R(radius) C(chassis_angle) S(suction_angle) X(suction_switch)
            int parse_count = sscanf((char*)rx_buffer, "T H%d R%d C%d S%d X%d",
                                     &parsed_numbers[0], &parsed_numbers[1],
                                     &parsed_numbers[2], &parsed_numbers[3],
																		 &parsed_numbers[4]);
						
						// 传入test结构体
						test.height = (float)parsed_numbers[0];					// 主轴高度
						test.radius = (float)parsed_numbers[1];					// 径向距离
						test.chassis_angle = (float)parsed_numbers[2];	// 底盘角度
						test.suction_angle = (float)parsed_numbers[3];	// 吸盘角度
						test.suction_switch = parsed_numbers[4];				// 吸盘开关
				}
        
        // 3. 重新启动 UART DMA 接收，准备接收下一条指令
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buffer, buf_size);
    }
}
/* USER CODE END Application */

