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
#include "cybergear_motor.h"
#include "trajectory_planner.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern UART_HandleTypeDef huart1;

static CyberGear_Motor_Instance *mi_motor = NULL;
uint8_t uart_rx_buffer[BUFFER_SIZE];

volatile float set_pos = 0;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_buffer, BUFFER_SIZE);
	trajectory_planner_init();
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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
	
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	mi_motor = cybergear_motor_get_instance(0);
	
	vTaskDelay(pdMS_TO_TICKS(10));
	cybergear_motor_stop(mi_motor);
	vTaskDelay(pdMS_TO_TICKS(10));
	cybergear_motor_set_mode(mi_motor, MOTOR_CONTROL_MODE_POSITION);
	vTaskDelay(pdMS_TO_TICKS(10));
	//cybergear_motor_set_current_limit(mi_motor, 15.0f);
	vTaskDelay(pdMS_TO_TICKS(10));
	cybergear_motor_enable(mi_motor);
	//vTaskDelay(pdMS_TO_TICKS(10));
	//cybergear_motor_set_zero_position(mi_motor);
	//vTaskDelay(pdMS_TO_TICKS(10));
  //cybergear_motor_set_speed(mi_motor, 6.28);
	//vTaskDelay(pdMS_TO_TICKS(10));
	trajectory_planner_start(mi_motor);
  /* Infinite loop */
  for(;;)
  {
		trajectory_planner_set_target(cybergear_motor_degree2rad(set_pos));
    
    printf("current pos: %.2f		|		current speed: %.2f rad/s\r\n", mi_motor->measure.angle / M_PI * 180.0, mi_motor->measure.speed);
		vTaskDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/**
  * @brief  Reception Event Callback (Rx event notification called after use of advanced reception service).
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance == USART1)
  {
    if(uart_rx_buffer[0] == 's')
    {
      if(set_pos == 0.0f)
      {
        set_pos = 180.0f;
      }
      else
      {
        set_pos = 0.0f;
      }
    }

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_buffer, BUFFER_SIZE);
  }
}
/* USER CODE END Application */

