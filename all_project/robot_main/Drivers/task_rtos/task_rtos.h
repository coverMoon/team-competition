#ifndef TASK_RTOS_H
#define TASK_RTOS_H

#include "main.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "Moter.h"
#include <math.h>


// 任务三位置消息结构
typedef struct {
    float radius;
    float angle;
} Task3Position_t;

// 外部声明的FreeRTOS对象（在freertos.c中定义）
extern osMessageQueueId_t task3positionHandle;
extern osMutexId_t motorMutexHandle;


// 任务函数声明
void task1_rtos();
void task2_rtos();
void task3_rtos();

// 公共接口函数
void send_task3_positions(Task3Position_t positions[], uint8_t count);
void stop_competition_task(osThreadId_t task_handle);

#endif /* TASKS_RTOS_H */







