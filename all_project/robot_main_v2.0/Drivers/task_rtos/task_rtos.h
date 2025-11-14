#ifndef TASK_RTOS_H
#define TASK_RTOS_H

#include "main.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"

/* ------------------------- 参数测试结构体 ------------------------- */
typedef struct{
		float height;							// 主轴高度 (mm)
		float radius;							// 径向距离 (mm)
		float chassis_angle;			// 底盘角度 (度)
		float suction_angle;			// 吸盘角度 (度)
		uint8_t suction_switch;		// 吸盘开关 (1-开 0-关)
} TestPosition_t;

/* ------------------------- 通用坐标结构体 ------------------------- */

/**
 * @brief 极坐标定义 (用于任务一、二)
 */
typedef struct {
    float radius; // 径向距离 (mm)
    float angle;  // 角度 (度)
} PolarCoord_t;

/**
 * @brief 任务三位置消息结构 (用于FreeRTOS消息队列)
 */
typedef struct {
    float radius;
    float angle;
} Task3Position_t;


/* ------------------------- 比赛规则常量定义 ------------------------- */
//限位
#define MAX_HEIGHT 620.0f // 最大高度
#define MIN_HEIGHT 130.0f // 最小高度
#define MAX_RADIUS 450.0f // 最大长度
#define MIN_RADIUS 145.0f // 最小长度
#define MAX_ANGLE 110.0f // 最大角度
#define MIN_ANGLE -90.0f // 最小角度
// 纸箱尺寸
#define SMALL_BOX_HEIGHT 150.0f // 小纸箱高度 (mm) 
#define BIG_BOX_HEIGHT   180.0f // 大纸箱高度 (mm) 

// 任务一：箱子堆叠
static const PolarCoord_t task1_boxes[3] = {{450.0f, -60.0f},  // (450mm, +60°) 
                                            {350.0f, -45.0f}, // (350mm, +45°) 
                                            {400.0f, 30.0f}}; // (400mm, -30°) 
static const PolarCoord_t TASK1_TARGET_POS = {350.0f, 90.0f}; // (350mm, -90°) 

// 任务二：定点放置
static const PolarCoord_t TASK2_PICKUP_POS = {350.0f, -180.0f}; // 小纸箱叠放位置

//// 任务二 目标区域 (为简化，这里只列出坐标，高度和类型在 .c 文件中定义)
//// 一区
//static const PolarCoord_t TASK2_ZONE1_POS_1 = {350.0f, 0.0f};
//static const PolarCoord_t TASK2_ZONE1_POS_2 = {350.0f, 45.0f};
//static const PolarCoord_t TASK2_ZONE1_POS_3 = {350.0f, -45.0f};
//// 二区
//static const PolarCoord_t TASK2_ZONE2_POS_1 = {600.0f, 30.0f};
//static const PolarCoord_t TASK2_ZONE2_POS_2 = {600.0f, -30.0f};
//// 三区
//static const PolarCoord_t TASK2_ZONE3_POS_1 = {550.0f, 0.0f};
//static const PolarCoord_t TASK2_ZONE3_POS_2 = {550.0f, 60.0f};
//static const PolarCoord_t TASK2_ZONE3_POS_3 = {550.0f, -60.0f};


/* ------------------------- FreeRTOS 对象外部声明 ------------------------- */
// (这些对象在 app_freertos.c 中定义)
extern osMessageQueueId_t task3positionHandle;

/* ------------------------- 公共接口函数声明 ------------------------- */

/**
 * @brief 主调度任务 (在 app_freertos.c 中被创建为线程)
 */
void Task0(void *argument);

/**
 * @brief (供UART中断调用) 发送任务三的随机位置到消息队列
 * @param positions 包含3个位置的数组
 * @param count 数组中有效位置的个数 (应为 3)
 */
void send_task3_positions(Task3Position_t positions[], uint8_t count);


#endif /* TASK_RTOS_H */