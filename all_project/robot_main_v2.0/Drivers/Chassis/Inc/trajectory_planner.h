/**
  ******************************************************************************
  * @file           : trajectory_planner.h
  * @brief          : 底盘旋转平滑轨迹规划器 接口文件
  ******************************************************************************
  */

#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include "main.h"
#include "cybergear_motor.h"

/**
 * @brief  初始化轨迹规划器模块
 * @details
 *      - 创建一个互斥锁，用于保护目标位置变量。
 *      - 创建一个高优先级的周期性任务(prv_planner_task)，并使其处于【挂起状态】。
 * @note   必须在 FreeRTOS 调度器启动(vTaskStartScheduler)之前调用。
 * @param  None
 * @retval 0: 成功, -1: 失败
 */
int8_t trajectory_planner_init(void);

/**
 * @brief  启动轨迹规划器任务
 * @details
 *      - 必须在电机实例初始化、使能、并传回有效数据后调用。
 *      - 此函数会将规划器的初始位置同步为电机的当前实际位置。
 *      - 调用后，规划器任务(prv_planner_task)将解除挂起，开始运行。
 * @param  motor: 指向一个已经使能的电机实例的指针。
 * @retval 0: 成功, -1: 失败 (e.g., 传入了NULL)
 */
int8_t trajectory_planner_start(CyberGear_Motor_Instance* motor);

/**
 * @brief  设置底盘的最终目标位置 (线程安全)
 * @details
 *      - 其他任务通过调用此函数来命令底盘旋转。
 *      - 函数内部使用互斥锁保护目标值，可被任意任务安全调用。
 * @param  target_position: 最终的目标角度 (rad)
 * @retval None
 */
void trajectory_planner_set_target(float target_position);

#endif // TRAJECTORY_PLANNER_H