/**
  ******************************************************************************
  * @file           : trajectory_planner.c
  * @brief          : 底盘旋转平滑轨迹规划器 实现文件
  * @author         : 秦泽宇 & Gemini
  * @date           : 2025-10-23
  * @version        : v1.0
  * @note           :
  
  ******************************************************************************
  * @attention
  *
  * [Module Design]
  * 1. 本模块旨在提供一个上层的、平滑的运动控制器。
  * 2. 核心机制是一个高优先级的FreeRTOS周期任务(`prv_planner_task`) 。
  * 3. 该任务充当一个“轨迹生成器”，生成一个梯形速度曲线（2阶规划器）。
  * 	 通过规划加速度，确保在启停阶段速度平滑变化，从而消除顿挫感。
  * 4. 本模块不直接控制硬件，完全依赖于 `cybergear_motor` 驱动模块。
  * 5. 通过高频调用 `cybergear_motor_set_position()` 并传入平滑变化的
  *    中间位置，来“引领”电机内部的位置PID控制器。
  * 6. [核心优势]: 利用电机内置的位置环PID（特别是其Ki积分器）
  * 	 来自动处理所有复杂的、时变的负载（如机械臂伸展导致的变惯量、摩擦力），
  * 	 同时保证零静态误差。主控(STM32)只负责生成轨迹，不负责动力学补偿。
  *
  * [Usage Flow]
  * 1. 在 `main()` 中，`vTaskStartScheduler()` 之前，调用 `trajectory_planner_init()`。
  * 	 这将创建互斥锁和**挂起**的控制任务。
  * 2. 在系统的初始化任务(Init_Task)中，在 `cybergear_motor` 实例被完全初始化、配置为
			 位置模式、并使能后，调用 `trajectory_planner_start(&motor)`。
  * 3. `trajectory_planner_start()` 会同步规划器的初始位置为电机的当前实际位置，并
			 恢复（启动）内部的控制任务。
  * 4. 在任何其他逻辑任务中，当需要底盘旋转时，只需调用 
  * 	 `trajectory_planner_set_target(float new_pos)`。
  * 5. 规划器任务将自动在后台生成平滑的梯形速度曲线，驱动电机平滑、准确地
  * 	 到达新位置。
  *
  * [FreeRTOS 线程安全支持]
  * 1. 本模块的核心是一个高优先级的周期性任务 (`prv_planner_task`) ，使用 
  * 	 `vTaskDelayUntil()` 来保证精确的控制周期 。
  * 2. 对外接口 `trajectory_planner_set_target()` 是线程安全的。内部
  * 	 使用了一个互斥锁 (`g_chassis_target_mutex`) 来保护最终目标位置
  * 	 `g_final_target_pos` ，防止多任务并发写入。
  * 3. 本模块会高频调用 `cybergear_motor_set_position()`，该函数自身
  * 	 是线程安全的（其内部调用了`cybergear_lock_tx()`）。
  *
  ******************************************************************************
  */

#include "trajectory_planner.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "cybergear_motor.h"

/* ------------------------- 私有宏定义 (Private Defines) ------------------------- */

// --- 任务配置 ---

// 控制周期 (10ms -> 100Hz)
#define PLANNER_CONTROL_PERIOD_MS   (10)
// 任务栈大小 (256 words * 4 bytes/word = 1024 bytes)
#define PLANNER_TASK_STACK_SIZE     (256) 
// 任务优先级 (高优先级，确保实时性)
#define PLANNER_TASK_PRIORITY       (tskIDLE_PRIORITY + 3)

// --- 规划器参数 ---
// 底盘最大平滑速度 (rad/s)         - 【调参】
#define PLANNER_MAX_VELOCITY        (8.00f)
// 底盘最大平滑加速度 (rad/s^2)     - 【调参】
#define PLANNER_MAX_ACCELERATION    (0.9f) 

// --- 电机控制参数 ---
// 电机内部的速度限制 (rad/s) - 必须大于 PLANNER_MAX_VELOCITY
#define MOTOR_INTERNAL_SPD_LIMIT    (15.0f) 



/* ----------------------- 模块私有变量 (Static Variables) ----------------------- */



// 任务句柄
static TaskHandle_t xPlannerTaskHandle = NULL;

// 互斥锁：用于保护 g_final_target_pos
static SemaphoreHandle_t g_chassis_target_mutex = NULL;

// 目标位置：由外部任务通过 set_target() 写入
static float g_final_target_pos = 0.0f;

// 电机实例指针：指向一个已使能的电机
static CyberGear_Motor_Instance* g_motor = NULL;

// --- 规划器状态变量 ---
static float g_current_ramped_pos = 0.0f;   // 规划器输出的【位置】
static float g_current_ramped_vel = 0.0f;   // 规划器内部的【速度】



/* --------------------- 私有任务函数 (Static Task Function) --------------------- */



/**
 * @brief   轨迹规划器私有任务
 * @note    此任务在创建时被挂起，直到 trajectory_planner_start() 被调用才开始运行。
 *          它假定 g_motor 实例已有效，且 g_current_ramped_pos 已被同步。
 */
static void prv_planner_task(void *pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(PLANNER_CONTROL_PERIOD_MS);
    
    // 计算每个周期的固定增量
    const float dt = (float)PLANNER_CONTROL_PERIOD_MS / 1000.0f; // 周期 (s)
    const float accel_step = PLANNER_MAX_ACCELERATION * dt;      // 每周期最大速度变化量
    const float max_vel = PLANNER_MAX_VELOCITY;

    xLastWakeTime = xTaskGetTickCount();

    // --- 任务主循环 ---
    for(;;)
    {
        // A. 精确周期等待
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // B. 线程安全地获取最终目标
        float local_target;
        xSemaphoreTake(g_chassis_target_mutex, portMAX_DELAY);
        local_target = g_final_target_pos;
        xSemaphoreGive(g_chassis_target_mutex);
        
        // C. 计算位置误差
        float pos_error = local_target - g_current_ramped_pos;
        
        // D. 核心逻辑计算刹车距离
        //    d = v^2 / (2*a)
        //    我们需要知道速度朝向哪个方向
        float vel_sign = (g_current_ramped_vel > 0) ? 1.0f : -1.0f;
        float stop_dist = (g_current_ramped_vel * g_current_ramped_vel) / (2.0f * PLANNER_MAX_ACCELERATION);
        
        // E. 【核心逻辑】状态机：加速 - 减速
        
        // 1. 判断是否应该开始减速
        //    如果 (位置误差的绝对值) <= (刹车距离)
        //    并且 (误差方向 与 速度方向 一致) -> 意味着我们正冲向目标
        if ((fabsf(pos_error) <= stop_dist) && (pos_error * g_current_ramped_vel >= 0.0f) )
        {
            // --- 状态：减速 ---
            // 速度向 0 变化
            g_current_ramped_vel -= vel_sign * accel_step;
            
            // 防止速度过冲到0以下
            if (g_current_ramped_vel * vel_sign < 0.0f) 
            {
                g_current_ramped_vel = 0.0f;
            }
        }
        else
        {
            // --- 状态：加速 或 巡航 ---
            // 速度向 V_max (或 -V_max) 变化
            float target_vel_sign = (pos_error > 0) ? 1.0f : -1.0f;
            g_current_ramped_vel += target_vel_sign * accel_step;
        }

        // F. 限制最大速度 (巡航)
        if (g_current_ramped_vel > max_vel) 
        {
            g_current_ramped_vel = max_vel;
        } 
        else if (g_current_ramped_vel < -max_vel) 
        {
            g_current_ramped_vel = -max_vel;
        }

        // G. 积分：计算新的位置
        //    (只有在速度不为0时才更新位置，防止目标漂移)
        if (g_current_ramped_vel != 0.0f || fabsf(pos_error) > 1e-4f)
        {
             g_current_ramped_pos += g_current_ramped_vel * dt;
        }

        // H. 发送指令
        cybergear_motor_set_position(g_motor, g_current_ramped_pos, MOTOR_INTERNAL_SPD_LIMIT);
    }
}



/* ----------------------- 公共接口函数 (Public Functions) ----------------------- */



/**
 * @brief   初始化轨迹规划器模块
 * @details 该函数会创建一个轨迹规划器任务并设置为挂起状态，在调用
 *          start() 函数后会唤醒任务
 */
int8_t trajectory_planner_init(void)
{
    // 1. 创建互斥锁
    g_chassis_target_mutex = xSemaphoreCreateMutex();
    if (g_chassis_target_mutex == NULL) 
    {
        return -1; // 互斥锁创建失败
    }

    // 2. 创建任务
    BaseType_t xReturned = xTaskCreate(
                                    prv_planner_task,           // 任务函数
                                    "PlannerTask",              // 任务名
                                    PLANNER_TASK_STACK_SIZE,    // 栈大小
                                    NULL,                       // 任务参数
                                    PLANNER_TASK_PRIORITY,      // 任务优先级
                                    &xPlannerTaskHandle         // 获取任务句柄
                                    );

    if (xReturned != pdPASS) 
    {
        return -1; // 任务创建失败
    }

    // 3. 立刻挂起任务，等待 start() 函数来唤醒
    if (xPlannerTaskHandle != NULL) 
    {
        vTaskSuspend(xPlannerTaskHandle);
    } 
    else 
    {
        return -1; // 句柄无效
    }
    
    return 0; // 初始化成功
}

/**
 * @brief   启动轨迹规划器任务
 * @details 该函数会唤醒挂起的轨迹规划任务，任务从此不再挂起
 * @param   motor: 控制电机实例
 */
int8_t trajectory_planner_start(CyberGear_Motor_Instance* motor)
{
    if (motor == NULL) 
    {
        return -1; // 传入了无效的电机指针
    }
    
    if (xPlannerTaskHandle == NULL || g_chassis_target_mutex == NULL) 
    {
        return -1; // 模块未正确初始化 (init未调用或失败)
    }

    // 1. 存储电机实例
    g_motor = motor;

    // 2. 同步初始位置
    //    调用者必须保证此时 motor->measure.angle 是有效的
    g_current_ramped_pos = g_motor->measure.angle;

    // 3. 线程安全地初始化目标值为当前值
    xSemaphoreTake(g_chassis_target_mutex, portMAX_DELAY);
    g_final_target_pos = g_current_ramped_pos;
    xSemaphoreGive(g_chassis_target_mutex);

    // 4. 恢复(启动)挂起的控制任务
    vTaskResume(xPlannerTaskHandle);

    return 0;
}


/**
 * @brief   设置底盘的最终目标位置 (线程安全)
 * @details 该函数会通过使用互斥锁保证最终目标位置是临界资源，不会发生并发冲突
 */
void trajectory_planner_set_target(float target_position)
{
    if (g_chassis_target_mutex != NULL)
    {
        // 等待获取互斥锁 (无限等待)
        xSemaphoreTake(g_chassis_target_mutex, portMAX_DELAY);
        
        // 安全地更新目标值
        g_final_target_pos = target_position;
        
        // 释放互斥锁
        xSemaphoreGive(g_chassis_target_mutex);
    }
}