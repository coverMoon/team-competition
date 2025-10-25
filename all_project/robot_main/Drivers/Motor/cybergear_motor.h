/**
  ******************************************************************************
  * @file           : cybergear_motor.h
  * @brief          : 小米 CyberGear 电机控制驱动接口文件
  ******************************************************************************
  * @attention
  *
  * 1. 本模块封装了对小米CyberGear系列微电机的CAN总线控制。
  * 2. 支持多种控制模式，包括位置、速度、力矩以及高效的运控模式。
  * 3. 提供了主动轮询电机特定参数（如角度、电压）的功能，以支持高频状态监控。
  * 4. 使用前必须确保 `fdcan_bsp` 模块已经正确移植并可用。
  * 5. 所有电机实例存储在一个静态全局数组中，通过索引进行访问。
  *
  ******************************************************************************
  */

#ifndef CYBERGEAR_MOTOR_H
#define CYBERGEAR_MOTOR_H

#include "main.h"
#include <math.h>

// 定义项目中 CyberGear 电机的总数量
#define CYBERGEAR_MOTOR_COUNT 1 

// 定义主机ID，与fdcan_bsp注册时保持一致
#define MASTER_CAN_ID 0


// 根据说明书 4.1.3 电机反馈数据 定义
#define P_MIN (-4.0f * M_PI) // 角度下限 -4π rad
#define P_MAX (4.0f * M_PI)  // 角度上限 4π rad
#define V_MIN (-30.0f)       // 角速度下限 -30 rad/s
#define V_MAX (30.0f)        // 角速度上限 30 rad/s
#define T_MIN (-12.0f)       // 力矩下限 -12 N.m
#define T_MAX (12.0f)        // 力矩上限 12 N.m

// 根据说明书和原例程 CanCommand.h 添加
#define KP_MIN (0.0f)
#define KP_MAX (500.0f)
#define KD_MIN (0.0f)
#define KD_MAX (5.0f)

/**
 * @brief 电机运行模式反馈枚举 (来自CAN报文)
 */
typedef enum {
    MOTOR_RESET_MODE = 0, // Reset 模式 [复位] 
    MOTOR_CALI_MODE  = 1, // Cali 模式 [标定] 
    MOTOR_RUN_MODE   = 2, // Motor 模式 [运行] 
} CyberGear_Motor_Mode_e;

/**
 * @brief 电机控制模式枚举
 */
typedef enum {
	  MOTOR_CONTROL_MODE_MOTION = 0,    // 运控模式
    MOTOR_CONTROL_MODE_POSITION,  		// 位置模式
    MOTOR_CONTROL_MODE_SPEED,     		// 速度模式
    MOTOR_CONTROL_MODE_TORQUE,    		// 力矩(电流)模式
		MOTOR_CONTROL_MODE_UNSET, 				// 未设定或未知模式
} CyberGear_Control_Mode_e;

/**
 * @brief 可供主动请求读取的电机参数地址枚举
 * @details 用于 cybergear_motor_request_parameter() 函数
 */
typedef enum {
    PARAM_MECH_POS   = 0x7019, // 负载端计圈机械角度 (rad), float
    PARAM_IQ_FILTER  = 0x701A, // iq 滤波值 (A), float
    PARAM_MECH_VEL   = 0x701B, // 负载端转速 (rad/s), float
    PARAM_VBUS       = 0x701C, // 母线电压 (V), float
    PARAM_ROTATION   = 0x701D, // 圈数, int16_t
} CyberGear_Param_Index_e;

/**
 * @brief 电机测量值与状态结构体 (来自电调反馈)
 */
typedef struct {
    float angle;         // 负载端计圈机械角度, 单位 rad
    float speed;         // 负载端转速, 单位 rad/s
    float torque;        // 反馈力矩 (N.m) - 注意: 这个只能由类型2报文更新
    float temperature;   // 电机温度 (°C) - 注意: 这个也只能由类型2报文更新

    // 这些值可以由类型17的报文更新
    float vbus;          // 母线电压 (V)
    int16_t rotation;    // 累计圈数

    CyberGear_Motor_Mode_e mode; // 当前电机所处的模式 
    uint8_t fault;               // 故障信息 
} motor_measure_cybergear_t;

/**
 * @brief CyberGear 电机实例的总结构体
 */
typedef struct CyberGear_Motor_Instance_t {
    motor_measure_cybergear_t measure;      // 电机的测量值和状态
    // 当前控制模式
    // 注意该参数仅在主控侧维护，用于跟踪当前的控制模式，不会由电机反馈更新
    CyberGear_Control_Mode_e control_mode;  

    // --- 以下是与 bsp 框架集成所需的信息 ---
    uint8_t  can_id;                // 该电机的反馈ID (e.g., 1)
    uint8_t  is_online;             // 在线状态标志
    uint32_t last_msg_time;         // 上次收到报文的HAL_GetTick()时间戳
    FDCAN_HandleTypeDef* hfdcan;    // 该电机绑定的FDCAN句柄

} CyberGear_Motor_Instance;


/*************************** Public Functions ***************************/

float cybergear_motor_rpm2rad(uint16_t rpm);
float cybergear_motor_degree2rad(float degree);
void cybergear_motors_init(void);
CyberGear_Motor_Instance* cybergear_motor_get_instance(uint8_t motor_index);
void cybergear_motor_enable(CyberGear_Motor_Instance* motor);
void cybergear_motor_stop(CyberGear_Motor_Instance* motor);
void cybergear_motor_set_mode(CyberGear_Motor_Instance* motor, CyberGear_Control_Mode_e mode);
void cybergear_motor_set_zero_position(CyberGear_Motor_Instance* motor);
void cybergear_motor_set_position(CyberGear_Motor_Instance* motor, float position, float speed_limit);
void cybergear_motor_set_speed(CyberGear_Motor_Instance* motor, float speed);
void cybergear_motor_set_torque(CyberGear_Motor_Instance* motor, float torque);
void cybergear_motor_set_motion_control(CyberGear_Motor_Instance* motor, float position, float speed, float kp, float kd, float torque_ff);
void cybergear_motor_set_pid_gains(CyberGear_Motor_Instance* motor, float loc_kp, float spd_kp, float spd_ki);
void cybergear_motor_set_current_limit(CyberGear_Motor_Instance* motor, float current_limit);
void cybergear_motor_request_parameter(CyberGear_Motor_Instance* motor, CyberGear_Param_Index_e param_index);

/*************************** Internal Functions (for fdcan_bsp) ***************************/
void cybergear_message_handler(void* instance, FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[8]);

#endif // CYBERGEAR_MOTOR_H
