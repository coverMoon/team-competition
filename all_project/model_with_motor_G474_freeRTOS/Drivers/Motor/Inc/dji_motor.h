/**
  ******************************************************************************
  * @file           : dji_motor.h
  * @brief          : DJI电机控制驱动接口文件
  ******************************************************************************
  */

#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "main.h"
#include "pid.h"

#define DJI_MOTOR_COUNT 8	// 定义系统中DJI电机的总数量

/**
 * @brief 电机 ID 枚举
 */
typedef enum{
    CAN_3508_2006_M1_ID = 0x201,
    CAN_3508_2006_M2_ID = 0x202,
    CAN_3508_2006_M3_ID = 0x203,
    CAN_3508_2006_M4_ID = 0x204,
    CAN_3508_2006_M5_ID = 0x205,
    CAN_3508_2006_M6_ID = 0x206,
    CAN_3508_2006_M7_ID = 0x207,
    CAN_3508_2006_M8_ID = 0x208,
} can_msg_id_e;

/**
 * @brief 电机控制模式枚举
 */
typedef enum {
    MOTOR_MODE_SPEED = 0,    // 速度闭环模式 
    MOTOR_MODE_POSITION = 1  // 位置闭环模式 (串级PID) 
} DJI_Motor_Mode_e;

/**
 * @brief 电机测量值结构体 (来自电调反馈)
 * @note  保持与原版驱动一致
 */
typedef struct {
    uint16_t angle;          // 机械角度, 0-8191
    int16_t  speed_rpm;      // 转速 (RPM) 
    int16_t  given_current;  // 实际转矩电流, -16384 to 16384 
    uint8_t  temperate;      // 电机温度 (°C) 
    int16_t  last_angle;     // 上一次的机械角度，用于计算累计角度 
    int32_t  total_angle;    // 累计总角度 (支持多圈)
} motor_measure_t;

/**
 * @brief DJI电机实例的总结构体
 * @details 封装了单个电机的所有信息，包括测量值、目标值、PID控制器和状态
 */
typedef struct DJI_Motor_Instance_t {
    motor_measure_t measure;            // 电机的测量值 

    int32_t target_loc;                 // 目标位置 (编码器值) 
    int16_t target_speed;               // 目标速度 (RPM) 
    DJI_Motor_Mode_e control_mode;      // 当前的控制模式 

    pid_incremental_struct loc_pid;     // 位置环PID控制器实例 
    pid_incremental_struct spd_pid;     // 速度环PID控制器实例 

    uint16_t can_id;                    // 该电机的反馈ID (e.g., 0x201) 
    uint8_t  is_online;                 // 在线状态标志 
    uint32_t last_msg_time;             // 上次收到报文的HAL_GetTick()时间戳 

    FDCAN_HandleTypeDef* hfdcan;        // 该电机绑定的FDCAN句柄

} DJI_Motor_Instance;


/*************************** Public Functions ***************************/

int32_t dji_degree2encoder(float degree);
void dji_motors_init(void);
void dji_motor_set_location(DJI_Motor_Instance* motor, int32_t location);
void dji_motor_set_speed(DJI_Motor_Instance* motor, int16_t speed);
DJI_Motor_Instance* dji_motor_get_instance(uint8_t motor_index);
void dji_motor_stop_all(void);
void dji_motor_resume_all(void);


/*************************** Internal Functions (for fdcan_bsp) ***************************/

void dji_motor_message_handler(void* instance, FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[8]);

#endif // DJI_MOTOR_H
