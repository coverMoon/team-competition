#ifndef VESC_H
#define VESC_H

#include "main.h"
#include "fdcan.h" 

#define VESC_MOTOR_NUMS 5 // 控制的电机总数

// CAN commands
typedef enum {
	CAN_PACKET_SET_DUTY = 0,
	CAN_PACKET_SET_CURRENT,
	CAN_PACKET_SET_CURRENT_BRAKE,
	CAN_PACKET_SET_RPM,
	CAN_PACKET_SET_POS,
} CAN_PACKET_ID;

// 一个32位数据转为4个8位数据进行can的发送
union s32_to_u8{
	uint32_t s32_data;
	uint8_t u8_data[4];
};

extern union s32_to_u8 vesc_content_transform[VESC_MOTOR_NUMS];


/************** 外部接口 begin **************/

// --- 以下三个核心接口保持不变 ---
void Com2vesc(uint32_t motor_id);
void Change_vesc_speed(int motor_id, int target_spd);
void Vesc_speed_control_init(void);

// --- 新增的初始化配置接口 ---
/**
 * @brief  (新增) 初始化并配置一个VESC电机
 * @details 告诉VESC驱动这个电机ID将使用哪个FDCAN总线进行通信。
 * 必须在调用 Com2vesc 之前，为每个要使用的电机调用一次此函数。
 * @param  motor_id: 电机ID (1-4)
 * @param  hfdcan:   指向该电机所使用的FDCAN句柄 (e.g., &hfdcan1)
 */
void VESC_Init_Motor(uint32_t motor_id, FDCAN_HandleTypeDef* hfdcan);

/************** 外部接口 end **************/

#endif
