#ifndef FOC_H
#define FOC_H

#include "main.h"
#include "fdcan_bsp.h"
#include "dji_motor.h"
#include "pid.h"


#define Motor_Num	8

typedef enum
{
	Motor_Disable 		 = 0,
	Current_Mode 		 = 1,
	Speed_Mode			 = 2,
	Position_Mode		 = 3,
	Calib_Motor_R_L_Flux = 4,
	Calib_EncoderOffset  = 5,
	Default_Param		 = 6,
	Save_Param			 = 7,
	Clear_Error 		 = 8,
}Mode_TypeDef; 

typedef enum
{
	CAN_SET_MODE			= 0x00,
	CAN_GET_MODE			= 0x01,
	CAN_SET_CURRENT			= 0x02,
	CAN_GET_CURRENT_SET		= 0x03,
	CAN_SET_SPEED			= 0x04,
	CAN_GET_SPEED_SET		= 0x05,
	CAN_SET_POS				= 0x06,
	CAN_GET_POS_SET			= 0x07,
/***********************************/
	CAN_SET_NODE_ID			= 0x08,
	CAN_GET_NODE_ID			= 0x09,
	CAN_SET_POLEPARIS		= 0x0A,
	CAN_GET_POLEPARIS		= 0x0B,
	CAN_SET_ENCODER_TYPE 	= 0x0C,
	CAN_GET_ENCODER_TYPE	= 0x0D,
	CAN_SET_CURRENT_CAL		= 0x0E,
	CAN_GET_CURRENT_CAL		= 0x0F,
	CAN_SET_CURRENT_LIMIT	= 0x10,
	CAN_GET_CURRENT_LIMIT	= 0x11,
	CAN_SET_SPEED_LIMIT		= 0x12,
	CAN_GET_SPEED_LIMIT		= 0x13,
	CAN_SET_ACC 			= 0x14,
	CAN_GET_ACC				= 0x15,
	CAN_SET_DEC				= 0x16,
	CAN_GET_DEC				= 0x17,
	CAN_SET_SPEED_KP		= 0x18,
	CAN_GET_SPEED_KP		= 0x19,
	CAN_SET_SPEED_KI 		= 0x1A,
	CAN_GET_SPEED_KI		= 0x1B,
	CAN_SET_POS_KP			= 0x1C,
	CAN_GET_POS_KP			= 0x1D,
	CAN_SET_POS_KI			= 0x1E,
	CAN_GET_POS_KI			= 0x1F,
	CAN_SET_CAN_HB			= 0x20,
	CAN_GET_CAN_HB			= 0x21,
/***********************************/
	CAN_SET_VBUS			= 0x22,
	CAN_GET_VBUS			= 0x23,
	CAN_SET_IBUS			= 0x24,
	CAN_GET_IBUS			= 0x25,
	CAN_SET_IA				= 0x26,
	CAN_GET_IA				= 0x27,
	CAN_SET_IB				= 0x28,
	CAN_GET_IB				= 0x29,	
	CAN_SET_IC				= 0x2A,
	CAN_GET_IC				= 0x2B,	
	CAN_SET_ID				= 0x2C,
	CAN_GET_ID				= 0x2D,	
	CAN_SET_IQ				= 0x2E,
	CAN_GET_IQ				= 0x2F,	
	CAN_SET_SPEED_FILT		= 0x30,
	CAN_GET_SPEED_FILT		= 0x31,	
	CAN_SET_ENCODER_RAW		= 0x32,
	CAN_GET_ENCODER_RAW		= 0x33,	
	CAN_SET_TEMP			= 0x34,
	CAN_GET_TEMP			= 0x35,	
	CAN_SET_ERROR			= 0x36,
	CAN_GET_ERROR			= 0x37,
}CAN_PARAM_ID;

typedef struct
{
	float float_data;
	uint8_t u8_data[4];
}float_to_u8;

typedef struct
{
    uint32_t u32_data;
    uint8_t u8_data[4];
}u8_to_u32;

typedef struct
{
	uint8_t  node_id;
	uint8_t* data_ptr;
}CAN_Data_TypeDef;

extern CAN_Data_TypeDef CAN_Data_t[Motor_Num];

uint32_t FloatToIntBit(float x);
void FOC_Init();//FOC的初始化
extern u8_to_u32 blazer_speed_transform[3];

extern uint32_t kv_motor_speed_1[300];
extern uint32_t kv_motor_speed_2[300];
extern uint32_t kv_motor_speed_0[300];

//void CAN_SendMessage(uint32_t ID);
void FDCAN1_Transmit_FOC(uint32_t id);
void FDCAN3_Transmit_FOC(uint32_t id);
void set_blazer_mode_can1(uint32_t node_id, float mode);
void set_blazer_mode_can3(uint32_t node_id, float mode);
void set_blazer_speed_can1(uint32_t node_id, float speed);
void set_blazer_speed_can3(uint32_t node_id, float speed);
void set_blazer_speed_acc(uint32_t node_id, float speed_acc);
void set_blazer_speed_dec(uint32_t node_id, float speed_dec);
void set_blazer_speed_kp(uint32_t node_id, float speed_kp);
void set_blazer_speed_ki(uint32_t node_id, float speed_ki);
void set_blazer_cur_lim(uint32_t node_id, float cur_lim);
void get_blazer_speed(void);


#endif
