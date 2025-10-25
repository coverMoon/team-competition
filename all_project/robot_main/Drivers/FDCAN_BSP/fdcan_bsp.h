/**
  ******************************************************************************
  * @file           : fdcan_bsp.h
  * @brief          : FDCAN板级支持包接口文件
  ******************************************************************************
  */


#ifndef FDCAN_BSP_H
#define FDCAN_BSP_H

#include "main.h"

// 引入 HAL 库 fdcan 句柄
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

/**
 * @brief FDCAN分发条目的结构体定义
 * 每一个条目代表一个被监听的CAN ID及其处理方式
 */
typedef struct{
    // ID类型, FDCAN_STANDARD_ID 或 FDCAN_EXTENDED_ID
    uint32_t id_type;

    // 监听的 CAN ID (支持标准帧和扩展帧)
    uint32_t id;

    // 掩码，用于ID匹配。二进制位为1代表关心，为0代表不关心。
    uint32_t mask;

    // 指向与此ID关联的设备实例的指针 (e.g., &dji_motors[0])
    // 这个指针会在回调时作为参数传回，用于区分具体设备
    void* instance_ptr;

    /**
     * @brief 指向处理该报文的回调函数指针
     * @param instance_ptr 注册时传入的设备实例指针
     * @param rx_header   FDCAN 接收到的报文头，为兼容性设置，部分设备使用扩展帧，需要从 ID 段解包出必要信息
     * @param rx_data[8]   接收到的8字节CAN数据
     */
    void (*handler)(void* instance_ptr, FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[8]);

} FDCAN_Dispatch_t;


/*************************** Public Functions ***************************/

void fdcan_bsp_init(void);
void fdcan_bsp_start(FDCAN_HandleTypeDef* hfdcan);
void fdcan_bsp_register(FDCAN_Dispatch_t* dispatch_item, FDCAN_HandleTypeDef* hfdcan);


#endif // FDCAN_BSP_H
