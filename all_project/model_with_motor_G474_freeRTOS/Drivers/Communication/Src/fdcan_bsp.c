/**
  ******************************************************************************
  * @file           : fdcan_bsp.c
  * @brief          : FDCAN板级支持包实现文件，提供通用的FDCAN报文接收、分发和发送功能
  * @author         : 秦泽宇 & Gemini
  * @date           : 2025-10-09
  * @version        : v1.2
  * @note           : 
  * - v1.1(2025-10-12): 增加了对扩展帧的查询表，使用线性表查询。
  * - v1.2(2025-10-18): 增加了对扩展帧的非精确匹配(掩码匹配)，可用于查询返回报文ID中部分位段
  *                     固定的设备。
  ******************************************************************************
  * @attention
  *
  * [Module Design]
  * 1. 本模块旨在提供一个通用的FDCAN通信底层，将硬件中断与上层业务逻辑解耦。
  * 2. 核心机制是一个“发布-订阅”模型：
  * 	- 上层模块（发布者）通过 `fdcan_bsp_register()` 函数“订阅”自己感兴趣的CAN ID。
  * 	- 当硬件接收到报文时，本模块（调度中心）负责查找该ID的订阅者，并调用其
  * 		注册的回调函数。
  * 3. 为了实现 O(1) 时间复杂度的快速查找，标准帧使用了一个哈希表(直接寻址表)
  * 	 进行CAN ID到回调函数的映射。扩展帧由于ID空间较大，则使用线性查找。
  *
  * [Usage Flow]
  * 1. 在系统初始化早期，调用 `fdcan_bsp_init()` 来初始化内部哈希表。
  * 2. 对于每一个需要接收CAN报文的业务模块（如dji_motor），在其初始化函数中，
  * 	 为每个需要监听的CAN ID创建一个 `FDCAN_Dispatch_t` 结构体，并填入
  * 	 ID、实例指针和回调函数，然后调用 `fdcan_bsp_register()` 进行注册。
  * 3. 所有业务模块注册完毕后，调用 `fdcan_bsp_start()` 并传入相应的FDCAN句柄
  * 	 (如 &hfdcan1)，来启动CAN硬件和中断。
  *
  ******************************************************************************
  */

#include "fdcan_bsp.h"
#include "fdcan.h"
#include <string.h> 


/* ------------------------- Private Defines ------------------------- */

#define MAX_STD_DISPATCH_ITEMS 32     // 标准帧注册项的最大数量
#define MAX_EXT_DISPATCH_ITEMS 16     // 扩展帧注册项的最大数量 (可按需调整)
#define HASH_TABLE_SIZE 2048          // 哈希表大小 (仅用于标准帧)
#define HASH_EMPTY_SLOT UINT16_MAX    // 哈希表中未使用的槽位标记


/* ------------------------- Private Variables ------------------------- */

/** 
 * 分发表，存储所有已注册的CAN ID及其处理方式 
 * */
static FDCAN_Dispatch_t std_dispatch_table[MAX_STD_DISPATCH_ITEMS];

/**
 * 扩展帧分发表，由于扩展帧较大，使用哈希表需要构造专用的哈希函数，较为麻烦，因此暂时
 * 与标准帧分发表分离，使用线性查找法
 */
static FDCAN_Dispatch_t ext_dispatch_table[MAX_EXT_DISPATCH_ITEMS];

/**
 * 哈希表，用于通过CAN ID快速索引到dispatch_table中的条目
 * hash_table[CAN_ID] = index_in_dispatch_table
 */
static uint16_t hash_table[HASH_TABLE_SIZE];

/** 
 * 当前已注册的条目数量
 */
static uint8_t std_dispatch_count = 0; 
static uint8_t ext_dispatch_count = 0;


/* ------------------------- Private Function Prototypes ------------------------- */

static void fdcan_dispatch_router(FDCAN_HandleTypeDef* hfdcan, uint32_t fifo_location);



/* ------------------------- Public Function Implementations ------------------------- */

/**
 * @brief  初始化FDCAN分发系统
 * @details 该函数会初始化用于快速查找CAN ID的哈希表。
 *          必须在注册任何CAN ID之前调用。
 * @param  None
 * @retval None
 */
void fdcan_bsp_init(void)
{
    // 使用 UINT16_MAX (0xFFFF) 填充哈希表，标记所有槽位为空
    for (int i = 0; i < HASH_TABLE_SIZE; ++i)
    {
        hash_table[i] = HASH_EMPTY_SLOT;
    }
    std_dispatch_count = 0;		// 标准帧 ID 计数初始化
    ext_dispatch_count = 0;		// 扩展帧 ID 计数初始化
}

/**
 * @brief  向分发系统注册一个新的CAN ID监听条目
 * @details 各个业务模块(如DJI电机)通过此函数告知分发系统：“当这个CAN ID的报文
 *          到达时，调用该 ID 对应实例的回调函数，并传入一个实例指针”。
 * @param  dispatch_item: 指向一个配置好的FDCAN_Dispatch_t结构体，包含了ID、
 *                      实例指针和处理函数。
 * @param  hfdcan: 该ID所属的FDCAN通道，用于确保分发器知道在哪个外设上监听。
 *                  (当前实现中，所有FDCAN通道共享一个分发表)
 * @retval None
 */
void fdcan_bsp_register(FDCAN_Dispatch_t* dispatch_item, FDCAN_HandleTypeDef* hfdcan)
{
    // 根据ID类型，分发到不同的注册逻辑
    if (dispatch_item->id_type == FDCAN_STANDARD_ID)
    {
        // --- 标准帧注册逻辑 ---
        // 检查是否超出最大注册数量或ID是否有效
        if (std_dispatch_count >= MAX_STD_DISPATCH_ITEMS || dispatch_item->id >= HASH_TABLE_SIZE)
        {
            // TODO: 增加一个错误处理，例如断言或日志
            return; 
        }
        // 检查该ID是否已被注册
        if (hash_table[dispatch_item->id] != HASH_EMPTY_SLOT)
        {
            // TODO: ID冲突，增加错误处理
            return;
        }
        // 1. 将新的分发条目复制到分发表中
        memcpy(&std_dispatch_table[std_dispatch_count], dispatch_item, sizeof(FDCAN_Dispatch_t));
        // 2. 在哈希表中创建映射：CAN_ID -> dispatch_table_index
        hash_table[dispatch_item->id] = std_dispatch_count;
        // 3. 增加已注册条目的计数
        std_dispatch_count++;
    }
    else if (dispatch_item->id_type == FDCAN_EXTENDED_ID)
    {
        // --- 扩展帧注册逻辑 ---
        if (ext_dispatch_count >= MAX_EXT_DISPATCH_ITEMS)
        {
			// TODO: 增加扩展帧注册表已满的错误处理
            return; 
        }
        // 将条目复制到扩展帧的数组中
        memcpy(&ext_dispatch_table[ext_dispatch_count], dispatch_item, sizeof(FDCAN_Dispatch_t));
        ext_dispatch_count++;
    }
    
}


/**
 * @brief  启动并配置一个指定的FDCAN外设
 * @details 该函数会根据提供的配置，设置FDCAN的过滤器，
 *          激活中断通知，并正式启动FDCAN外设。
 * @param  hfdcan: 需要启动的FDCAN外设的句柄指针 (e.g., &hfdcan1)
 * @retval None
 */
void fdcan_bsp_start(FDCAN_HandleTypeDef* hfdcan)
{
    FDCAN_FilterTypeDef sFilterConfig;

    // --- 配置Rx过滤器  ---
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;                      // 使用第一个过滤器
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;       // 掩码过滤
    sFilterConfig.FilterID1 = 0x00000000;               // 接收ID
    sFilterConfig.FilterID2 = 0x00000000;               // 掩码ID(配置为全通)
    
    // 根据不同的FDCAN句柄，将报文路由到不同的FIFO
    if (hfdcan->Instance == FDCAN1)
    {
        sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // FDCAN1 使用 FIFO0
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1; // FDCAN2 使用 FIFO1
    }
    else if (hfdcan->Instance == FDCAN3)
    {
        sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // FDCAN3 使用 FIFO0
    }
    
    if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    // 为扩展帧配置 FilterIndex = 1 的过滤器，同样配置为全通模式。
    // 标准帧和扩展帧都能通过硬件过滤，到达软件分发器。
    sFilterConfig.IdType = FDCAN_EXTENDED_ID;
    sFilterConfig.FilterIndex = 1; // 使用下一个过滤器槽位
    if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    
    // --- 配置全局过滤器 ---
    if (hfdcan->Instance == FDCAN1)
    {
         if(HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
         {
             Error_Handler();
         }
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        if(HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_ACCEPT_IN_RX_FIFO1, FDCAN_ACCEPT_IN_RX_FIFO1, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
        {
             Error_Handler();
        }
    }
    else if (hfdcan->Instance == FDCAN3)
    {
        if(HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
        {
             Error_Handler();
        }
    }

    // --- 激活中断通知 ---
    // 1. Bus-Off 错误中断
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_BUS_OFF, 0) != HAL_OK)
    {
        Error_Handler();
    }
    // 2. FIFO 新消息中断
    if (hfdcan->Instance == FDCAN1)
    {
        if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
        {
            Error_Handler();
        }
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
        {
            Error_Handler();
        }
    }
    else if (hfdcan->Instance == FDCAN3)
    {
        if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
        {
            Error_Handler();
        }
    }
    
    // --- 启动FDCAN ---
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
    {
        Error_Handler();
    }
}


/* ------------------------- HAL Callback Implementations ------------------------- */

/**
 * @brief FDCAN FIFO0 接收回调函数
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        // 调用通用路由函数处理
        fdcan_dispatch_router(hfdcan, FDCAN_RX_FIFO0);
    }
}

/**
 * @brief FDCAN FIFO1 接收回调函数
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
    {
        // 调用通用路由函数处理
        fdcan_dispatch_router(hfdcan, FDCAN_RX_FIFO1);
    }
}

/**
 * @brief FDCAN 错误状态回调函数
 * @note  在发生Bus-Off等严重错误时尝试重新初始化FDCAN外设
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != RESET)
    {
        // 清除标志位并重新初始化
        __HAL_FDCAN_CLEAR_FLAG(hfdcan, FDCAN_FLAG_BUS_OFF);

        if(hfdcan->Instance == FDCAN1)
        {
            MX_FDCAN1_Init(); // 重新初始化
            fdcan_bsp_start(&hfdcan1); // 重新启动配置
        }
        else if(hfdcan->Instance == FDCAN2)
        {
            MX_FDCAN2_Init();
            fdcan_bsp_start(&hfdcan2);
        }
        else if(hfdcan->Instance == FDCAN3)
        {
            MX_FDCAN3_Init();
            fdcan_bsp_start(&hfdcan3);
        }
    }
}


/* ------------------------- Private Function Implementations ------------------------- */

/**
 * @brief 内部通用的报文路由函数，由FIFO回调函数调用
 * @note  对于扩展帧ID，由于某些位段可能包含状态信息，因此使用掩码式匹配，
 *        对于标准帧，由于一般使用标准帧的设备都返回固定ID，因此使用精确匹配
 * @param hfdcan 触发中断的FDCAN句柄
 * @param fifo_location 触发中断的FIFO (e.g., FDCAN_RX_FIFO0)
 */
static void fdcan_dispatch_router(FDCAN_HandleTypeDef* hfdcan, uint32_t fifo_location)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    // 1. 从指定的FIFO中获取报文
    if (HAL_FDCAN_GetRxMessage(hfdcan, fifo_location, &rx_header, rx_data) != HAL_OK)
    {
        // TODO: 获取失败，可以增加错误处理
        return;
    }

    // 根据接收到的ID类型，选择不同的查找方法
    if (rx_header.IdType == FDCAN_STANDARD_ID)
    {
        // --- 标准帧哈希查找逻辑 ---
        uint16_t id = rx_header.Identifier;
        // 通过哈希表查找该ID是否被注册
        if (id < HASH_TABLE_SIZE && hash_table[id] != HASH_EMPTY_SLOT)
        {
            uint16_t index = hash_table[id];
            // 从分发表中找到对应的条目
            FDCAN_Dispatch_t* item = &std_dispatch_table[index]; 
            // 调用注册的回调函数，并传入实例指针和数据
            if (item->handler != NULL)
            {
                item->handler(item->instance_ptr, &rx_header, rx_data);
            }
        }
    }
    else if (rx_header.IdType == FDCAN_EXTENDED_ID)
    {
        // --- 扩展帧线性查找逻辑 ---
        uint32_t received_id = rx_header.Identifier;
        // 遍历所有已注册的扩展帧条目
        for (uint8_t i = 0; i < ext_dispatch_count; i++)
        {
            FDCAN_Dispatch_t* item = &ext_dispatch_table[i];
			      // 使用掩码匹配
            if ((received_id & item->mask) == (item->id & item->mask))
            {
                // ID匹配成功
                if (item->handler != NULL)
                {
                    item->handler(item->instance_ptr, &rx_header, rx_data);
                }
                // 注意：这里可能需要考虑是否要return。如果多个规则可能匹配同一个ID，
                // 就不应该return。由于(received_id & item->mask)得到的位段可以设置为
                // 包含电机CAN ID，因此不同设备一般不会匹配到同一个ID，在这里先设为return。
                return; 
            }
        }
    }
}
