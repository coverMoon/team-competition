#include "foc.h"
#include "string.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

float_to_u8 blazer_content_transform[Motor_Num];
u8_to_u32 blazer_speed_transform[3];

int kv_speed_i=0;
uint32_t kv_motor_speed_1[300]={0};
uint32_t kv_motor_speed_2[300]={0};
uint32_t kv_motor_speed_0[300]={0};

CAN_Data_TypeDef CAN_Data[Motor_Num]=
{
	{0x00, (uint8_t*)(&blazer_content_transform[0].u8_data)},
	{0x01, (uint8_t*)(&blazer_content_transform[1].u8_data)},
	{0x02, (uint8_t*)(&blazer_content_transform[2].u8_data)},
	{0x03, (uint8_t*)(&blazer_content_transform[3].u8_data)},
	{0x04, (uint8_t*)(&blazer_content_transform[4].u8_data)},
	{0x05, (uint8_t*)(&blazer_content_transform[5].u8_data)},
	{0x06, (uint8_t*)(&blazer_content_transform[6].u8_data)},
	{0x07, (uint8_t*)(&blazer_content_transform[7].u8_data)},
};

/**
 * @brief  FOC电机报文的统一处理函数 (回调函数)
 * @details 这个函数会被 `fdcan_bsp` 模块在收到匹配的扩展CAN ID时调用。
 * @param  instance_ptr: 注册时传入的电机索引 (void*类型)
 * @param  rx_header: FDCAN 接收到的报文头，在此函数中未使用
 * @param  rx_data: 接收到的CAN数据
 */
static void foc_message_handler(void* instance_ptr, FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[8])
{
    // instance_ptr 在注册时被用来传递电机索引，这里把它转换回来
    uint32_t motor_index = (uint32_t)instance_ptr;

    // 安全检查，防止索引越界
    if (motor_index >= 3) // 目前只处理0,1,2号电机的速度反馈
    {
        return;
    }

    // 将接收到的数据直接存入对应的全局缓冲区
    // FOC的速度反馈是4个字节的float数据
    memcpy(blazer_speed_transform[motor_index].u8_data, rx_data, 4);
}

uint32_t FloatToIntBit(float x)
{
	uint32_t *pInt;
	pInt = (uint32_t*)(&x);
	return *pInt;
}

float IntBitToFloat(uint32_t x)
{
	float * pFloat;
	pFloat = (float *)(&x);
	return *pFloat;
}

void FOC_Init()
{
	  //set_blazer_mode_can1(0x01, (float)Speed_Mode);
    //set_blazer_mode_can1(0x02, (float)Speed_Mode);
    //set_blazer_mode_can1(0x03, (float)Speed_Mode);
    
    set_blazer_mode_can3(0x00, (float)Speed_Mode);
    set_blazer_mode_can3(0x01, (float)Speed_Mode);
    set_blazer_mode_can3(0x02, (float)Speed_Mode);
    
    //改变发射电机的电流上限
    set_blazer_cur_lim(0x00,50);
    set_blazer_cur_lim(0x01,50);
    set_blazer_cur_lim(0x02,50);
    
    //改变发射电机的加速度，暂且取1000，手册中的最大值为1000，初值为50，后续再调整
    set_blazer_speed_acc(0x00,1000);
    set_blazer_speed_acc(0x01,1000);
    set_blazer_speed_acc(0x02,1000);
    
    //改变发射电机的减速度，暂且取1000，手册中的最大值为1000，初值为50，后续再调整
    set_blazer_speed_dec(0x00,1000);
    set_blazer_speed_dec(0x01,1000);
    set_blazer_speed_dec(0x02,1000);

	// --- 向 fdcan_bsp 注册反馈报文的处理器 ---
    FDCAN_Dispatch_t dispatch_item;

	// 配置所有FOC电机回调的通用部分
    dispatch_item.id_type = FDCAN_EXTENDED_ID;
    dispatch_item.handler = foc_message_handler;

	// 注册电机0的速度反馈 (ID: 0x031, 即 CAN_GET_SPEED_FILT)
    dispatch_item.id = (0x00 << 8) | CAN_GET_SPEED_FILT; // 0x031
    dispatch_item.instance_ptr = (void*)0; // 用 instance_ptr 巧妙地传递电机索引 0
    fdcan_bsp_register(&dispatch_item, &hfdcan3);

    // 注册电机1的速度反馈 (ID: 0x131)
    dispatch_item.id = (0x01 << 8) | CAN_GET_SPEED_FILT; // 0x131
    dispatch_item.instance_ptr = (void*)1; // 传递电机索引 1
    fdcan_bsp_register(&dispatch_item, &hfdcan3);

    // 注册电机2的速度反馈 (ID: 0x231)
    dispatch_item.id = (0x02 << 8) | CAN_GET_SPEED_FILT; // 0x231
    dispatch_item.instance_ptr = (void*)2; // 传递电机索引 2
    fdcan_bsp_register(&dispatch_item, &hfdcan3);
}


///**
//   * @brief  CAN1过滤器初始化函数
//   * @param  无
//   * @retval 无
//   */
//void CAN_Filter_Init(void)
//{
//	CAN_FilterTypeDef CAN_FilterStruct;
//	
//	CAN_FilterStruct.FilterActivation	  = ENABLE;
//	CAN_FilterStruct.FilterMode			  = CAN_FILTERMODE_IDMASK;
//	CAN_FilterStruct.FilterScale		  = CAN_FILTERSCALE_32BIT;
//	CAN_FilterStruct.FilterIdHigh		  = 0x0000;
//	CAN_FilterStruct.FilterIdLow		  = 0x0000;
//	CAN_FilterStruct.FilterMaskIdHigh	  = 0x0000;
//	CAN_FilterStruct.FilterMaskIdLow	  = 0x0000;
//	CAN_FilterStruct.FilterBank			  = 0;
//	CAN_FilterStruct.FilterFIFOAssignment = CAN_RX_FIFO0;
//	
//	HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterStruct);
//	HAL_CAN_Start(&hcan1);
//    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
//}

///**
//   * @brief  CAN发送报文
//   * @param  ID 由节点ID和参数ID构成的复合ID
//   * @retval 无
//   */
//void CAN_SendMessage(uint32_t ID)
//{
//	CAN_TxHeaderTypeDef CAN_TxHeaderStruct;
//	uint8_t send_num = 0;
//	uint32_t pTxMailbox = 0;
//	
//	CAN_TxHeaderStruct.StdId			  = 0;
//	CAN_TxHeaderStruct.ExtId			  = ID;
//	CAN_TxHeaderStruct.IDE				  = CAN_ID_EXT;
//	CAN_TxHeaderStruct.RTR				  = CAN_RTR_DATA;
//	CAN_TxHeaderStruct.DLC				  = 0x04;
//	CAN_TxHeaderStruct.TransmitGlobalTime = ENABLE;
//	HAL_CAN_AddTxMessage(&hcan1, &CAN_TxHeaderStruct, CAN_Data[ID >> 8].data_ptr, &pTxMailbox);
////	while(HAL_CAN_AddTxMessage(&hcan1, &CAN_TxHeaderStruct, CAN_Data[ID >> 8].data_ptr, &pTxMailbox) != HAL_OK)
////	{
////		delay_us(25);
////		/*发送阻滞*/
////		if(++ send_num == 10)
////			break;
////	}
//}

/*
@brief FDCAN1发送函数
@param 
	TxData 待发送数据
	id 帧ID
	len 待发送数据帧长度
	EXTflag = 1 使用扩展帧 EXTflag = 0 使用标准帧
@return
	0 发送成功
	1 发送失败
*/
void FDCAN1_Transmit_FOC(uint32_t id)
{
	//printf("lll");
	FDCAN_TxHeaderTypeDef TxMessage;
	
	TxMessage.Identifier 			= id;					/* 设置发送帧消息的ID */
	
	if(1)
		TxMessage.IdType			= FDCAN_EXTENDED_ID;	/* 扩展ID */
	else
		TxMessage.IdType			= FDCAN_STANDARD_ID;	/* 标准ID */
	
	TxMessage.TxFrameType 			= FDCAN_DATA_FRAME;		/* 数据帧 */
	TxMessage.DataLength 			= 0x04;					/* 设置数据长度 */
	TxMessage.ErrorStateIndicator 	= FDCAN_ESI_ACTIVE;		/* 设置错误状态指 */
	TxMessage.BitRateSwitch 		= FDCAN_BRS_OFF;		/* 关闭可变波特率 */
	TxMessage.FDFormat 				= FDCAN_CLASSIC_CAN;			/* FDCAN格式 */
	TxMessage.TxEventFifoControl 	= FDCAN_NO_TX_EVENTS;	/* 用于发送事件FIFO控制, 无发送事件*/
	TxMessage.MessageMarker 		= 0;					/* 用于复制到TX EVENT FIFO的消息Maker来识别消息状态，范围0-0xFF */
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxMessage, CAN_Data[id >> 8].data_ptr);
}

void FDCAN3_Transmit_FOC(uint32_t id)
{
	//printf("lll");
	FDCAN_TxHeaderTypeDef TxMessage;
	
	TxMessage.Identifier 			= id;					/* 设置发送帧消息的ID */
	
	if(1)
		TxMessage.IdType			= FDCAN_EXTENDED_ID;	/* 扩展ID */
	else
		TxMessage.IdType			= FDCAN_STANDARD_ID;	/* 标准ID */
	
	TxMessage.TxFrameType 			= FDCAN_DATA_FRAME;		/* 数据帧 */
	TxMessage.DataLength 			= 0x04;					/* 设置数据长度 */
	TxMessage.ErrorStateIndicator 	= FDCAN_ESI_ACTIVE;		/* 设置错误状态指 */
	TxMessage.BitRateSwitch 		= FDCAN_BRS_OFF;		/* 关闭可变波特率 */
	TxMessage.FDFormat 				= FDCAN_CLASSIC_CAN;			/* FDCAN格式 */
	TxMessage.TxEventFifoControl 	= FDCAN_NO_TX_EVENTS;	/* 用于发送事件FIFO控制, 无发送事件*/
	TxMessage.MessageMarker 		= 0;					/* 用于复制到TX EVENT FIFO的消息Maker来识别消息状态，范围0-0xFF */
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxMessage, CAN_Data[id >> 8].data_ptr);
}
/**
   * @brief  设定运行模式
   * @param  node_id   驱动节点ID 
			 mode      模式 
			 0:闲置状态 	1:电流模式 2:速度模式 3:位置模式 4:校准电阻电感磁链 
		     5:校准编码器偏移 6:恢复默认配置 7:保存当前配置 8:清除错误
   * @retval 无
   */
void set_blazer_mode_can1(uint32_t node_id, float mode)
{
	uint32_t blazer_mode_id = node_id << 8 | CAN_SET_MODE;
	uint32_t mode_u32;
	
	mode_u32 = FloatToIntBit(mode);
	
	blazer_content_transform[node_id].u8_data[0] = mode_u32 >> 24;
	blazer_content_transform[node_id].u8_data[1] = mode_u32 >> 16;
	blazer_content_transform[node_id].u8_data[2] = mode_u32 >> 8;
	blazer_content_transform[node_id].u8_data[3] = mode_u32;
	
	FDCAN1_Transmit_FOC(blazer_mode_id);
}
void set_blazer_mode_can3(uint32_t node_id, float mode)
{
	uint32_t blazer_mode_id = node_id << 8 | CAN_SET_MODE;
	uint32_t mode_u32;
	
	mode_u32 = FloatToIntBit(mode);
	
	blazer_content_transform[node_id].u8_data[0] = mode_u32 >> 24;
	blazer_content_transform[node_id].u8_data[1] = mode_u32 >> 16;
	blazer_content_transform[node_id].u8_data[2] = mode_u32 >> 8;
	blazer_content_transform[node_id].u8_data[3] = mode_u32;
	
	FDCAN3_Transmit_FOC(blazer_mode_id);
}

/**
   * @brief  设定电机转速
			 转速为机械转速
   * @param  node_id 驱动节点ID 
             speed   设定速度(r/s)
   * @retval 无
   */
void set_blazer_speed_can1(uint32_t node_id, float speed)
{
	uint32_t blazer_speed_id = node_id << 8 | CAN_SET_SPEED;
	uint32_t speed_u32;
	
	speed_u32 = FloatToIntBit(speed);
	
	blazer_content_transform[node_id].u8_data[0] = speed_u32 >> 24;
	blazer_content_transform[node_id].u8_data[1] = speed_u32 >> 16;
	blazer_content_transform[node_id].u8_data[2] = speed_u32 >> 8;
	blazer_content_transform[node_id].u8_data[3] = speed_u32;
	
  FDCAN1_Transmit_FOC(blazer_speed_id);
}

void set_blazer_speed_can3(uint32_t node_id, float speed)
{
	uint32_t blazer_speed_id = node_id << 8 | CAN_SET_SPEED;
	uint32_t speed_u32;
	
	speed_u32 = FloatToIntBit(speed);
	
	blazer_content_transform[node_id].u8_data[0] = speed_u32 >> 24;
	blazer_content_transform[node_id].u8_data[1] = speed_u32 >> 16;
	blazer_content_transform[node_id].u8_data[2] = speed_u32 >> 8;
	blazer_content_transform[node_id].u8_data[3] = speed_u32;
	
  FDCAN3_Transmit_FOC(blazer_speed_id);
}

/**
   * @brief  设定电机加速度
   * @param  node_id   驱动节点ID 
             speed_acc 设定加速度(r/(s*s))
   * @retval 无
    //仅用于can3,如需在别的can口上使用，需要改动transmit函数
   */
void set_blazer_speed_acc(uint32_t node_id, float speed_acc)
{
	uint32_t blazer_speed_acc_id = node_id << 8 | CAN_SET_ACC;
	uint32_t speed_acc_u32;
	
	speed_acc_u32 = FloatToIntBit(speed_acc);
	
	blazer_content_transform[node_id].u8_data[0] = speed_acc_u32 >> 24;
	blazer_content_transform[node_id].u8_data[1] = speed_acc_u32 >> 16;
	blazer_content_transform[node_id].u8_data[2] = speed_acc_u32 >> 8;
	blazer_content_transform[node_id].u8_data[3] = speed_acc_u32;
	
  FDCAN3_Transmit_FOC(blazer_speed_acc_id);
}

/**
   * @brief  设定电机减速度
   * @param  node_id   驱动节点ID 
             speed_dec 设定减速度(r/(s*s))
   * @retval 无
    //仅用于can3,如需在别的can口上使用，需要改动transmit函数
   */
void set_blazer_speed_dec(uint32_t node_id, float speed_dec)
{
	uint32_t blazer_speed_dec_id = node_id << 8 | CAN_SET_DEC;
	uint32_t speed_dec_u32;
	
	speed_dec_u32 = FloatToIntBit(speed_dec);
	
	blazer_content_transform[node_id].u8_data[0] = speed_dec_u32 >> 24;
	blazer_content_transform[node_id].u8_data[1] = speed_dec_u32 >> 16;
	blazer_content_transform[node_id].u8_data[2] = speed_dec_u32 >> 8;
	blazer_content_transform[node_id].u8_data[3] = speed_dec_u32;
	
	 FDCAN3_Transmit_FOC(blazer_speed_dec_id);
}

/**
   * @brief  设定电机电流上限
   * @param  node_id   驱动节点ID 
             cur_lim 电流上限(A)
   * @retval 无
    //仅用于can3,如需在别的can口上使用，需要改动transmit函数
   */
void set_blazer_cur_lim(uint32_t node_id, float cur_lim)
{
	uint32_t blazer_cur_lim_id = node_id << 8 | CAN_SET_CURRENT_LIMIT;
	uint32_t cur_lim_u32;
	
	cur_lim_u32 = FloatToIntBit(cur_lim);
	
	blazer_content_transform[node_id].u8_data[0] = cur_lim_u32 >> 24;
	blazer_content_transform[node_id].u8_data[1] = cur_lim_u32 >> 16;
	blazer_content_transform[node_id].u8_data[2] = cur_lim_u32 >> 8;
	blazer_content_transform[node_id].u8_data[3] = cur_lim_u32;
	
	FDCAN3_Transmit_FOC(blazer_cur_lim_id);
}


/**
   * @brief  设定速度环kp
   * @param  node_id 驱动节点ID 
             kp 	 速度环kp
   * @retval 无
   */
void set_blazer_speed_kp(uint32_t node_id, float speed_kp)
{
	uint32_t blazer_speed_kp_id = node_id << 8 | CAN_SET_SPEED_KP;
	uint32_t speed_kp_u32;
	
	speed_kp_u32 = FloatToIntBit(speed_kp);
	
	blazer_content_transform[node_id].u8_data[0] = speed_kp_u32 >> 24;
	blazer_content_transform[node_id].u8_data[1] = speed_kp_u32 >> 16;
	blazer_content_transform[node_id].u8_data[2] = speed_kp_u32 >> 8;
	blazer_content_transform[node_id].u8_data[3] = speed_kp_u32;
	
	 FDCAN1_Transmit_FOC(blazer_speed_kp_id);
}

/**
   * @brief  设定速度环ki
   * @param  node_id 驱动节点ID 
             kp 	 速度环ki
   * @retval 无
   */
void set_blazer_speed_ki(uint32_t node_id, float speed_ki)
{
	uint32_t blazer_speed_ki_id = node_id << 8 | CAN_SET_SPEED_KI;
	uint32_t speed_ki_u32;
	
	speed_ki_u32 = FloatToIntBit(speed_ki);
	
	blazer_content_transform[node_id].u8_data[0] = speed_ki_u32 >> 24;
	blazer_content_transform[node_id].u8_data[1] = speed_ki_u32 >> 16;
	blazer_content_transform[node_id].u8_data[2] = speed_ki_u32 >> 8;
	blazer_content_transform[node_id].u8_data[3] = speed_ki_u32;
	
    FDCAN1_Transmit_FOC(blazer_speed_ki_id);
}

/**
   * @brief  读取电机当前速度
   * @param  无
   * @retval 无
   */
int get_speed_count=4;//初值为4使得第一次进这个函数的时候即可立刻获取速度值，之后是每五次获取一次速度值
void get_blazer_speed()
{
    uint32_t id_1,id_2,id_3;
    id_1=0x031;
    id_2=0x131;
    id_3=0x231;
    get_speed_count++;
    if(get_speed_count==5)
    {
		get_speed_count = 0;
        FDCAN3_Transmit_FOC(id_1);
        FDCAN3_Transmit_FOC(id_2);
        FDCAN3_Transmit_FOC(id_3);
        
        blazer_speed_transform[0].u32_data=((uint32_t)blazer_speed_transform[0].u8_data[0] << 24 |
								(uint32_t)blazer_speed_transform[0].u8_data[1] << 16 |
								(uint32_t)blazer_speed_transform[0].u8_data[2] << 8 |
								(uint32_t)blazer_speed_transform[0].u8_data[3]);
        blazer_speed_transform[1].u32_data=((uint32_t)blazer_speed_transform[1].u8_data[0] << 24 |
                                    (uint32_t)blazer_speed_transform[1].u8_data[1] << 16 |
                                    (uint32_t)blazer_speed_transform[1].u8_data[2] << 8 |
                                    (uint32_t)blazer_speed_transform[1].u8_data[3]);
        blazer_speed_transform[2].u32_data=((uint32_t)blazer_speed_transform[2].u8_data[0] << 24 |
                                    (uint32_t)blazer_speed_transform[2].u8_data[1] << 16 |
                                    (uint32_t)blazer_speed_transform[2].u8_data[2] << 8 |
                                    (uint32_t)blazer_speed_transform[2].u8_data[3]);
        
        // 记录数据到日志数组
        if (kv_speed_i < 300) // 防止数组越界
        {
            kv_motor_speed_0[kv_speed_i] = blazer_speed_transform[0].u32_data;
            kv_motor_speed_1[kv_speed_i] = blazer_speed_transform[1].u32_data;
            kv_motor_speed_2[kv_speed_i] = blazer_speed_transform[2].u32_data;
            kv_speed_i++;
        }
    }
}