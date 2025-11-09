#include "vesc.h"

/************** 内部宏定义与重命名 begin **************/

/************** 内部宏定义与重命名 end **************/


/************** 内部变量与函数 begin **************/

// 存储并转换vesc命令的内容
union s32_to_u8 vesc_content_transform[VESC_MOTOR_NUMS] = {0};

// 电机的极对数, 这里从下标为1开始对应1号电机
static int vesc_motor_poles_s[VESC_MOTOR_NUMS] = {0, 6, 6, 6, 6}; 

// 发送电机速度，下标为电机ID
static int motor_speed_s[VESC_MOTOR_NUMS] = {0};

// (新增) VESC电机与其FDCAN通道的映射表
static FDCAN_HandleTypeDef* vesc_can_map[VESC_MOTOR_NUMS] = {NULL};

// (重构) 内部私有的FDCAN发送函数
static void vesc_fdcan_send_extid(uint32_t id_num);

/************** 内部变量与函数 end **************/

/**
 * @brief  (新增) 初始化并配置一个VESC电机
 */
void VESC_Init_Motor(uint32_t motor_id, FDCAN_HandleTypeDef* hfdcan)
{
    if (motor_id > 0 && motor_id < VESC_MOTOR_NUMS)
    {
        vesc_can_map[motor_id] = hfdcan;
    }
}

/*
1.函数功能：更改vesc电机速度
2.入参：电机ID+目标速度
3.返回值：无
4.用法及调用要求：
5.其它：
*/
void Change_vesc_speed(int motor_id,int target_spd){
	 motor_speed_s[motor_id]=target_spd;
}

/*
1.函数功能：填充并发送 vesc 速度 控制报文的id和content
2.入参：电机ID
3.返回值：无
4.用法及调用要求：间隔1ms以内调用一次
5.其它：
*/
void Com2vesc(uint32_t motor_id){
	uint32_t vesc_speed_id=motor_id+(CAN_PACKET_SET_RPM<<8);
	uint32_t Erpm= motor_speed_s[motor_id] * vesc_motor_poles_s[motor_id];//转换为电角度
	vesc_content_transform[motor_id].u8_data[0]=Erpm>>24;
	vesc_content_transform[motor_id].u8_data[1]=Erpm>>16;
	vesc_content_transform[motor_id].u8_data[2]=Erpm>>8;
	vesc_content_transform[motor_id].u8_data[3]=Erpm;
	vesc_fdcan_send_extid(vesc_speed_id);
}

/*
1.函数功能：所有电调初始速度设为0
2.入参：无
3.返回值：无
4.用法及调用要求：
5.其它：
*/
void Vesc_speed_control_init(void){
	for (int i = 0; i < VESC_MOTOR_NUMS; i++)
    {
		motor_speed_s[i]=0;
	}
}


/**
 * @brief  (重构) 填充VESC控制报文并直接通过FDCAN发送
 * @details 不再依赖can_database，而是通过内部的映射表查找CAN通道
 */
static void vesc_fdcan_send_extid(uint32_t id_num)
{
    uint8_t motor_id = id_num & 0xFF; // 从组合ID中解析出电机ID

    // 安全检查
    if (motor_id == 0 || motor_id >= VESC_MOTOR_NUMS) return;

    // 从映射表中查找此电机对应的FDCAN句柄
    FDCAN_HandleTypeDef* hfdcan = vesc_can_map[motor_id];
    if (hfdcan == NULL)
    {
        // 如果电机没有通过 VESC_Init_Motor 初始化，则不发送
        return; 
    }

    FDCAN_TxHeaderTypeDef TxMessage;
	
	TxMessage.Identifier = id_num;
	TxMessage.IdType = FDCAN_EXTENDED_ID;
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;
	TxMessage.DataLength = FDCAN_DLC_BYTES_4; // VESC RPM指令是4字节
	TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxMessage.BitRateSwitch = FDCAN_BRS_OFF;
	TxMessage.FDFormat = FDCAN_CLASSIC_CAN;
	TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxMessage.MessageMarker = 0;
	
    // 数据指针直接从全局数组获取
    uint8_t* data_ptr = vesc_content_transform[motor_id].u8_data;
    
	HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, data_ptr);
}

#if 0

void set_vesc_position(uint32_t ID,int32_t location){
	uint32_t vesc_position_id=ID+(CAN_PACKET_SET_POS<<8);
	location*=1000000;
	vesc_content_transform[ID].u8_data[0]=location>>24;
	vesc_content_transform[ID].u8_data[1]=location>>16;
	vesc_content_transform[ID].u8_data[2]=location>>8;
	vesc_content_transform[ID].u8_data[3]=location;
	Write_database_vesc_extid(vesc_position_id);
}

void set_vesc_current(uint32_t ID,int32_t current){
	uint32_t vesc_current_id=ID+(CAN_PACKET_SET_CURRENT<<8);
	current*=1000;
	vesc_content_transform[ID].u8_data[0]=current>>24;
	vesc_content_transform[ID].u8_data[1]=current>>16;
	vesc_content_transform[ID].u8_data[2]=current>>8;
	vesc_content_transform[ID].u8_data[3]=current;
	Write_database_vesc_extid(vesc_current_id);
}

#endif