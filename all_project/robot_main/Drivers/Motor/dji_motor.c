/**
  ******************************************************************************
  * @file           : dji_motor.c
  * @brief          : DJI电机控制驱动实现文件
  * @author         : 秦泽宇 & Gemini
  * @date           : 2025-10-09
  * @version        : v1.2
  * @note           :
  * - v1.2 (2025-10-20): 新增对FreeRTOS的适配升级，使用条件编译与裸机区分。
  * - v1.1 (2025-10-18): 修改部分函数标签，以适配更新后的fdcan_bsp。
  ******************************************************************************
  * @attention
  * 
  * [Module Design]
  * 1. 本模块封装了DJI RM系列电机(如M3508, C620/C610电调)的位置和速度闭环控制。
  * 2. 使用前必须确保`pid`模块和`fdcan_bsp`模块已经正确移植并可用。
  * 3. 所有电机实例存储在一个静态全局数组中，通过索引进行访问。
  *
  * [Usage Flow]
  * 1. 在 `dji_motors_init()` 函数中，根据硬件连接，修改并配置每个电机
  *    的CAN ID, 所属的FDCAN通道, 以及两套PID参数。
  * 2. 在初始化代码中(例如 `main.c` 的用户代码区)，调用 `dji_motors_init()`。
  *	   注：此外还要实现`fdcan_bsp`模块的初始化，详见 fdcan_bsp.c
  * 3. 在需要控制电机的地方，首先通过 `dji_motor_get_instance(motor_index)`
  *    来获取指定电机的实例指针。
  * 4. 使用获取到的指针，调用 `dji_motor_set_location()` 或
  *    `dji_motor_set_speed()` 来下达控制指令。
  * 5. 电机的CAN报文接收和PID计算在 `dji_motor_message_handler()` 回调函数中
  *    自动完成，无需用户干预。
  * 
  * [FreeRTOS 线程安全支持]
  * 1. 由于DJI电机的上层API不涉及发送CAN报文，因此线程安全仅出现在对共享变量的读/写
  *    以及可能(但微小)的中断冲突中。本驱动通过实现临界区保护，解决在 FreeRTOS 环境下
  *    任务与CAN中断并发访问共享数据（如目标值、控制模式）可能导致的风险。
  * 2. 所有线程安全代码通过条件编译宏 `USE_FREERTOS` 管理，确保了对裸机环境的兼容性。
  * 3. 要在FreeRTOS项目中使用，请在全局头文件 (如 `main.h`) 中添加以下定义：
  *     #ifndef USE_FREERTOS
  *     #define USE_FREERTOS
  *     #endif
  * 4. 驱动内部已自动处理所有并发保护，上层应用开发者无需额外操作。
  * 
  *
  ******************************************************************************
  */

#include "dji_motor.h"
#include "fdcan_bsp.h" 

#ifdef USE_FREERTOS
    #include "FreeRTOS.h"
    #include "task.h" 
#endif


/* ------------------------- Private Defines ------------------------- */

// DJI电机控制报文的ID
#define CAN_ID_FIRST_FOUR_MOTORS  0x200
#define CAN_ID_LAST_FOUR_MOTORS   0x1FF

// M3508电机编码器分辨率
#define DJI_ENCODER_RESOLUTION    8192


/* ------------------------- Static Variables ------------------------- */

/**
 * @brief 全局电机实例数组，存储所有电机的状态和配置
 */
static DJI_Motor_Instance dji_motors[DJI_MOTOR_COUNT];

/**
 * @brief 全局电机控制使能标志
 */
static bool dji_motor_control_enabled = true;


/* ------------------------- Private Function Prototypes ------------------------- */

static void get_motor_measure(DJI_Motor_Instance* motor, uint8_t rx_data[8]);
static void get_total_angle(DJI_Motor_Instance* motor);
static void dji_motor_send_commands(DJI_Motor_Instance* trigger_motor);
static void dji_motor_configure(uint8_t index, const uint16_t id, FDCAN_HandleTypeDef* hfdcan, 
                                float loc_kp, float loc_ki, float loc_kd,
                                float spd_kp, float spd_ki, float spd_kd,
																float loc_out_limit_up, float loc_out_limit_down,

                                                                float spd_out_limit_up, float spd_out_limit_down);
// 用于并发控制的私有函数
static void dji_critical_section_enter(void);
static void dji_critical_section_exit(void);

/* ------------------------- Public Function Implementations ------------------------- */

/**
 * @brief  将一个角度值(度)转换为编码器值
 * @param  degree: 角度值(度)，可超过一个周期
 * @retval 计算得到的目标编码器值
 */
int32_t dji_degree2encoder(float degree)
{
		float ratio = (float)DJI_ENCODER_RESOLUTION / 360.0f;
		
		return (int32_t)(degree * ratio);
}
											
/**
 * @brief  初始化所有DJI电机实例
 * @details 此函数会根据内部预设的配置，初始化每个电机的数据结构、PID参数，
 * 并注册到 `fdcan_bsp` 模块中。
 * @param  None
 * @retval None
 */
void dji_motors_init(void)
{
    // --- 在这里集中配置所有电机 ---
    // 可以根据实际情况修改每个电机的 CAN ID, FDCAN通道, 以及PID参数
    
    // 示例配置: 前4个电机在FDCAN1, 后4个在FDCAN2
    // PID参数直接从 pid.c 文件中迁移过来
    dji_motor_configure(0, CAN_3508_2006_M1_ID, &hfdcan1, 
												0.15f, 0.001f, 0.035f, 
												24.0f, 1.2f, 0.03f, 
												500, -500, 9000, -9000);
    dji_motor_configure(1, CAN_3508_2006_M2_ID, &hfdcan1, 
												0.15f, 0.001f, 0.035f, 
												24.0f, 1.2f, 0.03f, 
												500, -500, 12000, -12000);
    dji_motor_configure(2, CAN_3508_2006_M3_ID, &hfdcan1, 
												0.15f, 0.001f, 0.035f, 
												24.0f, 1.2f, 0.03f, 
												500, -500, 12000, -12000);
//    dji_motor_configure(3, CAN_3508_2006_M4_ID, &hfdcan1, 
//												0.15f, 0.001f, 0.035f, 
//												24.0f, 1.2f, 0.03f, 
//												500, -500, 12000, -12000);
//    dji_motor_configure(4, CAN_3508_2006_M5_ID, &hfdcan2, 
//												0.15f, 0.001f, 0.03f,  
//												24.0f, 1.2f, 0.035f, 
//												500, -500, 12000, -12000);
//    dji_motor_configure(5, CAN_3508_2006_M6_ID, &hfdcan2, 
//												0.15f, 0.001f, 0.03f,  
//												24.0f, 1.2f, 0.035f, 
//												500, -500, 12000, -12000);
//    dji_motor_configure(6, CAN_3508_2006_M7_ID, &hfdcan2, 
//												0.15f, 0.001f, 0.03f,  
//												24.0f, 1.2f, 0.035f, 
//												500, -500, 12000, -12000);
//    dji_motor_configure(7, CAN_3508_2006_M8_ID, &hfdcan2, 
//												0.15f, 0.001f, 0.03f,  
//												24.0f, 1.2f, 0.035f, 
//												500, -500, 12000, -12000);
}

/**
 * @brief  获取指定索引的电机实例指针
 * @details 通过此函数获取电机实例，以便将其传递给set系列函数。
 * @param  motor_index: 电机索引 (0 to DJI_MOTOR_COUNT-1)
 * @retval 指向电机实例的指针，如果索引无效则返回NULL
 */
DJI_Motor_Instance* dji_motor_get_instance(uint8_t motor_index)
{
    if (motor_index >= DJI_MOTOR_COUNT)
    {
        return NULL;
    }
    return &dji_motors[motor_index];
}

/**
 * @brief  设置一个电机的位置目标
 * @details 调用此函数会自动将电机切换到位置闭环模式。
 * @param  motor: 指向要控制的电机实例的指针
 * @param  location: 目标位置 (编码器累计值)
 * @retval None
 */
void dji_motor_set_location(DJI_Motor_Instance* motor, int32_t location)
{
    if (motor == NULL) return;

    dji_critical_section_enter();   // 进入临界区
    motor->control_mode = MOTOR_MODE_POSITION;
    motor->target_loc = location;
    dji_critical_section_exit();    // 退出临界区
}

/**
 * @brief  设置一个电机的速度目标
 * @details 调用此函数会自动将电机切换到速度闭环模式。
 * @param  motor: 指向要控制的电机实例的指针
 * @param  speed: 目标速度 (RPM)
 * @retval None
 */
void dji_motor_set_speed(DJI_Motor_Instance* motor, int16_t speed)
{
    if (motor == NULL) return;

    dji_critical_section_enter();   // 进入临界区
    motor->control_mode = MOTOR_MODE_SPEED;
    motor->target_speed = speed;
    dji_critical_section_exit();    // 退出临界区
}

/**
 * @brief   停止所有DJI电机的力矩输出
 * @details 调用此函数后，所有发送给电机的指令电流值将为0，使电机处于自由状态。
 */
void dji_motor_stop_all(void)
{
    dji_critical_section_enter();   // 进入临界区
    dji_motor_control_enabled = false;
    dji_critical_section_exit();    // 退出临界区
}

/**
 * @brief   恢复所有DJI电机的力矩输出
 * @details 调用此函数后，电机将根据PID计算结果恢复正常的力矩控制。
 */
void dji_motor_resume_all(void)
{
    dji_critical_section_enter();   // 进入临界区
    dji_motor_control_enabled = true;
    dji_critical_section_exit();    // 退出临界区
}

/* ------------------------- Internal/Callback Function Implementations ------------------------- */

/**
 * @brief  电机报文的统一处理函数 (回调函数)
 * @details 该函数会被 `fdcan_bsp` 模块在收到匹配的CAN ID时调用。
 * 					不由用户直接调用。
 * @param  instance: 指向触发回调的电机实例的void指针
 * @param  rx_header: FDCAN 接收到的报文头，在此函数中未使用
 * @param  rx_data: 接收到的8字节CAN数据
 * @retval None
 */
void dji_motor_message_handler(void* instance, FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[8])
{
    if (instance == NULL) return;
    
	// 将接收到的实例转换回电机实例
    DJI_Motor_Instance* motor = (DJI_Motor_Instance*)instance;

    // 1. 解析数据并更新电机状态
    get_motor_measure(motor, rx_data);
    
    // 2. 更新在线状态
    motor->is_online = 1;
    motor->last_msg_time = HAL_GetTick();

    // 3. 计算累计角度
    get_total_angle(motor);

    // 4. 执行PID闭环控制计算
    if (motor->control_mode == MOTOR_MODE_POSITION)
    {
        // 串级PID: 位置环的输出是速度环的目标
        float speed_target = Pid_incremental_cal(&motor->loc_pid, motor->measure.total_angle, motor->target_loc);
        Pid_incremental_cal(&motor->spd_pid, motor->measure.speed_rpm, speed_target);
    }
    else // MOTOR_MODE_SPEED
    {
        Pid_incremental_cal(&motor->spd_pid, motor->measure.speed_rpm, motor->target_speed);
    }

    // 5. 发送控制指令 (任何电机反馈都会触发对所有电机的指令发送)
    dji_motor_send_commands(motor);
}


/* ------------------------- Private Function Implementations ------------------------- */

/**
 * @brief  配置单个电机实例并注册到FDCAN分发器
 * @param index:             	电机在dji_motors数组中的索引 (0-7)
 * @param id:                	电机的CAN反馈ID (e.g. 0x201)
 * @param hfdcan:            	该电机所属的FDCAN句柄 (e.g. &hfdcan1)
 * @param loc_kp:            	位置环 比例系数
 * @param loc_ki:            	位置环 积分系数
 * @param loc_kd:            	位置环 微分系数
 * @param spd_kp:            	速度环 比例系数
 * @param spd_ki:            	速度环 积分系数
 * @param spd_kd:            	速度环 比例系数
 * @param loc_out_limit_up:  	位置环输出上限 (速度值)
 * @param loc_out_limit_down: 位置环输出下限 (速度值)
 * @param spd_out_limit_up:  	速度环输出上限 (电流值)
 * @param spd_out_limit_down: 速度环输出下限 (电流值)
 */
static void dji_motor_configure(uint8_t index, const uint16_t id, FDCAN_HandleTypeDef* hfdcan, 
                                float loc_kp, float loc_ki, float loc_kd,
                                float spd_kp, float spd_ki, float spd_kd,
																float loc_out_limit_up, float loc_out_limit_down,
																float spd_out_limit_up, float spd_out_limit_down)
{
    if (index >= DJI_MOTOR_COUNT) return;

    DJI_Motor_Instance* motor = &dji_motors[index];

    // 配置基本信息
    motor->can_id = id;
    motor->hfdcan = hfdcan;
    motor->control_mode = MOTOR_MODE_POSITION; // 默认位置模式

    // 初始化PID控制器参数
	// TODO: 若对于不同电机要设置不同限幅，可修改函数将其作为参数一并传入 
    // 位置环 (输出限幅为速度)
    motor->loc_pid.Kp = loc_kp; motor->loc_pid.Ki = loc_ki; motor->loc_pid.Kd = loc_kd;
    motor->loc_pid.out_limit_up = loc_out_limit_up; motor->loc_pid.out_limit_down = loc_out_limit_down;

    // 速度环 (输出限幅为电流)
    motor->spd_pid.Kp = spd_kp; motor->spd_pid.Ki = spd_ki; motor->spd_pid.Kd = spd_kd;
    motor->spd_pid.out_limit_up = spd_out_limit_up; motor->spd_pid.out_limit_down = spd_out_limit_down;

    // 准备注册信息
    FDCAN_Dispatch_t dispatch_item;
    dispatch_item.id_type = FDCAN_STANDARD_ID;					// ID 类型
    dispatch_item.id = motor->can_id;										
    dispatch_item.instance_ptr = motor;									// 电机实例
    dispatch_item.handler = dji_motor_message_handler;	// 回调函数句柄

    // 调用bsp层的注册函数
    fdcan_bsp_register(&dispatch_item, hfdcan);
}


/**
 * @brief 解析来自电机电调的CAN报文数据
 * @param motor: 传回报文的电机实例
 * @param rx_data: 接收到的数据
 * @retval: None
 */
static void get_motor_measure(DJI_Motor_Instance* motor, uint8_t rx_data[8])
{
    motor->measure.last_angle = motor->measure.angle;
    motor->measure.angle = (uint16_t)(rx_data[0] << 8 | rx_data[1]);
    motor->measure.speed_rpm = (int16_t)(rx_data[2] << 8 | rx_data[3]);
    motor->measure.given_current = (int16_t)(rx_data[4] << 8 | rx_data[5]);
    motor->measure.temperate = rx_data[6];
}

/**
 * @brief  计算电机累计总角度 (支持多圈)
 * @param motor: 要计算的电机实例
 * @retval: None
 */
static void get_total_angle(DJI_Motor_Instance* motor)
{
		// 得到变化量
    int32_t delta = motor->measure.angle - motor->measure.last_angle;

    // 处理编码器值回绕
	  // 注意到 DJI 电机最高转速约为9000rpm，回报率为 1KHz，
		// 该回报率下两次接收 CAN 报文之间电机不可能转过一圈
    if (delta > (DJI_ENCODER_RESOLUTION / 2))
    {
        // 从 8191 -> 0, 反转
        delta -= DJI_ENCODER_RESOLUTION;
    }
    else if (delta < -(DJI_ENCODER_RESOLUTION / 2))
    {
        // 从 0 -> 8191, 正转
        delta += DJI_ENCODER_RESOLUTION;
    }

    motor->measure.total_angle += delta;
}

/**
 * @brief   打包并发送电机控制指令
 * @details 根据触发电机来决定发送哪个组的指令。
 *          如果传入的指针为NULL，则会向两个组都发送指令（用于stop_all等场景）
 * @param trigger_motor: 触发回调函数的电机实例
 */
static void dji_motor_send_commands(DJI_Motor_Instance* trigger_motor)
{
    FDCAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    int16_t out[DJI_MOTOR_COUNT];

    // 根据全局使能标志决定输出是PID计算值还是0
    for(int i=0; i < DJI_MOTOR_COUNT; i++)
    {
        out[i] = dji_motor_control_enabled ? (int16_t)dji_motors[i].spd_pid.now_out : 0;
    }

    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;
    
    // 判断是否需要发送第一组 (电机索引 0-3)
    if (trigger_motor == NULL || (trigger_motor >= &dji_motors[0] && trigger_motor <= &dji_motors[3]))
    {
        tx_header.Identifier = CAN_ID_FIRST_FOUR_MOTORS;
        tx_data[0] = (uint8_t)(out[0] >> 8); tx_data[1] = (uint8_t)out[0];
        tx_data[2] = (uint8_t)(out[1] >> 8); tx_data[3] = (uint8_t)out[1];
        tx_data[4] = (uint8_t)(out[2] >> 8); tx_data[5] = (uint8_t)out[2];
        tx_data[6] = (uint8_t)(out[3] >> 8); tx_data[7] = (uint8_t)out[3];

        // 根据 trigger_motor 动态选择FDCAN句柄
        // 注意这里假设前四个 ID 的电机被分配在一条 CAN 总线上，若后续存在问题可进行修改
        FDCAN_HandleTypeDef* hcan_to_use = (trigger_motor != NULL) ? trigger_motor->hfdcan : dji_motors[0].hfdcan;

        if (hcan_to_use  != NULL)
        {
            if (HAL_FDCAN_AddMessageToTxFifoQ(hcan_to_use, &tx_header, tx_data) != HAL_OK)
            {
                Error_Handler();
            }
        }
    }
    
    // 判断是否需要发送第二组 (电机索引 4-7)
    if (trigger_motor == NULL || (trigger_motor >= &dji_motors[4] && trigger_motor <= &dji_motors[7]))
    {
        tx_header.Identifier = CAN_ID_LAST_FOUR_MOTORS;
        tx_data[0] = (uint8_t)(out[4] >> 8); tx_data[1] = (uint8_t)out[4];
        tx_data[2] = (uint8_t)(out[5] >> 8); tx_data[3] = (uint8_t)out[5];
        tx_data[4] = (uint8_t)(out[6] >> 8); tx_data[5] = (uint8_t)out[6];
        tx_data[6] = (uint8_t)(out[7] >> 8); tx_data[7] = (uint8_t)out[7];

        // 根据 trigger_motor 动态选择FDCAN句柄
        // 注意这里假设后四个 ID 的电机被分配在另一条 CAN 总线上，若后续存在问题可进行修改
        FDCAN_HandleTypeDef* hcan_to_use = (trigger_motor != NULL) ? trigger_motor->hfdcan : dji_motors[4].hfdcan;

        if (hcan_to_use  != NULL)
        {
            if (HAL_FDCAN_AddMessageToTxFifoQ(hcan_to_use , &tx_header, tx_data) != HAL_OK)
            {
                Error_Handler();
            }
        }
    }
}

/**
 * @brief   进入临界区，保护共享数据
 * @details RTOS环境下调用FreeRTOS的API，裸机环境下直接关中断
 */
static void dji_critical_section_enter(void)
{
#ifdef USE_FREERTOS
    taskENTER_CRITICAL();
#else
    __disable_irq();
#endif
}

/**
 * @brief   退出临界区，恢复现场
 * @details RTOS环境下调用FreeRTOS的API，裸机环境下直接开中断
 */
static void dji_critical_section_exit(void)
{
#ifdef USE_FREERTOS
    taskEXIT_CRITICAL();
#else
    __enable_irq();
#endif
}
