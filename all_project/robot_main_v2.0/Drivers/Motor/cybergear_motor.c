/**
  ******************************************************************************
  * @file           : cybergear_motor.c
  * @brief          : 小米 CyberGear 电机控制驱动实现文件。
  * @author         : 秦泽宇 & Gemini
  * @date           : 2025-10-18
  * @version        : v1.2
  * @note           :
  * - v1.2 (2025-10-20): 新增对FreeRTOS的适配升级，使用条件编译与裸机区分。
  * - v1.1 (2025-10-19): 新增电机模式设置函数，新增速度/位置模式电流限制设置函数，新增
  *                      设置机械零位函数。
  ******************************************************************************
  * @attention
  *
  * [Module Design]
  * 1. 本模块旨在封装小米CyberGear电机的CAN通信协议，将复杂的报文构造与解析
  *    细节对上层应用透明化。
  * 2. 模块完全依赖于 `fdcan_bsp` 模块提供的底层CAN报文收发和中断分发服务。
  *    通过向 `fdcan_bsp` 注册一个带掩码的通用回调规则，能够异步接收和处理来自
  *    同一电机的多种反馈报文（如类型2的指令应答和类型17的参数查询回复）。
  * 3. 驱动内部实现了对电机多种控制模式的API封装，通过发送通信类型为18的
  *    参数写入指令或通信类型为1的运控指令来控制电机。
  * 4. 提供了主动参数请求机制，允许上层应用按需轮询电机状态。
  *
  * [Usage Flow]
  * 1. 在 `cybergear_motors_init()` 函数中，根据硬件连接，配置每个电机
  *    的CAN ID和所属的FDCAN通道。
  * 2. 在系统初始化代码中，继 `fdcan_bsp_init()` 之后，调用
  *    `cybergear_motors_init()` 来完成电机的注册。
  * 3. 在需要控制电机的地方，首先通过 `cybergear_motor_get_instance(index)`
  *    来获取指定电机的实例指针。
  * 4. 在下达具体控制指令前，【必须】先调用 `cybergear_motor_set_mode()` 将电机
  *    设置为目标模式（运控模式除外）。
  * 5. 在初始化或模式切换后，调用 `cybergear_motor_enable()` 来使能电机。根据需要，
  *    可调用 `set_pid_gains()` 或 `set_current_limit()` 进行参数配置。
  * 6. 在主循环或定时任务中，调用 `set_position()`, `set_speed()` 等函数下达
  *    控制指令，或调用 `request_parameter()` 请求数据更新。
  * 7. 电机的状态会在 `cybergear_message_handler()` 回调函数中被自动异步更新，
  *    上层应用可随时访问电机实例中的 `measure` 结构体来获取最新数据。
  * 
  * [FreeRTOS 线程安全支持]
  * 1. 在 FreeRTOS 等多任务环境下，不同的任务可能同时调用本驱动的API函数来发送CAN指令，
  *    这会造成对FDCAN硬件资源的并发访问，可能导致发送队列损坏或总线错误。
  * 2. 本驱动通过内部实现的一个互斥锁 (Mutex) 机制，对所有CAN报文发送操作进行保护，确保
  *    任何时刻只有一个任务可以访问CAN发送硬件，从而实现线程安全。
  * 3. 所有与FreeRTOS相关的代码均通过条件编译宏 `USE_FREERTOS` 进行隔离。
  *    - 当定义了 `USE_FREERTOS` 时，线程安全代码将被编译，驱动适用于FreeRTOS环境。
  *    - 当未定义 `USE_FREERTOS` 时，相关代码将被忽略，驱动可无缝用于裸机环境。
  * 4. 实现细节：
  *    a. `cybergear_lock_init()`: 在 `cybergear_motors_init()` 中自动调用，用于创建互斥锁。
  *    b. `cybergear_lock_tx()`: 在发送CAN报文前获取锁。
  *    c. `cybergear_unlock_tx()`: 在发送CAN报文后释放锁。
  *
  ******************************************************************************
  */

#include "cybergear_motor.h"
#include "fdcan_bsp.h" 
#include <string.h>

// 防止重定义 M_PI
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifdef USE_FREERTOS 
    #include "FreeRTOS.h"
    #include "semphr.h"
    
    // RTOS环境下：定义一个真实的互斥锁
    static SemaphoreHandle_t g_can_tx_mutex = NULL;
    
    // 定义锁成功的返回值 (基于FreeRTOS)
    #define CYBERGEAR_LOCK_SUCCESS  (pdTRUE)

#else // 裸机 (Bare-metal) 环境下
    
    // 定义锁成功的返回值 (裸机)
    #define CYBERGEAR_LOCK_SUCCESS  (1) 
    // 裸机环境下不需要全局变量

#endif



/* ------------------------- Static Variables ------------------------- */



// 全局电机实例数组
static CyberGear_Motor_Instance cybergear_motors[CYBERGEAR_MOTOR_COUNT];



/* ------------------------- Private Function Prototypes ------------------------- */



// 浮点数到无符号整数的转换辅助函数
static uint16_t float_to_uint(float x, float x_min, float x_max, int bits);

// 无符号整数到浮点数的转换辅助函数
static float uint16_to_float(uint16_t x, float x_min, float x_max, int bits);

// 单个电机配置与注册的私有辅助函数
static void cybergear_motor_configure(uint8_t index, const uint8_t id, FDCAN_HandleTypeDef* hfdcan);

// 发送CAN报文的底层辅助函数
static void cybergear_can_send_raw(CyberGear_Motor_Instance* motor, uint8_t comm_type, uint16_t data2, uint8_t target_id, uint8_t tx_data[8]);

// 使能电机
static void motor_enable(CyberGear_Motor_Instance* motor);

// 向电机写入一个浮点型(float)参数
static void cybergear_write_float_parameter(CyberGear_Motor_Instance* motor, uint16_t index, float value);

// 向电机写入一个单字节(uint8_t)参数
static void cybergear_write_u8_parameter(CyberGear_Motor_Instance* motor, uint16_t index, uint8_t value);

// 适配FreeRTOS的用于并发控制的私有函数
static void cybergear_lock_init(void);
static uint8_t cybergear_lock_tx(void); // 返回值用于判断是否成功
static void cybergear_unlock_tx(void);

/* ------------------------- Public Function Implementations ------------------------- */



/**
 * @brief   将一个给定的角度值(度)转换为弧度值(弧度)
 * @note    由于电机位置模式仅支持-720~720度范围内的位置控制，超出范围的值将被裁剪
 * @param   degree: 角度值(-720~720)
 * @retval  一个弧度值(float)
 */
float cybergear_motor_degree2rad(float degree)
{
    if (degree < -720.0f) degree = -720.0f;
    if (degree > 720.0f) degree = 720.0f;
    return degree * (M_PI / 180.0f);
}

/**
 * @brief   将一个给定的转速值(rpm)转换为角速度(rad/s)
 * @note    由于电机速度模式仅支持-30~30 rad/s范围内的速度控制，超出范围的值将被裁剪
 * @param   rpm: 转速(-296~296±10%)
 * @retval  一个角速度值(float)
 */
float cybergear_motor_rpm2rad(uint16_t rpm)
{
    float rad_per_sec = (float)rpm * (2.0f * M_PI / 60.0f);
    if (rad_per_sec < -30.0f) rad_per_sec = -30.0f;
    if (rad_per_sec > 30.0f) rad_per_sec = 30.0f;
    return rad_per_sec;
}

/**
 * @brief  初始化所有CyberGear电机实例
 */
void cybergear_motors_init(void)
{
    // 调用实际的锁初始化函数
    cybergear_lock_init();

    // --- 在这里集中配置所有电机 ---
    // 参数: 数组索引, 电机CAN_ID, 所属FDCAN句柄
    cybergear_motor_configure(0, 11, &hfdcan2); 
    // 如果有更多电机，请继续添加...
    // cybergear_motor_configure(1, 2, &hfdcan1);
}

/**
 * @brief  获取指定索引的电机实例指针
 * @param  motor_index: 电机索引 (0 to CYBERGEAR_MOTOR_COUNT-1)
 * @retval 指向电机实例的指针，如果索引无效则返回NULL
 */
CyberGear_Motor_Instance* cybergear_motor_get_instance(uint8_t motor_index)
{
    if (motor_index >= CYBERGEAR_MOTOR_COUNT)
    {
        return NULL;
    }
    return &cybergear_motors[motor_index];
}

/**
 * @brief  公开的电机使能函数
 * @param  motor: 指向要使能的电机实例
 */
void cybergear_motor_enable(CyberGear_Motor_Instance* motor)
{
    motor_enable(motor); // 调用 static motor_enable 函数
}

/**
 * @brief  停止电机运行 (发送通信类型4)
 * @param  motor: 指向要停止的电机实例
 */
void cybergear_motor_stop(CyberGear_Motor_Instance* motor)
{
    uint8_t tx_data[8] = {0};
    // 通信类型4，主机ID在data2字段，目标是电机ID
    cybergear_can_send_raw(motor, 4, MASTER_CAN_ID, motor->can_id, tx_data);
}

/**
 * @brief   设置电机的控制模式
 * @details 遵循说明书安全规范：先发送停止指令，再切换模式。
 *          注意：此函数执行后，电机将处于停止状态，需要重新调用 enable 才能运行。
 * @param  motor: 指向电机实例
 * @param  mode:  要设置的目标控制模式
 */
void cybergear_motor_set_mode(CyberGear_Motor_Instance* motor, CyberGear_Control_Mode_e mode)
{
    // 如果目标模式与当前模式相同，则无需操作
    if (motor->control_mode == mode)
    {
        return;
    }

    // 1. 先发送停止指令，确保安全
    cybergear_motor_stop(motor);
    // (可以根据需要加入一个小的延时, HAL_Delay(1), 以确保电机有足够时间响应)
    // HAL_Delay(1);

    // 2. 根据目标模式，发送参数写入指令
    cybergear_write_u8_parameter(motor, 0x7005, (uint8_t)mode);

    // 3. 更新主控侧的模式缓存
    motor->control_mode = mode;
}

/**
 * @brief   设置当前电机位置为机械零位 (掉电丢失)
 * @details 发送通信类型为6的指令
 * @param  motor: 指向电机实例
 */
void cybergear_motor_set_zero_position(CyberGear_Motor_Instance* motor)
{
    uint8_t tx_data[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    // 通信类型6，主机ID在data2字段，目标是电机ID
    cybergear_can_send_raw(motor, 6, MASTER_CAN_ID, motor->can_id, tx_data);
}

/**
 * @brief   设置电机到指定位置 (位置模式)
 * @details 内部会发送指令：设置目标位置
 * @note    调用此函数前，必须先通过 cybergear_motor_set_mode() 将电机设为位置模式。
 * @param   motor:       指向电机实例
 * @param   position:    目标位置 (rad)
 * @param   speed_limit: 到达目标位置过程中的最大速度限制 (rad/s)
 */
void cybergear_motor_set_position(CyberGear_Motor_Instance* motor, float position, float speed_limit)
{
    // 1. 安全检查：确保当前处于正确的模式
    if (motor->control_mode != MOTOR_CONTROL_MODE_POSITION)
    {
        return;
    }
    
    // 2. 设置位置模式下的速度限制
    cybergear_write_float_parameter(motor, 0x7017, speed_limit);
    
    // 3. 设置目标位置
    cybergear_write_float_parameter(motor, 0x7016, position);
}

/**
 * @brief  设置电机以指定速度运行 (速度模式)
 * @details 内部会发送指令：设置目标速度
 * @note   调用此函数前，必须先通过 cybergear_motor_set_mode() 将电机设为速度模式。
 * @param  motor: 指向电机实例
 * @param  speed: 目标速度 (rad/s), 范围 -30 ~ 30
 */
void cybergear_motor_set_speed(CyberGear_Motor_Instance* motor, float speed)
{
    // 1. 安全检查：确保当前处于正确的模式
    if (motor->control_mode != MOTOR_CONTROL_MODE_SPEED)
    {
        return;
    }
    
    // 2. 设置目标速度
    cybergear_write_float_parameter(motor, 0x700A, speed);
}

/**
 * @brief  设置电机输出指定的力矩 (电流模式)
 * @details 内部会发送指令：设置目标 Iq 电流
 * @note   调用此函数前，必须先通过 cybergear_motor_set_mode() 将电机设为力矩模式。
 * @param  motor:  指向电机实例
 * @param  torque: 目标力矩 (N.m)，该值会通过转矩常数近似转换为Iq电流
 */
void cybergear_motor_set_torque(CyberGear_Motor_Instance* motor, float torque)
{
    // 电机的转矩常数约为 0.87 N.m/Arms
    // 这是一个近似值，实际Iq到力矩的关系可能更复杂，但作为控制足够了
    const float TORQUE_CONSTANT = 0.87f;
    float iq_ref = torque / TORQUE_CONSTANT;

    // 1. 安全检查：确保当前处于正确的模式
    if (motor->control_mode != MOTOR_CONTROL_MODE_TORQUE)
    {
        return;
    }

    // 2. 设置目标Iq电流
    cybergear_write_float_parameter(motor, 0x7006, iq_ref);
}

/**
 * @brief  使用运控模式控制电机
 * @details 发送通信类型为1的单帧报文，同时设定所有5个控制参数。
 * @note   运控模式是电机的默认模式，且为直接动作指令，无需预设模式。
 * @param  motor:     指向电机实例
 * @param  position:  目标位置 (rad)
 * @param  speed:     目标速度 (rad/s)
 * @param  kp:        位置环比例增益 Kp (0~500)
 * @param  kd:        速度环阻尼系数 Kd (0~5)
 * @param  torque_ff: 前馈力矩 (N.m), 范围 -12 ~ 12
 */
void cybergear_motor_set_motion_control(CyberGear_Motor_Instance* motor, float position, float speed, float kp, float kd, float torque_ff)
{
    // 更新主控侧的模式缓存，以便其他函数能正确判断状态
    motor->control_mode = MOTOR_CONTROL_MODE_MOTION;

    // 1. 将前馈力矩打包到扩展ID的 data2 字段中
    uint16_t torque_uint = float_to_uint(torque_ff, T_MIN, T_MAX, 16);

    // 2. 将其他四个参数打包到8字节的数据区中
    uint8_t tx_data[8];
    
    uint16_t pos_uint = float_to_uint(position, P_MIN, P_MAX, 16);
    uint16_t spd_uint = float_to_uint(speed, V_MIN, V_MAX, 16);
    uint16_t kp_uint = float_to_uint(kp, KP_MIN, KP_MAX, 16);
    uint16_t kd_uint = float_to_uint(kd, KD_MIN, KD_MAX, 16);
    
    tx_data[0] = (uint8_t)(pos_uint >> 8);
    tx_data[1] = (uint8_t)(pos_uint);
    tx_data[2] = (uint8_t)(spd_uint >> 8);
    tx_data[3] = (uint8_t)(spd_uint);
    tx_data[4] = (uint8_t)(kp_uint >> 8);
    tx_data[5] = (uint8_t)(kp_uint);
    tx_data[6] = (uint8_t)(kd_uint >> 8);
    tx_data[7] = (uint8_t)(kd_uint);


    // 3. 调用底层发送函数
    // 通信类型1，打包后的力矩在data2字段，目标是电机ID
    cybergear_can_send_raw(motor, 1, torque_uint, motor->can_id, tx_data);
}

/**
 * @brief  设置电机内部的位置环和速度环PID增益
 * @details 将调试好的PID参数写入电机RAM (掉电丢失)
 * @param  motor:  指向电机实例
 * @param  loc_kp: 位置环 Kp (来自参数表 0x2016)
 * @param  spd_kp: 速度环 Kp (来自参数表 0x2014)
 * @param  spd_ki: 速度环 Ki (来自参数表 0x2015)
 */
void cybergear_motor_set_pid_gains(CyberGear_Motor_Instance* motor, float loc_kp, float spd_kp, float spd_ki)
{
    // 写入位置环 Kp (地址 0x2016)
    cybergear_write_float_parameter(motor, 0x2016, loc_kp);

    // 写入速度环 Kp (地址 0x2014)
    cybergear_write_float_parameter(motor, 0x2014, spd_kp);

    // 写入速度环 Ki (地址 0x2015)
    cybergear_write_float_parameter(motor, 0x2015, spd_ki);
}

/**
 * @brief   设置电机在速度/位置模式下的最大电流限制
 * @details 将值写入参数地址 0x7018 (limit_cur)。该值掉电丢失。
 * @param   motor:         指向电机实例
 * @param   current_limit: 最大电流值 (A)，范围 0~23A
 */
void cybergear_motor_set_current_limit(CyberGear_Motor_Instance* motor, float current_limit)
{
    // 设置一个安全钳位
    if (current_limit < 0) current_limit = 0;
    if (current_limit > 23.0f) current_limit = 23.0f;

    cybergear_write_float_parameter(motor, 0x7018, current_limit);
}

/**
 * @brief  主动请求读取电机的单个参数
 * @details 发送通信类型为17的指令。电机将会回复一帧同样类型为17的报文，
 *          其中包含了所请求参数的值。需要在FDCAN的接收端处理该回复。
 * @param  motor:           指向电机实例
 * @param  parameter_index: 想读取的参数的地址，来自说明书4.1.11节。
 *          已封装了部分常用参数为枚举 CyberGear_Param_Index_e
 *          - PARAM_MECH_POS = 0x7019: 负载端计圈机械角度 (rad)
 *          - PARAM_IQ_FILTER = 0x701A: iq 滤波值 (A)
 *          - PARAM_MECH_VEL = 0x701B: 负载端转速 (rad/s)
 *          - PARAM_VBUS = 0x701C: 母线电压 (V)
 *          - PARAM_ROTATION = 0x701D: 圈数, int16_t
 */
void cybergear_motor_request_parameter(CyberGear_Motor_Instance* motor, CyberGear_Param_Index_e param_index)
{
    uint8_t tx_data[8] = {0};
    uint16_t index_val = param_index; // 枚举值可以直接赋给uint16_t

    memcpy(&tx_data[0], &index_val, sizeof(uint16_t));
    
    cybergear_can_send_raw(motor, 17, MASTER_CAN_ID, motor->can_id, tx_data);
}



/* ------------------------- Private Function Implementations ------------------------- */



/**
 * @brief  CyberGear电机报文的统一处理函数 (回调函数)
 * @details 该函数会被 `fdcan_bsp` 模块在收到匹配的CAN ID时调用。
 * @param  instance:  指向触发回调的电机实例的void指针
 * @param  rx_header: 包含扩展ID等信息的FDCAN报文头
 * @param  rx_data:   接收到的8字节CAN数据
 */
void cybergear_message_handler(void* instance, FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[8])
{
    if (instance == NULL || rx_header == NULL) return;
    
    // 将接收到的实例转换回电机实例
    CyberGear_Motor_Instance* motor = (CyberGear_Motor_Instance*)instance;

    // 从扩展ID中解析出通信类型
    uint8_t comm_type = (rx_header->Identifier >> 24) & 0x1F;

    // 根据通信类型，分发到不同的处理逻辑
    switch(comm_type) {
        case 2: // 类型2: 电机通用反馈报文
        {
            // 解析模式位和故障位
            motor->measure.mode = (CyberGear_Motor_Mode_e)((rx_header->Identifier >> 22) & 0x03);
            motor->measure.fault = (uint8_t)((rx_header->Identifier >> 16) & 0x3F);

            // 解析数据位
            uint16_t raw_angle = (rx_data[0] << 8) | rx_data[1];
            uint16_t raw_speed = (rx_data[2] << 8) | rx_data[3];
            uint16_t raw_torque = (rx_data[4] << 8) | rx_data[5];
            uint16_t raw_temp = (rx_data[6] << 8) | rx_data[7];

            // 处理数据，转化为直观的量纲
            motor->measure.angle = uint16_to_float(raw_angle, P_MIN, P_MAX, 16);
            motor->measure.speed = uint16_to_float(raw_speed, V_MIN, V_MAX, 16);
            motor->measure.torque = uint16_to_float(raw_torque, T_MIN, T_MAX, 16);
            motor->measure.temperature = (float)raw_temp * 0.1f;
            break;
        }
        case 17: // 类型17: 单个参数读取的回复报文
        {
            // 解析回复的参数地址和值
            uint16_t param_index;
            // 使用小端解析器
            memcpy(&param_index, &rx_data[0], sizeof(uint16_t));

            // 根据地址，将值存入对应的结构体成员
            // 注意这里仅根据枚举CyberGear_Param_Index_e中封装的几个参数做解析
            // 如果需要其他参数解析，可自行拓展
            switch(param_index) {
                case PARAM_MECH_POS: // 负载端计圈机械角度 (rad), float
                {
                    float val;
                    memcpy(&val, &rx_data[4], sizeof(float));
                    motor->measure.angle = val; // 更新与类型2报文相同的字段
                    break;
                }
                case PARAM_MECH_VEL: // 负载端转速 (rad/s), float
                {
                    float val;
                    memcpy(&val, &rx_data[4], sizeof(float));
                    motor->measure.speed = val; // 更新与类型2报文相同的字段
                    break;
                }
                case PARAM_VBUS: // 母线电压 (V), float
                {
                    float val;
                    memcpy(&val, &rx_data[4], sizeof(float));
                    motor->measure.vbus = val; // 更新类型17报文传回的字段
                    break;
                }
                case PARAM_ROTATION: // 圈数, int16_t
                {
                    int16_t val;
                    // 注意圈数是int16，占2字节
                    memcpy(&val, &rx_data[4], sizeof(int16_t));
                    motor->measure.rotation = val; // 更新类型17报文传回的字段
                    break;
                }
                // 对于其他参数，可以暂时不处理或按需添加
                default: break; 
            }

            break;
        }
        // 对于其他通信类型，暂时不关心
        // 后续有需要时可添加相关处理逻辑
        default: break;
    }
}

/**
 * @brief  将一个浮点数转换为指定位数的无符号整数
 * @param  x:      输入的浮点数
 * @param  x_min:  浮点数范围的下限
 * @param  x_max:  浮点数范围的上限
 * @param  bits:   目标整数的位数 (通常是16)
 * @retval 转换后的无符号整数
 */
static uint16_t float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    // 限制输入值在范围内
    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;
    // 执行转换
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief  将一个无符号整数根据范围转换为浮点数
 * @param  x:      输入的16位无符号整数
 * @param  x_min:  浮点数范围的下限
 * @param  x_max:  浮点数范围的上限
 * @param  bits:   输入整数的位数 (通常是16)
 * @retval 转换后的浮点数
 */
static float uint16_to_float(uint16_t x, float x_min, float x_max, int bits)
{
    uint32_t span = (1 << bits) - 1;
    float offset = x_max - x_min;
    return ((float)x * offset) / ((float)span) + x_min;
}

/**
 * @brief  配置单个电机实例并注册到FDCAN分发器
 * @param  index:  电机在 cybergear_motors 数组中的索引
 * @param  id:     电机的CAN ID (0-127)
 * @param  hfdcan: 该电机所属的FDCAN句柄 (e.g. &hfdcan1)
 */
static void cybergear_motor_configure(uint8_t index, const uint8_t id, FDCAN_HandleTypeDef* hfdcan)
{
    if (index >= CYBERGEAR_MOTOR_COUNT) return;

    CyberGear_Motor_Instance* motor = &cybergear_motors[index];

    // 1. 配置基本信息
    motor->can_id = id;
    motor->hfdcan = hfdcan;
    motor->control_mode = MOTOR_CONTROL_MODE_UNSET; // 初始化控制模式为未设定
    
    // 2. 准备注册信息
    FDCAN_Dispatch_t dispatch_item;
    
    // 小米电机使用29位扩展帧
    dispatch_item.id_type = FDCAN_EXTENDED_ID; 
    
    // 【使用掩码机制进行注册】
    // 1. 我们要匹配的ID模板：只设置我们关心的、固定的部分
    // 格式: comm_type (5bit) | source_id (16bit) | target_id (8bit)
    // 为了同时实现应答式反馈帧和轮询式反馈帧，只关心报文的发送方和接收方
    dispatch_item.id = (uint32_t)(id << 8) |      // 源地址必须是电机自己的ID
                       (uint32_t)(MASTER_CAN_ID); // 目标地址必须是主机ID
    
    // 2. 设置掩码：告诉分发器，我们只比较ID中的源地址和目标地址部分
    //    我们不关心通信类型(bit28-24)和中间的数据部分(bit23-16)
    dispatch_item.mask = (uint32_t)(0x0000FF00) |  // 比较 bit15-8 (源电机ID)
                         (uint32_t)(0x000000FF);   // 比较 bit7-0  (目标主机ID)
    
    dispatch_item.instance_ptr = motor;                 // 传递电机实例指针
    dispatch_item.handler = cybergear_message_handler;  // 注册回调函数

    // 3. 调用bsp层的注册函数
    fdcan_bsp_register(&dispatch_item, hfdcan);
}

/**
 * @brief  CyberGear电机CAN指令的底层发送函数
 * @param  motor:       指向电机实例，用于获取FDCAN句柄
 * @param  comm_type:   通信类型 (编码于ExtID bit28-24)
 * @param  data2:       数据区2 (编码于ExtID bit23-8, 通常是主机ID或参数)
 * @param  target_id:   目标电机ID (编码于ExtID bit7-0)
 * @param  tx_data:     指向8字节数据负载的指针
 */
static void cybergear_can_send_raw(CyberGear_Motor_Instance* motor, 
                                   uint8_t comm_type, 
                                   uint16_t data2, 
                                   uint8_t target_id, 
                                   uint8_t tx_data[8])
{
    if (motor == NULL || motor->hfdcan == NULL) return;

    // ---==== 保护区开始 ====---
    // 调用实际的锁函数
    if (cybergear_lock_tx() != CYBERGEAR_LOCK_SUCCESS)
    {
        // 获取锁超时 (仅在RTOS下可能发生)
        // 注意此处超时时间为 10 ms，具体值可在`cybergear_lock_tx`函数中修改
        // TODO: 处理发送超时错误
        return; 
    }

    FDCAN_TxHeaderTypeDef tx_header;
    
    // 1. 构造29位扩展ID
    // 格式:  comm_type (5bit) | data2 (16bit) | target_id (8bit) 
    tx_header.Identifier = (uint32_t)(comm_type << 24) | 
                           (uint32_t)(data2 << 8)    | 
                           (uint32_t)(target_id);
    
    // 2. 配置报文头
    tx_header.IdType = FDCAN_EXTENDED_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN; // 使用传统CAN 2.0格式
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    // 3. 发送报文
    if (HAL_FDCAN_AddMessageToTxFifoQ(motor->hfdcan, &tx_header, tx_data) != HAL_OK)
    {
        // TODO: 处理发送错误 (例如记录日志)
        Error_Handler();
    }

    // ---==== 保护区结束 ====---
    // 调用实际的释放锁函数
    cybergear_unlock_tx();
}

/**
 * @brief  使能电机
 * @param  motor: 指向要使能的电机实例
 */
static void motor_enable(CyberGear_Motor_Instance* motor)
{
    uint8_t tx_data[8] = {0}; // 数据区为空
    // 通信类型3，主机ID在data2字段，目标是电机ID
    cybergear_can_send_raw(motor, 3, MASTER_CAN_ID, motor->can_id, tx_data);
}

/**
 * @brief  向电机写入一个浮点型参数
 * @param  motor: 指向电机实例
 * @param  index: 要写入的参数地址 (e.g., 0x7006 for iq_ref)
 * @param  value: 要写入的浮点数值
 */
static void cybergear_write_float_parameter(CyberGear_Motor_Instance* motor, uint16_t index, float value)
{
    uint8_t tx_data[8] = {0};
    
    // 根据协议，参数地址放在 Byte0-1
    memcpy(&tx_data[0], &index, sizeof(uint16_t));
    
    // 浮点数值放在 Byte4-7
    memcpy(&tx_data[4], &value, sizeof(float));

    // 通信类型18，主机ID在data2字段，目标是电机ID
    cybergear_can_send_raw(motor, 18, MASTER_CAN_ID, motor->can_id, tx_data);
}

/**
 * @brief  向电机写入一个单字节(uint8_t)参数
 * @param  motor: 指向电机实例
 * @param  index: 要写入的参数地址 (e.g., 0x7005 for run_mode)
 * @param  value: 要写入的单字节数值
 */
static void cybergear_write_u8_parameter(CyberGear_Motor_Instance* motor, uint16_t index, uint8_t value)
{
    uint8_t tx_data[8] = {0};
    
    // 参数地址放在 Byte0-1
    memcpy(&tx_data[0], &index, sizeof(uint16_t));
    
    // 单字节值放在 Byte4
    tx_data[4] = value;

    // 通信类型18，主机ID在data2字段，目标是电机ID
    cybergear_can_send_raw(motor, 18, MASTER_CAN_ID, motor->can_id, tx_data);
}

/**
 * @brief   初始化CAN发送锁
 * @details RTOS环境下创建Mutex，裸机环境下为空操作
 */
static void cybergear_lock_init(void)
{
#ifdef USE_FREERTOS
    if (g_can_tx_mutex == NULL) // 防止重复创建
    {
        g_can_tx_mutex = xSemaphoreCreateMutex();
    }
#else
    // 裸机环境下，此函数体为空
    (void)0;    // 防止某些编译器报 "empty function body" 警告
#endif
}

/**
 * @brief   获取CAN发送锁
 * @details RTOS环境下调用 xSemaphoreTake，裸机环境下直接返回成功
 * @retval  CYBERGEAR_LOCK_SUCCESS (1) 表示成功, 0 表示失败/超时
 */
static uint8_t cybergear_lock_tx(void)
{
#ifdef USE_FREERTOS
    if (g_can_tx_mutex == NULL) return 0; // 锁未初始化，返回失败
    
    // xSemaphoreTake 成功返回 pdTRUE (1), 失败返回 pdFALSE (0)
    // 直接返回结果 (转换为 uint8_t)
    return (uint8_t)xSemaphoreTake(g_can_tx_mutex, pdMS_TO_TICKS(10));
#else
    // 裸机环境下，总是返回成功
    return CYBERGEAR_LOCK_SUCCESS;
#endif
}

/**
 * @brief   释放CAN发送锁
 * @details RTOS环境下调用 xSemaphoreGive，裸机环境下为空操作
 */
static void cybergear_unlock_tx(void)
{
#ifdef USE_FREERTOS
    if (g_can_tx_mutex != NULL)
    {
        xSemaphoreGive(g_can_tx_mutex);
    }
#else
    // 裸机环境下，此函数体为空
    (void)0; // 防止警告
#endif
}
