#include "actuator_control.h"

#define buf_size 256               // UART接收缓冲区大小

static SemaphoreHandle_t mi = NULL;

// 电机控制参数配置
#define POSITION_TOLERANCE  50       // DJI电机位置容差（编码器值）
#define CHASSIS_TOLERANCE   0.1f    // 底盘电机位置容差（rad）
#define WAIT_TIMEOUT_MS     5000     // 电机到位等待超时时间（ms）
#define SUCTION_DELAY_MS    500      // 吸盘动作延迟时间（ms，确保机械动作完成）

// 电机实例全局指针（供所有控制函数访问）
static CyberGear_Motor_Instance *mi_motor = NULL;         // 底盘（CyberGear）电机实例
static DJI_Motor_Instance *dji_motor_zhuzhou = NULL;     // 主轴（DJI）电机实例
static DJI_Motor_Instance *dji_motor_dabi = NULL;        // 大臂（DJI）电机实例
static DJI_Motor_Instance *dji_motor_xiaobi = NULL;      // 小臂（DJI）电机实例
static float target_pos = 0.0f;                          // 底盘目标位置（rad，供监测任务读取）

/**
 * @brief  底盘电机（CyberGear）初始化函数
 * @details 完成底盘电机的实例获取、模式配置、参数设置和使能操作
 *          步骤：1. 获取电机实例 2. 停止电机 3. 设置位置控制模式 4. 配置电流限制
 *               5. 使能电机 6. 请求机械位置参数（确保初始化后位置数据有效）
 * @param  无
 * @retval 无
 */
void chassis_init(void)
{
  // 获取底盘电机实例（索引0对应配置的CyberGear电机）
  mi_motor = cybergear_motor_get_instance(0);
	
	vTaskDelay(pdMS_TO_TICKS(10));
	cybergear_motor_stop(mi_motor);
	vTaskDelay(pdMS_TO_TICKS(10));
	cybergear_motor_set_mode(mi_motor, MOTOR_CONTROL_MODE_POSITION);
	vTaskDelay(pdMS_TO_TICKS(10));
	// 电流限幅（可选）
	//cybergear_motor_set_current_limit(mi_motor, 15.0f);
	vTaskDelay(pdMS_TO_TICKS(10));
	cybergear_motor_set_zero_position(mi_motor);
	vTaskDelay(pdMS_TO_TICKS(10));
	cybergear_motor_enable(mi_motor);
	vTaskDelay(pdMS_TO_TICKS(10));
	trajectory_planner_start(mi_motor);  // 等待位置数据反馈
}

/**
 * @brief  底盘角度控制函数
 * @details 将输入的角度（度）转换为弧度，发送位置指令给CyberGear电机，并同步目标位置供监测任务使用
 * @param  Angle: 底盘目标角度（度），范围需符合CyberGear电机要求（-720~720度）
 * @retval 无
 */
void chassis_control(float Angle)
{
    if (mi_motor == NULL) return;  // 电机实例未初始化，直接返回

    // 1. 角度单位转换：度 → 弧度（调用CyberGear驱动的标准转换函数，自动裁剪超范围值）
    float target_rad = cybergear_motor_degree2rad(Angle);

    // 2. 发送位置控制指令：目标弧度、速度限制（50rpm转换为rad/s）
    trajectory_planner_set_target(target_rad);

    // 3. 同步目标位置到全局变量，供ChassisMonitor任务读取（关键：确保监测任务获取正确目标值）
    target_pos = target_rad;
}

/**
 * @brief  DJI电机（主轴、大臂、小臂）初始化函数
 * @details 获取各个功能电机的实例指针，为后续控制做准备
 * @param  无
 * @retval 无
 */
void dji_init(void)
{
    dji_motor_zhuzhou = dji_motor_get_instance(2);  // 获取主轴电机实例（索引2）
    dji_motor_dabi = dji_motor_get_instance(1);     // 获取大臂电机实例（索引1）
    dji_motor_xiaobi = dji_motor_get_instance(0);   // 获取小臂电机实例（索引0）
}

/**
 * @brief  主轴高度控制函数
 * @details 将输入的高度值转换为电机编码器角度，发送位置控制指令
 * @note   高度与编码器角度的换算关系（height * 30）需根据实际机械结构调试确认
 * @param  height: 主轴目标高度（自定义单位，需与机械设计匹配）
 * @retval 无
 */
void zhuzhou_control(float height)
{
		float angle = ((620.0f - height) / 100.6f * 360.0f * 19.0f);
    dji_motor_set_location(dji_motor_zhuzhou, dji_degree2encoder(angle));  // 发送目标位置指令
}

/**
 * @brief  小臂角度控制函数
 * @details 直接接收角度指令，发送给小臂电机
 * @param  angle: 小臂目标角度（编码器值，需根据电机特性配置）
 * @retval 无
 */
void xiaobi_control(float angle)
{
    dji_motor_set_location(dji_motor_xiaobi, dji_degree2encoder(-angle) * 36);  // 发送小臂目标角度指令
}

/**
 * @brief  大臂长度控制函数
 * @details 将输入的长度值转换为电机编码器角度，发送位置控制指令
 * @note   长度与编码器角度的换算关系（length * 60）需根据实际机械结构调试确认
 * @param  length: 大臂目标长度（自定义单位，需与机械设计匹配）
 * @retval 无
 */
void dabi_control(float length)
{
		float target = -((length - 155.0f) / 76.5f * 360.0f * 19.0f);
    int32_t angle = dji_degree2encoder(target);  // 长度→编码器角度换算（待调试确认）
    dji_motor_set_location(dji_motor_dabi, angle);  // 发送目标位置指令
}




/**
 * @brief  吸盘控制函数
 * @details 通过控制GPIO引脚电平实现吸盘的吸合与释放
 * @param  open: 控制指令（1=吸合，0=释放）
 * @retval 无
 */
void xipan_control(int open)
{
    if(open == 1)
    {
        HAL_GPIO_WritePin(Switch_GPIO_Port, Switch_Pin, GPIO_PIN_SET);  // 引脚置高，吸盘吸合
    }
    else
    {
        HAL_GPIO_WritePin(Switch_GPIO_Port, Switch_Pin, GPIO_PIN_RESET); // 引脚置低，吸盘释放
    }
}

/**
 * @brief  检查所有电机是否到位
 * @retval bool: true-所有电机到位，false-有电机未到位
 */
bool all_motors_in_position(void)
{
    // 检查底盘电机
    if (mi_motor != NULL) {
        if (fabsf(mi_motor->measure.angle - target_pos) > CHASSIS_TOLERANCE) {
            return false;
        }
    }
    
    // 检查DJI电机
    if (dji_motor_dabi != NULL) {
        int32_t error = abs(dji_motor_dabi->measure.total_angle - dji_motor_dabi->target_loc);
        if (error > POSITION_TOLERANCE) return false;
    }
    
    if (dji_motor_xiaobi != NULL) {
        int32_t error = abs(dji_motor_xiaobi->measure.total_angle - dji_motor_xiaobi->target_loc);
        if (error > POSITION_TOLERANCE) return false;
    }
    
    if (dji_motor_zhuzhou != NULL) {
        int32_t error = abs(dji_motor_zhuzhou->measure.total_angle - dji_motor_zhuzhou->target_loc);
        if (error > POSITION_TOLERANCE) return false;
    }
    
    return true;
}

