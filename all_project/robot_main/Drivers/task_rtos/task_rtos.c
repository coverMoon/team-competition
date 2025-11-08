#include "task_rtos.h" 
#include "actuator_control.h"	// 包含电机控制层
#include "stdio.h"

// 参数测试结构体
extern TestPosition_t test;

/* ------------------------- 任务状态定义 ------------------------- */
#define TASK_NONE 0 // 无任务状态
#define TASK_1 1    // 任务1：箱子堆叠流程
#define TASK_2 2    // 任务2：定点放置
#define TASK_3 3    // 任务3：随机放置

/* ------------------------- 任务状态标志 ------------------------- */

// 任务切换变量 (在 app_freertos.c 中定义，由UART中断修改)
extern volatile int16_t sys;
// 任务运行标志，防止多任务并发
static volatile bool is_task_running = false;

/* ------------------------- 任务二 目标区域定义 ------------------------- */
// (补充 .h 文件中未定义的 高度 和 类型 信息)
typedef struct {
    PolarCoord_t pos;    // 坐标 (来自 .h 文件)
    float height_offset; // 放置的目标高度 (mm)
    int type;            // 区域类型 (1, 2, 3)
} BoxZone_t;


static const BoxZone_t TASK2_ZONES[] = {
    {{350.0f, 0.0f},155.0f, 1},      // 一区：径向350mm、角度0°，放置高度155mm
    {{350.0f, 45.0f}, 155.0f, 1},    // 一区：径向350mm、角度45°，放置高度155mm
    {{350.0f, -45.0f}, 155.0f, 1},   // 一区：径向350mm、角度-45°，放置高度155mm
    {{425.0f, 30.0f}, 390.0f, 2},    // 二区：径向600mm、角度30°，放置高度390mm
    {{425.0f, -30.0f}, 390.0f, 2},   // 二区：径向600mm、角度-30°，放置高度390mm
    {{425.0f, 0.0f}, 260.0f, 3},     // 三区：径向550mm、角度0°，放置高度260mm
    {{425.0f, 60.0f}, 260.0f, 3},    // 三区：径向550mm、角度60°，放置高度260mm
    {{425.0f, -60.0f}, 260.0f, 3}    // 三区：径向550mm、角度-60°，放置高度260mm
};



/* ------------------------- 私有函数声明 ------------------------- */

// 任务实现
static void task1_rtos(void);
static void task2_rtos(void);
static void task3_rtos(void);

// 运动学封装
static void move_to_position(float height, float radius, float chassis_angle,float suction_angle);
static void pickup_box(void);
static void place_box(void);


/* ------------------------- 任务函数实现 ------------------------- */

/**
 * @brief  主任务：任务调度核心
 * @details 初始化电机，循环检测sys变量，根据sys值切换执行不同任务
 * 通过is_task_running标志防止任务并发执行
 */
void Task0(void *argument)
{
    // 初始化硬件抽象层
    chassis_init(); // 初始化底盘电机 (小米电机)
    dji_init();     // 初始化DJI电机 (3508*2, 2006*1)

    /* 无限循环：任务调度逻辑 */
    for (;;)
    {
			move_to_position(test.height,test.radius,test.chassis_angle ,test.suction_angle);
        // 仅当无任务运行且sys指定有效任务时，启动新任务
        if (!is_task_running && sys != TASK_NONE)
        {
            is_task_running = true; // 标记任务开始运行
            printf("Starting Task %d\r\n", sys);

            switch (sys)
            {
            case TASK_1:
                task1_rtos();
                break;
            case TASK_2:
                task2_rtos();
                break;
            case TASK_3:
                task3_rtos();
                break;
            default:
                printf("Invalid task number: %d\r\n", sys);
                break;
            }

            // 任务执行完成，重置状态
            printf("Task %d Finished\r\n", sys);
            is_task_running = false;
            sys = TASK_NONE; // 复位sys，等待下一次任务指令
        }

        osDelay(20); // 降低调度器轮询频率
    }
}

/**
 * @brief 任务一：箱子堆叠
 * @details 抓取三个固定位置的箱子，堆叠到一个固定位置
 */
static void task1_rtos(void)
{
    /* * --- 运动学代码 ---
     * * 示例逻辑：
     * 1. 移动到 TASK1_BOX_POS_1 上方安全高度
     * 2. 下降到抓取高度
     * 3. pickup_box()
     * 4. 抬升到安全高度
     * 5. 移动到 TASK1_TARGET_POS 上方 (高度 0 + 安全裕量)
     * 6. 下降到放置高度 (高度 0)
     * 7. place_box()
     * 8. 抬升到安全高度
     * 9. 移动到 TASK1_BOX_POS_2 ...
     * 10. ...
     * 11. 移动到 TASK1_TARGET_POS 上方 (高度 1*SMALL_BOX_HEIGHT + 安全裕量)
     * 12. 下降到放置高度 (高度 1*SMALL_BOX_HEIGHT)
     * 13. ...
     * 14. 重复抓取 TASK1_BOX_POS_3 并放置在 (高度 2*SMALL_BOX_HEIGHT)
     */
 for (int i = 0; i < 3; i++) 
    {
			// 3. 移动到目标位置安全高度
			move_to_position((i + 1) * SMALL_BOX_HEIGHT + 20.0f,task1_boxes[i].radius,task1_boxes[i].angle, 0.0f);
			// 1. 移动到抓取位置安全高度
			move_to_position(SMALL_BOX_HEIGHT + 15.0f,task1_boxes[i].radius, task1_boxes[i].angle, 0.0f);
			// 2. 下降到抓取高度并抓取
			move_to_position(SMALL_BOX_HEIGHT -13.0f,task1_boxes[i].radius, task1_boxes[i].angle, 0.0f);
			pickup_box();
			move_to_position(3* SMALL_BOX_HEIGHT + 20.0f ,task1_boxes[i].radius, task1_boxes[i].angle, 0.0f);
			// 3. 移动到目标位置安全高度
			move_to_position((i + 1) * SMALL_BOX_HEIGHT + 20.0f,TASK1_TARGET_POS.radius,TASK1_TARGET_POS.angle, 0.0f);
			// 4. 下降到放置高度并放置
			move_to_position((i + 1) * SMALL_BOX_HEIGHT+10.0f,TASK1_TARGET_POS.radius,TASK1_TARGET_POS.angle,0.0f);
			place_box();
    }
    printf("Task 1 logic not implemented.\r\n");
}

/**(i + 1) * SMALL_BOX_HEIGHT + 20.0f
 * @brief 任务二：定点放置
 * @details 从固定位置抓取4个小纸箱，放置到8个大纸箱中的任意4个
 */
static void task2_rtos(void)
{
    // 策略：选择要放置的4个目标区域的索引 (0-7)
    int chosen_target[4] = {3,4,5,6};

    for (int i = 0; i < 4; i++)
    {
        // 1. 计算抓取高度
        // 抓取第 i 个箱子 (从0开始)，每个高 SMALL_BOX_HEIGHT
        // 注意：从上往下抓取 (第0个在最上)
        float pickup_height = (4 - i) * SMALL_BOX_HEIGHT + 15.0f;		// 留够余量
			  // 1. 移动到抓取位置上方安全高度（随层数递增）
        move_to_position(pickup_height,TASK2_PICKUP_POS.radius,TASK2_PICKUP_POS.angle,0.0f);
        // 2. 下降到抓取高度，吸起箱子
        move_to_position((4 - i) * SMALL_BOX_HEIGHT,TASK2_PICKUP_POS.radius,TASK2_PICKUP_POS.angle,0.0f);
        pickup_box();
        // 3. 抬升20mm，防碰撞
        move_to_position(pickup_height, TASK2_PICKUP_POS.radius,TASK2_PICKUP_POS.angle,0.0f);
        // 3. 获取目标位置信息
        int target_index = chosen_target[i];
        const BoxZone_t* target_zone = &TASK2_ZONES[target_index];

        // 4. 放置
        /* * --- 运动学代码 (放置) ---
         * * 1. 分析 target_zone->type (1, 2, 3)
         * 2. 根据不同类型，计算真实的 move_to_position 参数
         * 		(例如二区和三区需要调整小臂角度和坐标)
         * 3. 移动到目标点 (target_zone->pos.radius, target_zone->pos.angle, target_zone->height_offset)
         * 4. place_box()
         * 5. 回到安全位置
         */


        if (target_zone->type == 1) 
				{
            // 一区放置逻辑
            printf("Placing in Zone 1\r\n");
        } 
				else if (target_zone->type == 2) 
				{
            // 二区放置逻辑
					  // 第一步：预定位（径向减150mm，防碰撞）
            move_to_position(target_zone->height_offset ,target_zone->pos.radius - 150.0f,target_zone->pos.angle,90.0f);
            // 第二步：精准定位到目标位置
            move_to_position(target_zone->height_offset ,target_zone->pos.radius,target_zone->pos.angle,90.0f);
            place_box();
            // 抬升安全高度
            move_to_position(target_zone->height_offset,target_zone->pos.radius - 150.0f ,target_zone->pos.angle,90.0f);
            printf("Placing in Zone 2\r\n");
        } 
				else if (target_zone->type == 3) 
				{
            // 三区放置逻辑
					  move_to_position(target_zone->height_offset,target_zone->pos.radius - 180.0f,target_zone->pos.angle,75.0f);
					  move_to_position(target_zone->height_offset,target_zone->pos.radius,target_zone->pos.angle,75.0f);
            place_box();
            // 放置后抬升20mm
            move_to_position(target_zone->height_offset,target_zone->pos.radius - 180.0f,target_zone->pos.angle,75.0f);
            printf("Placing in Zone 3\r\n");
        }
    }
}

/**
 * @brief 任务三：随机放置
 * @details 抓取3个小纸箱，放置到3个随机位置
 */
static void task3_rtos(void)
{
    Task3Position_t random_positions[3];
    uint8_t received_count = 0;
    osStatus_t status;

    printf("Task 3: Waiting for 3 positions...\r\n");

    // 1. 阻塞式接收 3 个随机坐标
    //    使用 osWaitForever 确保任务在收到所有数据前不会执行
    while (received_count < 3)
    {
        // 从队列中获取一个 Task3Position_t (大小为 8 字节)
        status = osMessageQueueGet(task3positionHandle, &random_positions[received_count], NULL, osWaitForever);
        
        if (status == osOK)
        {
            received_count++;
            printf("Received position %d: R=%.1f, A=%.1f\r\n",
                   received_count,
                   random_positions[received_count - 1].radius,
                   random_positions[received_count - 1].angle);
        }
    }
    
    printf("All 3 positions received. Starting movement.\r\n");

    // 2. 抓取并放置
    // 抓取位置同任务二
    for (int i = 0; i < 3; i++)
    {
        // 1. 计算抓取高度 (从上往下抓)

        // 2. 抓取
        /* * --- 运动学代码 (抓取) ---
         * * 1. 移动到 TASK2_PICKUP_POS (pickup_height + 安全裕量)
         * 2. 移动到 TASK2_PICKUP_POS (pickup_height)
         * 3. pickup_box()
         * 4. 抬升到安全高度
         */

        // 3. 放置
        /* * --- 运动学代码 (放置) ---
				 * 1. 规则说明：大纸箱摆放为随机坐标
         * 2. 计算放置高度：往大纸箱里放小纸箱
         *
         * 3. 移动到 random_positions[i].radius, random_positions[i].angle, 0.0f
         * 4. place_box()
         * 5. 回到安全位置
         */
			// 1. 移动到抓取位置上方安全高度
        move_to_position(i * SMALL_BOX_HEIGHT + 20.0f,TASK2_PICKUP_POS.radius,TASK2_PICKUP_POS.angle,0.0f);
        
        // 2. 下降到抓取高度，吸起箱子
        move_to_position(i * SMALL_BOX_HEIGHT,TASK2_PICKUP_POS.radius,TASK2_PICKUP_POS.angle,0.0f);
        pickup_box();
        
        // 3. 抬升（叠加大纸箱高度)
        move_to_position(i * SMALL_BOX_HEIGHT + BIG_BOX_HEIGHT,TASK2_PICKUP_POS.radius,TASK2_PICKUP_POS.angle,0.0f);
        move_to_position(i * SMALL_BOX_HEIGHT + BIG_BOX_HEIGHT,TASK2_PICKUP_POS.radius,TASK2_PICKUP_POS.angle,0.0f);

        // 4. 移动到随机位置的补偿位
        move_to_position(i * SMALL_BOX_HEIGHT + BIG_BOX_HEIGHT,random_positions[i].radius,random_positions[i].angle,0.0f);
        
        // 5. 下降到放置高度，松开箱子
        move_to_position(SMALL_BOX_HEIGHT + 5.0f,random_positions[i].radius,random_positions[i].angle,0.0f);
        place_box();
         
         printf("Placing at random pos %d (R=%.1f, A=%.1f)\r\n", 
								i, 
                random_positions[i].radius, 
                random_positions[i].angle);
    }
}


/* ------------------------- 公共接口函数实现 ------------------------- */

/**
 * @brief (供UART中断调用) 发送任务三的随机位置到消息队列
 */
void send_task3_positions(Task3Position_t positions[], uint8_t count)
{
    // 在中断中调用，使用 0 超时
    for (int i = 0; i < count && i < 3; i++)
    {
        osMessageQueuePut(task3positionHandle, &positions[i], 0, 0);
    }
}


/* ------------------------- 私有函数实现 (运动学封装) ------------------------- */

/**
 * @brief  控制机械臂移动到指定位置
 * @param  radius: 目标径向距离 (mm)
 * @param  angle:  目标角度 (度)
 * @param  height: 目标高度 (mm)
 */
static void move_to_position(float height, float radius, float chassis_angle,float suction_angle)
{
    // 控制小臂旋转（2006电机）
    xiaobi_control(suction_angle);
		osDelay(1);
		// 控制主轴高度（3508电机1）
    zhuzhou_control(height);
		osDelay(1);
	  // 控制底盘旋转到目标角度（小米电机）
    chassis_control(chassis_angle);
		osDelay(1);
    // 控制径向距离（3508电机2）
    dabi_control(radius);
		while (!all_motors_in_position()) 
		{
				osDelay(10);
		}						
}


/**
 * @brief  抓取纸箱操作
 * @param  height: 抓取高度 (mm)
 */
static void pickup_box()
{
    // 吸盘抓取
		xipan_control(1);
	osDelay(3000);
}

/**
 * @brief  放置纸箱操作
 * @param  height: 放置高度 (mm)
 */
static void place_box()
{
		//吸盘放下
		xipan_control(0);
	osDelay(3000);
}