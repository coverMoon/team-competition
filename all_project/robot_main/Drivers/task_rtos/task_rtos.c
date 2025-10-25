#include "task_rtos.h"

/* ------------------------- 机械臂参数定义 ------------------------- */
#define ARM_LENGTH 300.0f           // 小臂长度 (mm)
#define MAIN_SHAFT_MIN_LENGTH 0.0f  // 主轴最小长度 (mm)
#define MAIN_SHAFT_MAX_LENGTH 400.0f // 主轴最大长度 (mm)
#define BIG_ARM_MIN_LENGTH 0.0f     // 大臂最小长度 (mm)
#define BIG_ARM_MAX_LENGTH 400.0f   // 大臂最大长度 (mm)
#define SMALL_BOX_HEIGHT 150.0f        // 小纸箱高度 (mm)
#define BIG_BOX_HEIGHT 180.0f          //大纸箱高度（mm）
#define TASK_NONE     0            // 无任务状态
#define TASK_1 1                   // 任务1：箱子堆叠流程
#define TASK_2 2                   // 任务2：预留
#define TASK_3 3                   // 任务3：预留
#define task1_xiaobi_angle 0.0f  //任务一中的小臂旋转的角度
#define task3_xiaobi_angle 90.0f //任务三种的小臂旋转的角度
#define task2_3_xiaobi_angle 75.0f //任务二中三区的小臂旋转的角度
#define task2_2_xiaobi_angle 90.0f //任务二中二区的小臂旋转的角度
#define task2_3_jian_radius 146.8f //任务二中三区的距离差
#define task2_3_jia_height 113.1f //任务二中三区的高度差
static volatile bool is_task_running = false;            // 任务运行状态标志（防止任务并发）

/* ------------------------- 任务一相关定义 ------------------------- */
typedef struct {
    float radius;  // 径向距离 (mm)
    float angle;   // 角度 (度)
} PolarCoord;

static const PolarCoord task1_boxes[] = {
    {450.0f, 60.0f},
    {350.0f, 45.0f},  
    {400.0f, 330.0f}
};

static const PolarCoord task1_target = {350.0f, 270.0f};

/* ------------------------- 任务二相关定义 ------------------------- */
typedef struct {
    PolarCoord pos;
    float height;
    int type;
} BoxZone;

static const BoxZone task2_zones[] = {
    {{350.0f, 0.0f},155.0f, 1},      // 一区
    {{350.0f, 45.0f}, 155.0f, 1},
    {{350.0f, 315.0f}, 155.0f, 1},
    {{600.0f, 30.0f}, 390.0f, 2},   // 二区
    {{600.0f, 330.0f}, 390.0f, 2},
    {{550.0f, 0.0f}, 260.0f, 3},     // 三区
    {{550.0f, 60.0f}, 260.0f, 3},
    {{550.0f, 300.0f}, 260.0f, 3}
};

static const PolarCoord task2_pickup_pos = {350.0f, 180.0f};

/* ------------------------- 私有函数声明 ------------------------- */
static void move_to_position(float radius, float angle, float height,float xiaobi_angle);
static void pickup_box();
static void place_box();
static float calculate_arm_angle(float height);
static void calculate_arm_lengths(float target_radius, float arm_angle, float* main_shaft_length, float* big_arm_length);
static osStatus_t wait_for_motion_completion(uint32_t timeout);

/* ------------------------- 任务函数实现 ------------------------- */

/**
 * @brief  主任务：任务调度核心
 * @details 初始化电机后，循环检测sys变量，根据sys值切换执行不同任务
 *          通过is_task_running标志防止任务并发执行，确保动作安全性
 * @param  argument: 任务参数（未使用）
 * @retval 无
 */

void Task0(void *argument)
{
    /* USER CODE BEGIN Task0 */
    chassis_init();  // 初始化底盘电机
    dji_init();      // 初始化DJI电机（主轴、大臂、小臂）

    extern int16_t sys;  // 外部定义的任务切换变量（通过UART接收更新）

    /* 无限循环：任务调度逻辑 */
    for(;;)
    {
        // 仅当无任务运行且sys指定有效任务时，启动新任务
        if (!is_task_running && sys != TASK_NONE)  {
            is_task_running = true;  // 标记任务开始运行
            switch(sys) {
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
                    printf("Invalid task: %d\r\n", sys);  // 无效任务编号提示
                    break;
            }

            // 任务执行完成，重置状态
            is_task_running = false;
            sys = TASK_NONE;  // 复位sys，等待下一次任务指令
        }

        osDelay(1);  // 降低调度频率，减少CPU占用
    }
    /* USER CODE END Task0 */
}


void task1_rtos()
{
        // 初始化电机
        
        // 依次抓取三个纸箱
        for (int i = 0; i < 3; i++) 
        {
            // 移动到纸箱位置并抓取
            move_to_position(task1_boxes[i].radius, task1_boxes[i].angle, SMALL_BOX_HEIGHT+5.0f,task1_xiaobi_angle);
            move_to_position(task1_boxes[i].radius, task1_boxes[i].angle, SMALL_BOX_HEIGHT,task1_xiaobi_angle);
            pickup_box();
            
						//稍微抬升避免碰撞
            move_to_position(task1_target.radius, task1_target.angle, i * SMALL_BOX_HEIGHT+20.0f,task1_xiaobi_angle);
						// 移动到放置位置并放置
            move_to_position(task1_target.radius, task1_target.angle, i * SMALL_BOX_HEIGHT,task1_xiaobi_angle);
						while (!all_motors_in_position()) {
							osDelay(10);
						}						
            place_box();
        }
        
        // 返回安全位置
        move_to_position(MAIN_SHAFT_MAX_LENGTH,0.0f ,BIG_ARM_MAX_LENGTH,task1_xiaobi_angle );
}

void task2_rtos()
{
	int chosen_target[4]={3,4,5,6};
	for(int i = 0; i < 4; i++)
	{
		//稍微抬升避免碰撞
    move_to_position(task2_pickup_pos.radius, task2_pickup_pos.angle, i * SMALL_BOX_HEIGHT+20.0f,task1_xiaobi_angle);
		// 抓取小纸箱	
		move_to_position(task2_pickup_pos.radius, task2_pickup_pos.angle, i * SMALL_BOX_HEIGHT,task1_xiaobi_angle);
    pickup_box();		
    //稍微抬升避免碰撞
    move_to_position(task2_pickup_pos.radius, task2_pickup_pos.angle, i * SMALL_BOX_HEIGHT+20.0f,task1_xiaobi_angle);
		if(task2_zones[chosen_target[i]].type==3)
		{
			//匹配三区并放置
		move_to_position(task2_zones[chosen_target[i]].pos.radius-task2_3_jian_radius,task2_zones[chosen_target[i]].pos.angle,task2_zones[chosen_target[i]].height+task2_3_jia_height,task2_3_xiaobi_angle);		
		place_box();
		}
		else if(task2_zones[chosen_target[i]].type==3)
		{
			//匹配二区并放置
		move_to_position(task2_zones[chosen_target[i]].pos.radius-150.0f,task2_zones[chosen_target[i]].pos.angle,task2_zones[chosen_target[i]].height+task2_3_jia_height,task2_2_xiaobi_angle);		
		move_to_position(task2_zones[chosen_target[i]].pos.radius,task2_zones[chosen_target[i]].pos.angle,task2_zones[chosen_target[i]].height+task2_3_jia_height,task2_2_xiaobi_angle);		
		}
	
	}
	
	
				}

void task3_rtos()
{
	printf("OOK");
    Task3Position_t positions[3];
    uint8_t received_positions = 0;
    
    // 等待接收所有三个随机位置
    while (received_positions < 3) {
        if (osMessageQueueGet(task3positionHandle, &positions[received_positions], NULL, osWaitForever) == osOK) {
            received_positions++;
					printf("OK");
        }
    }        
        for (int i = 3; i > 0; i--) 
        {
						//稍微抬升避免碰撞
            move_to_position(task2_pickup_pos.radius, task2_pickup_pos.angle, i * SMALL_BOX_HEIGHT+20.0f,task1_xiaobi_angle);
						// 抓取小纸箱	
						move_to_position(task2_pickup_pos.radius, task2_pickup_pos.angle, i * SMALL_BOX_HEIGHT,task1_xiaobi_angle);
            pickup_box();
            //稍微抬升避免碰撞
						move_to_position(task2_pickup_pos.radius, task2_pickup_pos.angle, i * SMALL_BOX_HEIGHT+BIG_BOX_HEIGHT,task1_xiaobi_angle);
            move_to_position(task2_pickup_pos.radius, task2_pickup_pos.angle, i * SMALL_BOX_HEIGHT+BIG_BOX_HEIGHT,task3_xiaobi_angle);

            // 放置到随机位置
            move_to_position(positions[i].radius-75.0f, positions[i].angle, i * SMALL_BOX_HEIGHT+BIG_BOX_HEIGHT,task3_xiaobi_angle);
            move_to_position(positions[i].radius-75.0f, positions[i].angle, SMALL_BOX_HEIGHT+5.0f,task3_xiaobi_angle);
            place_box();
        }
        
        // 返回安全位置
        move_to_position(300.0f, 0.0f, 0.0f,task1_xiaobi_angle);
        
}

/* ------------------------- 公共接口函数 ------------------------- */

void send_task3_positions(Task3Position_t positions[], uint8_t count)
{
    for (int i = 0; i < count && i < 3; i++) {
        osMessageQueuePut(task3positionHandle, &positions[i], 0, 0);
    }
}

void stop_competition_task(osThreadId_t task_handle)
{
    if (task_handle != NULL) {
        osThreadFlagsSet(task_handle, 0x0001);
    }
}

/* ------------------------- 私有函数实现 ------------------------- */

/**
 * @brief  控制机械臂移动到指定位置
 * @param  radius: 目标径向距离 (mm)
 * @param  angle:  目标角度 (度)
 * @param  height: 目标高度 (mm)
 */
static void move_to_position(float radius, float angle, float height,float xiaobi_angle)
{
    
    // 控制小臂旋转（2006电机）
    xiaobi_control(xiaobi_angle);
		    // 控制主轴身长（3508电机1）
    zhuzhou_control(height);
	    //控制主轴旋转到目标角度（小米电机）
    chassis_control(angle);
    // 控制大臂伸长（3508电机2）
    dabi_control(radius);
						while (!all_motors_in_position()) {
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
}

/**
 * @brief  放置纸箱操作
 * @param  height: 放置高度 (mm)
 */
static void place_box()
{
	//吸盘放下
xipan_control(0);
}
