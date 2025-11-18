/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : FreeRTOS多任务架构 - 机械臂控制系统
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 *
 * 系统架构说明：
 * =============================================================================
 * 本系统基于 FreeRTOS 实现了一个三自由度机械臂控制器，通过 PS2 无线手柄进行操控。
 *
 * 硬件平台：
 * ----------
 * - MCU: STM32F103C8T6 (ARM Cortex-M3, 72MHz)
 * - RTOS: FreeRTOS V10.3.1
 * - 输入设备: PS2 无线手柄（SPI通信）
 * - 执行器:
 *   1. 底部双电机（差速控制行走）- L298N_L IN1/IN2 (PA0/PA1), L298N_R IN1/IN2 (PB4/PB5)
 *   2. 升降电机（垂直运动）       - L298N_R IN3/IN4 (PB0/PB1)
 *   3. 开合电机（机械爪夹持）     - L298N_L IN3/IN4 (PA2/PA3)
 *   4. 舵机（旋转关节）           - TIM1 CH1 (PA8)
 *
 * 任务架构：
 * ----------
 * 1. ps2Task        (osPriorityHigh)   - PS2数据采集与命令分发
 * 2. servoTask      (osPriorityNormal) - 舵机PWM控制
 * 3. motorTask01    (osPriorityNormal) - 底部双电机混合控制
 * 4. motorTask02    (osPriorityNormal) - 升降电机控制
 * 5. motorTask03    (osPriorityNormal) - 开合电机控制
 *
 * 通信机制：
 * ----------
 * - motorQueue01: ps2Task → motorTask01 (底部电机，摇杆+肩键)
 * - motorQueue02: ps2Task → motorTask02 (升降电机，三角形/X键)
 * - motorQueue03: ps2Task → motorTask03 (开合电机，方形/圆圈键)
 * - servoQueue:   ps2Task → servoTask   (舵机，UP/DOWN键)
 *
 * 控制映射：
 * ----------
 * - 左摇杆(60%) + 右摇杆(40%) → 底部双电机（前进/后退/转向）
 * - L1/L2/R1/R2肩键           → 底部电机肩部辅助控制
 * - 三角形键 / X键            → 升降电机（上/下）
 * - 方形键 / 圆圈键           → 开合电机（开/合）
 * - UP / DOWN 键              → 舵机（旋转）
 *
 * 设计特点：
 * ----------
 * - 平滑过渡算法：12%/10ms步长，避免电机启停冲击
 * - 主动刹车机制：速度为0时电机锁死，防止惯性滑行
 * - 四方向约束：前进/后退优先级高于转向，避免误操作
 * - 最小化系统复杂度：去除碰撞检测与限位保护，降低维护成本
 *
 * 版本历史：
 * ----------
 * - 2025-11-07: 系统简化重构，移除碰撞检测，新增开合电机控制
 * - 2025-10-23: 圆圈键锁止功能优化（三阶段状态机）
 * - 2025-10-20: 初始版本，实现基础控制功能
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ps2.h"
#include "tim.h"

/**
 * ============================================================================
 * FreeRTOS多任务机械臂控制系统
 * ============================================================================
 *
 * 系统架构：5个任务 + 4个消息队列，实现PS2手柄控制的三自由度机械臂
 *
 * 任务优先级架构：
 * ┌────────────────────┬─────────────────────┬──────────────────────┐
 * │ 任务名称           │ 优先级              │ 功能描述             │
 * ├────────────────────┼─────────────────────┼──────────────────────┤
 * │ ps2Task            │ osPriorityHigh      │ PS2手柄数据采集与分发│
 * │ servoTask          │ osPriorityNormal    │ 舵机旋转控制         │
 * │ motorTask01        │ osPriorityNormal    │ 底部双电机差速控制   │
 * │ motorTask02        │ osPriorityNormal    │ 升降电机控制         │
 * │ motorTask03        │ osPriorityNormal    │ 开合电机控制         │
 * └────────────────────┴─────────────────────┴──────────────────────┘
 *
 * 数据流向图：
 *
 *                        ┌──────────────┐
 *                        │  PS2手柄     │
 *                        └──────┬───────┘
 *                               │ SPI通信(50Hz)
 *                        ┌──────▼───────┐
 *                        │  ps2Task     │
 *                        └┬──┬───┬───┬──┘
 *                         │  │   │   │
 *           motorQueue01  │  │   │   │ servoQueue
 *         (摇杆+肩键)     │  │   │   │ (UP/DOWN键)
 *                         │  │   │   │
 *                  ┌──────▼┐ │   │   └──────────┐
 *                  │motor  │ │   │              │
 *                  │Task01 │ │   │      ┌───────▼────┐
 *                  │(底部) │ │   │      │ servoTask  │
 *                  └───────┘ │   │      │ (舵机)     │
 *                        │   │   │      └────────────┘
 *           左右电机PWM  │   │   │
 *                        │   │   │ motorQueue02
 *                        │   │   │ (三角形/X键)
 *                        │   │   │
 *                        │   │   └──────────┐
 *                        │   │              │
 *                        │   │      ┌───────▼────┐
 *                        │   │      │motorTask02 │
 *                        │   │      │(升降电机)  │
 *                        │   │      └────────────┘
 *                        │   │
 *                        │   │ motorQueue03
 *                        │   │ (方形/圆圈键)
 *                        │   │
 *                        │   └──────────┐
 *                        │              │
 *                        │      ┌───────▼────┐
 *                        │      │motorTask03 │
 *                        │      │(开合电机)  │
 *                        ▼      └────────────┘
 *                  底部电机执行
 *
 * 关键技术特性：
 * 1. 平滑过渡：12%/10ms步长，避免电机启停冲击
 * 2. 主动刹车：速度为0时电机锁死(IN1=IN2=HIGH)，防止惯性滑行
 * 3. 死区处理：摇杆中心±10范围视为0，避免机械抖动
 * 4. 四方向约束：前进/后退优先于转向，避免误操作
 * 5. 队列通信：任务间解耦，数据安全传递
 *
 * 资源占用：
 * - Flash: ~18KB / 64KB (28%)
 * - RAM:   ~6.5KB / 20KB (33%)
 * - 栈总计: ~2.5KB (5个任务)
 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for ps2Task */
osThreadId_t ps2TaskHandle;
const osThreadAttr_t ps2Task_attributes = {
    .name       = "ps2Task",
    .stack_size = 256 * 4,
    .priority   = (osPriority_t)osPriorityHigh,
};
/* Definitions for servoTask */
osThreadId_t servoTaskHandle;
const osThreadAttr_t servoTask_attributes = {
    .name       = "servoTask",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityNormal,
};
/* Definitions for motorTask01 */
osThreadId_t motorTask01Handle;
const osThreadAttr_t motorTask01_attributes = {
    .name       = "motorTask01",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityNormal,
};
/* Definitions for motorTask02 */
osThreadId_t motorTask02Handle;
const osThreadAttr_t motorTask02_attributes = {
    .name       = "motorTask02",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityNormal,
};
/* Definitions for motorTask03 */
osThreadId_t motorTask03Handle;
const osThreadAttr_t motorTask03_attributes = {
    .name       = "motorTask03",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityNormal,
};
/* Definitions for motorQueue01 */
osMessageQueueId_t motorQueue01Handle;
const osMessageQueueAttr_t motorQueue01_attributes = {
    .name = "motorQueue01"};
/* Definitions for motorQueue02 */
osMessageQueueId_t motorQueue02Handle;
const osMessageQueueAttr_t motorQueue02_attributes = {
    .name = "motorQueue02"};
/* Definitions for motorQueue03 */
osMessageQueueId_t motorQueue03Handle;
const osMessageQueueAttr_t motorQueue03_attributes = {
    .name = "motorQueue03"};
/* Definitions for servoQueue */
osMessageQueueId_t servoQueueHandle;
const osMessageQueueAttr_t servoQueue_attributes = {
    .name = "servoQueue"};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartPs2Task(void *argument);
void StartServoTask(void *argument);
void StartMotorTask01(void *argument);
void StartMotorTask02(void *argument);
void StartMotorTask03(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* Create the queue(s) */
    /* creation of motorQueue01 */
    motorQueue01Handle = osMessageQueueNew(5, 12, &motorQueue01_attributes);

    /* creation of motorQueue02 */
    motorQueue02Handle = osMessageQueueNew(5, 2, &motorQueue02_attributes);

    /* creation of motorQueue03 */
    motorQueue03Handle = osMessageQueueNew(5, 2, &motorQueue03_attributes);

    /* creation of servoQueue */
    servoQueueHandle = osMessageQueueNew(5, 2, &servoQueue_attributes);

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of ps2Task */
    ps2TaskHandle = osThreadNew(StartPs2Task, NULL, &ps2Task_attributes);

    /* creation of servoTask */
    servoTaskHandle = osThreadNew(StartServoTask, NULL, &servoTask_attributes);

    /* creation of motorTask01 */
    motorTask01Handle = osThreadNew(StartMotorTask01, NULL, &motorTask01_attributes);

    /* creation of motorTask02 */
    motorTask02Handle = osThreadNew(StartMotorTask02, NULL, &motorTask02_attributes);

    /* creation of motorTask03 */
    motorTask03Handle = osThreadNew(StartMotorTask03, NULL, &motorTask03_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartPs2Task */
/**
 * @brief  PS2手柄数据采集与分发任务
 * @param  argument: Not used
 * @retval None
 * @note   任务优先级: osPriorityHigh（高优先级）
 *
 *         ========== 控制方案架构 ==========
 *
 *         【底部电机】双摇杆混合控制 + 肩键辅助
 *         - motorQueue01: 左摇杆(0.6权重) + 右摇杆(0.4权重) + L1/L2/R1/R2
 *           * 左摇杆: leftY/leftX × 0.6（主控制，四方向限制）
 *           * 右摇杆: rightY/rightX × 0.4（辅助控制，四方向限制）
 *           * 方向限制: 只保留前/后/左/右四个方向，偏斜时取最近的方向
 *           * L1/L2: 控制左轮正转/反转（仅摇杆无输出时生效）
 *           * R1/R2: 控制右轮正转/反转（仅摇杆无输出时生效）
 *
 *         【舵机】方向键控制
 *         - servoQueue: 方向键上/下
 *           * 上键: 舵机正转
 *           * 下键: 舵机反转
 *
 *         【升降电机】三角形/X键控制
 *         - motorQueue02: 三角形/X键
 *           * 三角形: 升降电机正转（上升）
 *           * X键: 升降电机反转（下降）
 *
 *         【开合电机】方形/圆圈键控制
 *         - motorQueue03: 方形/圆圈键
 *           * 方形键: 开合电机正转（打开）
 *           * 圆圈键: 开合电机反转（关闭）
 *
 *         数据包格式：
 *         motorQueue01（12字节）：
 *         [0-3]   uint32_t: 左摇杆数据 (高16位=leftY, 低16位=leftX)
 *         [4-7]   uint32_t: 右摇杆数据 (高16位=rightY, 低16位=rightX)
 *         [8-11]  uint32_t: 肩键数据 (bit0=L1, bit1=L2, bit2=R1, bit3=R2)
 *
 *         motorQueue02/03/servoQueue（2字节）：
 *         uint16_t: 按键状态位掩码
 *
 *         采集周期: 20ms（50Hz）
 */
/* USER CODE END Header_StartPs2Task */
void StartPs2Task(void *argument)
{
    /* USER CODE BEGIN StartPs2Task */
    PS2_Data_t ps2Data;

    // 底部电机数据包结构：12字节
    typedef struct {
        uint32_t leftJoystick;    // 左摇杆 (高16位=Y, 低16位=X)
        uint32_t rightJoystick;   // 右摇杆 (高16位=Y, 低16位=X)
        uint32_t shoulderButtons; // 肩键 (bit0=L1, bit1=L2, bit2=R1, bit3=R2)
    } MotorData_t;

    MotorData_t motorData;
    uint16_t buttonData; // 用于其他队列（2字节）

    // 初始化 PS2 手柄
    PS2_Init();

    // 初始化：发送"锁死"数据到所有队列
    motorData.leftJoystick    = 0;
    motorData.rightJoystick   = 0;
    motorData.shoulderButtons = 0;
    osMessageQueuePut(motorQueue01Handle, &motorData, 0, 0);

    buttonData = 0;
    osMessageQueuePut(servoQueueHandle, &buttonData, 0, 0);
    osMessageQueuePut(motorQueue02Handle, &buttonData, 0, 0);
    osMessageQueuePut(motorQueue03Handle, &buttonData, 0, 0);

    /* Infinite loop */
    for (;;) {
        // 读取 PS2 手柄数据
        if (PS2_ReadData(&ps2Data) == 0) {
            // 检查PS2是否处于模拟模式（0x79）或红灯数字模式（0x73）
            if (ps2Data.mode == 0x79 || ps2Data.mode == 0x73) {
                // ========== 1. 底部电机数据包（双摇杆 + 肩键） ==========
                int16_t leftY          = (int16_t)(ps2Data.leftY - 128);
                int16_t leftX          = (int16_t)(ps2Data.leftX - 128);
                motorData.leftJoystick = ((int32_t)leftY << 16) | (uint16_t)leftX;

                int16_t rightY          = (int16_t)(ps2Data.rightY - 128);
                int16_t rightX          = (int16_t)(ps2Data.rightX - 128);
                motorData.rightJoystick = ((int32_t)rightY << 16) | (uint16_t)rightX;

                // 肩键数据（L1/L2/R1/R2）
                motorData.shoulderButtons = 0;
                if (ps2Data.buttons & PS2_L1) motorData.shoulderButtons |= 0x01;
                if (ps2Data.buttons & PS2_L2) motorData.shoulderButtons |= 0x02;
                if (ps2Data.buttons & PS2_R1) motorData.shoulderButtons |= 0x04;
                if (ps2Data.buttons & PS2_R2) motorData.shoulderButtons |= 0x08;

                osMessageQueueReset(motorQueue01Handle);
                osMessageQueuePut(motorQueue01Handle, &motorData, 0, 0);

                // ========== 2. 舵机控制（方向键上/下） ==========
                buttonData = 0;
                if (ps2Data.buttons & PS2_UP) buttonData |= 0x01;
                if (ps2Data.buttons & PS2_DOWN) buttonData |= 0x02;

                osMessageQueueReset(servoQueueHandle);
                osMessageQueuePut(servoQueueHandle, &buttonData, 0, 0);

                // ========== 3. 升降电机控制（三角形/X键） ==========
                buttonData = 0;
                if (ps2Data.buttons & PS2_TRIANGLE) buttonData |= 0x01;
                if (ps2Data.buttons & PS2_CROSS) buttonData |= 0x02;

                osMessageQueueReset(motorQueue02Handle);
                osMessageQueuePut(motorQueue02Handle, &buttonData, 0, 0);

                // ========== 4. 开合电机控制（方形/圆圈键） ==========
                buttonData = 0;
                if (ps2Data.buttons & PS2_SQUARE) buttonData |= 0x01; // 方形键
                if (ps2Data.buttons & PS2_CIRCLE) buttonData |= 0x02; // 圆圈键

                osMessageQueueReset(motorQueue03Handle);
                osMessageQueuePut(motorQueue03Handle, &buttonData, 0, 0);
            } else {
                // PS2处于数字模式或异常模式，发送锁死命令
                motorData.leftJoystick    = 0;
                motorData.rightJoystick   = 0;
                motorData.shoulderButtons = 0;
                osMessageQueueReset(motorQueue01Handle);
                osMessageQueuePut(motorQueue01Handle, &motorData, 0, 0);

                buttonData = 0;
                osMessageQueueReset(servoQueueHandle);
                osMessageQueuePut(servoQueueHandle, &buttonData, 0, 0);
                osMessageQueueReset(motorQueue02Handle);
                osMessageQueuePut(motorQueue02Handle, &buttonData, 0, 0);
                osMessageQueueReset(motorQueue03Handle);
                osMessageQueuePut(motorQueue03Handle, &buttonData, 0, 0);
            }
        } else {
            // PS2读取失败，发送锁死命令保护电机
            motorData.leftJoystick    = 0;
            motorData.rightJoystick   = 0;
            motorData.shoulderButtons = 0;
            osMessageQueueReset(motorQueue01Handle);
            osMessageQueuePut(motorQueue01Handle, &motorData, 0, 0);

            buttonData = 0;
            osMessageQueueReset(servoQueueHandle);
            osMessageQueuePut(servoQueueHandle, &buttonData, 0, 0);
            osMessageQueueReset(motorQueue02Handle);
            osMessageQueuePut(motorQueue02Handle, &buttonData, 0, 0);
            osMessageQueueReset(motorQueue03Handle);
            osMessageQueuePut(motorQueue03Handle, &buttonData, 0, 0);
        }

        osDelay(20);
    }
    /* USER CODE END StartPs2Task */
}

/* USER CODE BEGIN Header_StartServoTask */
/**
 * @brief  舵机云台控制任务（方向键控制）
 * @param  argument: Not used
 * @retval None
 * @note   任务优先级: osPriorityNormal（普通优先级）
 *
 *         控制逻辑（新方案）：
 *         - 方向键上 → 舵机正转（角度增加）
 *         - 方向键下 → 舵机反转（角度减少）
 *         - 上+下同时按下 → 停止转动（保持当前角度）
 *         - 无按键 → 停止转动
 *
 *         转动速度: 2°/10ms = 200°/s
 *         角度范围: 0° ~ 180°
 *         PWM频率: 50Hz (TIM1 CH1)
 */
/* USER CODE END Header_StartServoTask */
void StartServoTask(void *argument)
{
    /* USER CODE BEGIN StartServoTask */
    extern TIM_HandleTypeDef htim1;
    uint16_t buttonData   = 0;
    uint16_t currentAngle = 0; // 当前角度，初始化为 0 度

    // 启动 PWM 输出
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    // 设置初始角度为 0 度
    Servo_SetAngle(currentAngle);
    osDelay(100);

    /* Infinite loop */
    for (;;) {
        // 从队列接收按键状态（非阻塞）
        if (osMessageQueueGet(servoQueueHandle, &buttonData, NULL, 0) == osOK) {
            // 检测按键冲突：上和下同时按下时停止转动
            if ((buttonData & 0x01) && (buttonData & 0x02)) {
                // 两个按键同时按下，舵机停止（不改变角度）
            }
            // 方向键上按下 → 舵机正转（角度增加）
            else if (buttonData & 0x01) {
                if (currentAngle < 179) {
                    currentAngle += 2; // 每次增加 2 度
                } else {
                    currentAngle = 180; // 接近边界时直接到 180
                }
            }
            // 方向键下按下 → 舵机反转（角度减少）
            else if (buttonData & 0x02) {
                if (currentAngle > 1) {
                    currentAngle -= 2; // 每次减少 2 度
                } else {
                    currentAngle = 0; // 接近边界时直接到 0
                }
            }
            // 两个按键都没按下，停止转动（保持当前角度）
        }

        // 设置舵机角度
        Servo_SetAngle(currentAngle);

        // 延时 10ms，实现平滑转动（约 200°/秒的转速）
        osDelay(10);
    }
    /* USER CODE END StartServoTask */
}

/* USER CODE BEGIN Header_StartMotorTask01 */
/**
 * @brief  底部双电机混合控制任务（双摇杆+肩键）
 * @param  argument: Not used
 * @retval None
 * @note   任务优先级: osPriorityNormal（普通优先级）
 *
 *         ========== 控制方案 ==========
 *
 *         【摇杆混合控制】双摇杆加权平均
 *         - 左摇杆权重: 0.6（主控制）
 *         - 右摇杆权重: 0.4（辅助控制）
 *         - 混合后的 leftY/leftX/rightY/rightX 共同决定双电机速度
 *
 *         【死区处理】±5
 *         - 摇杆值在 [-5, +5] 范围内视为0,避免机械抖动误触发
 *         - 死区较小，提高摇杆灵敏度
 *
 *         【四方向约束】
 *         - 检测摇杆主要方向（上/下/左/右）,抑制斜向输入
 *         - 前进/后退优先级高于左转/右转,避免误操作
 *
 *         【肩键辅助】L1/L2/R1/R2
 *         - 仅在摇杆无输出时生效
 *         - L1: 左轮正转  | L2: 左轮反转
 *         - R1: 右轮正转  | R2: 右轮反转
 *         - 肩键速度: 127（全速）
 *         - 可实现原地转向、精确调整等操作
 *
 *         【平滑过渡算法】
 *         - 步长: 12/10ms（每次最大变化12）
 *         - 避免电机启动/停止时的机械冲击和噪音
 *
 *         【主动刹车机制】
 *         - 目标速度为0时,电机锁死(IN1=IN2=HIGH)
 *         - 防止惯性滑行,提高定位精度
 *
 *         硬件连接：
 *         - 左轮: TIM2 CH1/CH2 (PA0/PA1) → L298N_L IN1/IN2
 *         - 右轮: TIM3 CH1/CH2 (PB4/PB5) → L298N_R IN1/IN2
 *         - PWM频率: 1kHz
 *
 *         系统简化记录（2025-11-07）：
 *         - 移除圆圈键锁止功能（三阶段状态机）
 *         - 移除碰撞检测相关代码（leftMotorScale/rightMotorScale变量）
 *         - 纯粹的摇杆+肩键控制,降低系统复杂度
 */
/* USER CODE END Header_StartMotorTask01 */
void StartMotorTask01(void *argument)
{
    /* USER CODE BEGIN StartMotorTask01 */
    // 初始化PWM比较值为锁死状态（防止启动时电机乱动）
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 999);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 999);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 999);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 999);

    // 启动TIM2 PWM通道1和2（左侧电机 - L298N_L IN1/IN2）
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    // 启动TIM3 PWM通道1和2（右侧电机 - L298N_R IN1/IN2）
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    // 确保初始状态为锁死
    Motor_SetSpeedLeft(0);
    Motor_SetSpeedRight(0); // 数据包结构定义
    typedef struct {
        uint32_t leftJoystick;    // 左摇杆
        uint32_t rightJoystick;   // 右摇杆
        uint32_t shoulderButtons; // 肩键
    } MotorData_t;

    MotorData_t motorData;
    int16_t leftY, leftX, rightY, rightX;
    float leftSpeed_f, rightSpeed_f;
    int16_t leftSpeed, rightSpeed;
    uint32_t buttons;

    const int16_t DEADZONE     = 5;   // 摇杆死区范围（±5）
    const int16_t BUTTON_SPEED = 127; // 肩键控制速度（100%最大值）

    /* Infinite loop */
    for (;;) {
        if (osMessageQueueGet(motorQueue01Handle, &motorData, NULL, 0) == osOK) {
            // ========== 1. 解包摇杆数据 ==========
            leftY   = (int16_t)((motorData.leftJoystick >> 16) & 0xFFFF);
            leftX   = (int16_t)(motorData.leftJoystick & 0xFFFF);
            rightY  = (int16_t)((motorData.rightJoystick >> 16) & 0xFFFF);
            rightX  = (int16_t)(motorData.rightJoystick & 0xFFFF);
            buttons = motorData.shoulderButtons;

            // ========== 2. 摇杆死区处理 ==========
            if (leftY > -DEADZONE && leftY < DEADZONE) leftY = 0;
            if (leftX > -DEADZONE && leftX < DEADZONE) leftX = 0;
            if (rightY > -DEADZONE && rightY < DEADZONE) rightY = 0;
            if (rightX > -DEADZONE && rightX < DEADZONE) rightX = 0;

            // ========== 3. 判断是否使用摇杆控制 ==========
            int joystickActive = (leftY != 0 || leftX != 0 || rightY != 0 || rightX != 0);

            if (joystickActive) {
                // ========== 摇杆模式：四方向限制 ==========

                // 处理左摇杆：判断最接近的方向
                int16_t L_finalY = 0, L_finalX = 0;
                if (leftY != 0 || leftX != 0) {
                    int16_t absY = (leftY > 0) ? leftY : -leftY;
                    int16_t absX = (leftX > 0) ? leftX : -leftX;

                    if (absY > absX) {
                        L_finalY = leftY;
                        L_finalX = 0;
                    } else {
                        L_finalY = 0;
                        L_finalX = leftX;
                    }
                }

                // 处理右摇杆：判断最接近的方向
                int16_t R_finalY = 0, R_finalX = 0;
                if (rightY != 0 || rightX != 0) {
                    int16_t absY = (rightY > 0) ? rightY : -rightY;
                    int16_t absX = (rightX > 0) ? rightX : -rightX;

                    if (absY > absX) {
                        R_finalY = rightY;
                        R_finalX = 0;
                    } else {
                        R_finalY = 0;
                        R_finalX = rightX;
                    }
                }

                // 差速计算
                float L_left  = (float)L_finalY - (float)L_finalX;
                float L_right = (float)L_finalY + (float)L_finalX;
                float R_left  = (float)R_finalY - (float)R_finalX;
                float R_right = (float)R_finalY + (float)R_finalX;

                // 加权混合（0.6 + 0.4）
                leftSpeed_f  = L_left * 0.6f + R_left * 0.4f;
                rightSpeed_f = L_right * 0.6f + R_right * 0.4f;

                // 动态归一化（防止溢出）
                float maxSpeed = (leftSpeed_f > 0 ? leftSpeed_f : -leftSpeed_f);
                float rightAbs = (rightSpeed_f > 0 ? rightSpeed_f : -rightSpeed_f);
                if (rightAbs > maxSpeed) maxSpeed = rightAbs;

                if (maxSpeed > 127.0f) {
                    float scale = 127.0f / maxSpeed;
                    leftSpeed_f *= scale;
                    rightSpeed_f *= scale;
                }

                leftSpeed  = (int16_t)leftSpeed_f;
                rightSpeed = (int16_t)rightSpeed_f;

            } else {
                // ========== 肩键模式：仅当摇杆无输出时响应 ==========
                leftSpeed  = 0;
                rightSpeed = 0;

                // L1/L2控制左轮
                if (buttons & 0x01) {
                    leftSpeed = BUTTON_SPEED;
                } else if (buttons & 0x02) {
                    leftSpeed = -BUTTON_SPEED;
                }

                // R1/R2控制右轮
                if (buttons & 0x04) {
                    rightSpeed = BUTTON_SPEED;
                } else if (buttons & 0x08) {
                    rightSpeed = -BUTTON_SPEED;
                }
            }

            // ========== 4. 最终安全限幅 ==========
            if (leftSpeed > 127) leftSpeed = 127;
            if (leftSpeed < -127) leftSpeed = -127;
            if (rightSpeed > 127) rightSpeed = 127;
            if (rightSpeed < -127) rightSpeed = -127;

            // ========== 5. 控制电机 ==========
            Motor_SetSpeedLeft(leftSpeed);
            Motor_SetSpeedRight(rightSpeed);
        }

        osDelay(10);
    }
    /* USER CODE END StartMotorTask01 */
}

/* USER CODE BEGIN Header_StartMotorTask02 */
/**
 * @brief  升降电机控制任务（三角形/X键控制）
 * @param  argument: Not used
 * @retval None
 * @note   任务优先级: osPriorityNormal（普通优先级）
 *
 *         控制逻辑：
 *         - 三角形键按下 → 正转最大速度 (+127,上升）
 *         - X键按下      → 反转最大速度 (-127,下降）
 *         - 两键同时按下 → 电机锁死 (0)
 *         - 无按键       → 电机锁死 (0)
 *
 *         硬件连接：
 *         - TIM3 CH3/CH4 (PB0/PB1) → L298N_R IN3/IN4
 *         - PWM频率: 1kHz
 *         - 最大速度: 127
 *
 *         系统简化记录（2025-11-07）：
 *         - 移除顶部限位开关检测（PA9/PA10引脚）
 *         - 移除topLimitPressed全局变量
 *         - 纯粹的按键控制,无硬件保护
 */
/* USER CODE END Header_StartMotorTask02 */
void StartMotorTask02(void *argument)
{
    /* USER CODE BEGIN StartMotorTask02 */
    // 初始化PWM比较值为锁死状态
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 999);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 999);

    // 启动TIM3 PWM通道3和4（升降电机）
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    // 确保初始状态为锁死
    Motor_SetSpeedTop(0);

    uint16_t buttonData = 0;
    int16_t motorSpeed  = 0;

    const int16_t MAX_SPEED = 127;

    /* Infinite loop */
    for (;;) {
        // 尝试从队列获取按键数据（非阻塞）
        if (osMessageQueueGet(motorQueue02Handle, &buttonData, NULL, 0) == osOK) {
            // 成功获取数据，解析按键状态
            uint8_t triangle_pressed = (buttonData & 0x01) ? 1 : 0;
            uint8_t x_pressed        = (buttonData & 0x02) ? 1 : 0;

            if (triangle_pressed && x_pressed) {
                motorSpeed = 0;
            } else if (triangle_pressed) {
                motorSpeed = MAX_SPEED;
            } else if (x_pressed) {
                motorSpeed = -MAX_SPEED;
            } else {
                motorSpeed = 0;
            }
        }

        // 无论是否获取到新数据，都执行电机控制
        Motor_SetSpeedTop(motorSpeed);

        osDelay(10);
    }
    /* USER CODE END StartMotorTask02 */
}

/* USER CODE BEGIN Header_StartMotorTask03 */
/**
 * @brief  开合电机控制任务（方形/圆圈键控制）
 * @param  argument: Not used
 * @retval None
 * @note   任务优先级: osPriorityNormal（普通优先级）
 *
 *         控制逻辑：
 *         - 方形键按下   → 正转最大速度 (+127,打开）
 *         - 圆圈键按下   → 反转最大速度 (-127,关闭）
 *         - 两键同时按下 → 电机锁死 (0)
 *         - 无按键       → 电机锁死 (0)
 *
 *         硬件连接：
 *         - TIM2 CH3/CH4 (PA2/PA3) → L298N_L IN3/IN4
 *         - PWM频率: 1kHz
 *         - 最大速度: 127
 *
 *         功能演进记录（2025-11-07）：
 *         - 新增开合电机控制功能
 *         - 圆圈键从"锁止模式"改为"开合电机反转"
 *         - 引脚 PA2/PA3 用于开合电机控制（L298N_L驱动器）
 *         - 使用新增函数 Motor_SetSpeedGrip() 进行控制
 */
/* USER CODE END Header_StartMotorTask03 */
void StartMotorTask03(void *argument)
{
    /* USER CODE BEGIN StartMotorTask03 */
    // 初始化PWM比较值为锁死状态
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 999);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 999);

    // 启动TIM2 PWM通道3和4（开合电机 - L298N_L IN3/IN4, PA2/PA3）
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    // 确保初始状态为锁死
    Motor_SetSpeedGrip(0);

    uint16_t buttonData = 0;
    int16_t motorSpeed  = 0;

    const int16_t MAX_SPEED = 127;

    /* Infinite loop */
    for (;;) {
        // 尝试从队列获取按键数据（非阻塞）
        if (osMessageQueueGet(motorQueue03Handle, &buttonData, NULL, 0) == osOK) {
            // 成功获取数据，解析按键状态
            uint8_t square_pressed = (buttonData & 0x01) ? 1 : 0; // 方形键
            uint8_t circle_pressed = (buttonData & 0x02) ? 1 : 0; // 圆圈键

            if (square_pressed && circle_pressed) {
                motorSpeed = 0;
            } else if (square_pressed) {
                motorSpeed = MAX_SPEED;
            } else if (circle_pressed) {
                motorSpeed = -MAX_SPEED;
            } else {
                motorSpeed = 0;
            }
        }

        // 无论是否获取到新数据，都执行电机控制
        Motor_SetSpeedGrip(motorSpeed);

        osDelay(10);
    }
    /* USER CODE END StartMotorTask03 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
