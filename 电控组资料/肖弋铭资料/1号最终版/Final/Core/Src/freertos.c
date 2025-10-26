/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
 * FreeRTOS多任务智能小车控制系统
 * ============================================================================
 *
 * 系统架构：6个任务 + 4个消息队列，实现PS2手柄控制的智能避障小车
 *
 * 任务优先级架构（从高到低）：
 * ┌────────────────────┬─────────────────────┬──────────────────────┐
 * │ 任务名称           │ 优先级              │ 功能描述             │
 * ├────────────────────┼─────────────────────┼──────────────────────┤
 * │ collisionTask01    │ osPriorityHigh      │ 碰撞检测与避障控制   │
 * │ ps2Task            │ osPriorityAboveNormal│ PS2手柄数据采集      │
 * │ collisionTask02    │ osPriorityAboveNormal│ 预留扩展（超声波等） │
 * │ servoTask          │ osPriorityNormal    │ 舵机云台控制         │
 * │ motorTask01        │ osPriorityNormal    │ 下部双电机差速转向   │
 * │ motorTask02        │ osPriorityNormal    │ 上部电机控制         │
 * └────────────────────┴─────────────────────┴──────────────────────┘
 *
 * 数据流向图：
 *
 *                        ┌──────────────┐
 *                        │  PS2手柄     │
 *                        └──────┬───────┘
 *                               │ SPI通信
 *                        ┌──────▼───────┐
 *                        │  ps2Task     │ (50Hz采集)
 *                        └──┬─────┬──┬──┘
 *                           │     │  │
 *              motorQueue01 │     │  │ servoQueue
 *              (左右摇杆Y)  │     │  │ (L1/L2按键)
 *                           │     │  │
 *                  ┌────────▼─┐   │  └──────────┐
 *                  │motorTask01│   │             │
 *                  │(下部电机) │   │    ┌────────▼────┐
 *                  └─────┬─────┘   │    │ servoTask   │
 *                        │         │    │ (舵机控制)  │
 *            左右电机PWM │         │    └─────────────┘
 *                        │         │ motorQueue02
 *            ┌───────────▼──┐      │ (R1/R2按键)
 *            │ 碰撞避障逻辑 │◄─────┤
 *            │leftMotorScale│      │
 *            │rightMotorScale│     └──────────┐
 *            └──────────────┘                 │
 *                                    ┌────────▼────┐
 *                                    │motorTask02  │
 *                                    │(上部电机)   │
 *                                    └─────────────┘
 *
 * 关键技术特性：
 * 1. 碰撞避障：微动开关+平滑过渡算法，50ms内完成减速/加速
 * 2. 电机锁死：摇杆回中时主动刹车(IN1=IN2=HIGH)，精确定位
 * 3. 死区处理：摇杆中心±10范围视为0，避免机械抖动
 * 4. 队列通信：任务间解耦，数据安全传递
 * 5. 优先级调度：碰撞检测最高优先级，确保安全
 *
 * 资源占用：
 * - Flash: ~18KB / 64KB (28%)
 * - RAM:   ~7KB / 20KB (35%)
 * - 栈总计: ~3.5KB (6个任务)
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
/**
 * @brief 碰撞避障速度缩放因子（全局volatile变量）
 * @note  这两个变量由collisionTask01动态调整，motorTask01实时读取
 *        使用volatile确保编译器不优化，保证多任务间数据同步
 *
 *        变量范围: 40 ~ 100 (百分比)
 *        - 100% = 正常速度（无碰撞）
 *        - 40%  = 避障减速（发生碰撞）
 *
 *        平滑过渡: 每10ms调整20%，约30ms完成过渡
 */
volatile uint8_t leftMotorScale  = 100; // 左侧电机速度比例
volatile uint8_t rightMotorScale = 100; // 右侧电机速度比例

/**
 * @brief 碰撞检测功能开关（全局volatile变量）
 * @note  由ps2Task响应L3按键切换，collisionTask01读取此状态
 *        使用volatile确保多任务间数据同步
 *
 *        1 = 碰撞检测启用
 *        0 = 碰撞检测禁用（默认）
 */
volatile uint8_t collisionDetectionEnabled = 0; // 碰撞检测启用标志（默认关闭）

/**
 * @brief 顶部电机限位开关状态（全局volatile变量）
 * @note  由collisionTask02读取限位开关并更新，motorTask02读取此状态
 *        使用volatile确保多任务间数据同步
 *
 *        topLimitPressed = 1 表示上限位已触发，禁止正转
 *        bottomLimitPressed = 1 表示下限位已触发，禁止反转
 */
volatile uint8_t topLimitPressed    = 0; // 上限位开关状态（PA9）
volatile uint8_t bottomLimitPressed = 0; // 下限位开关状态（PA10）
/* USER CODE END Variables */
/* Definitions for ps2Task */
osThreadId_t ps2TaskHandle;
const osThreadAttr_t ps2Task_attributes = {
    .name       = "ps2Task",
    .stack_size = 256 * 4,
    .priority   = (osPriority_t)osPriorityAboveNormal,
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
/* Definitions for collisionTask01 */
osThreadId_t collisionTask01Handle;
const osThreadAttr_t collisionTask01_attributes = {
    .name       = "collisionTask01",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityHigh,
};
/* Definitions for collisionTask02 */
osThreadId_t collisionTask02Handle;
const osThreadAttr_t collisionTask02_attributes = {
    .name       = "collisionTask02",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityAboveNormal,
};
/* Definitions for motorTask02 */
osThreadId_t motorTask02Handle;
const osThreadAttr_t motorTask02_attributes = {
    .name       = "motorTask02",
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
void StartCollisionTask01(void *argument);
void StartCollisionTask02(void *argument);
void StartMotorTask02(void *argument);

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
    motorQueue01Handle = osMessageQueueNew(5, 14, &motorQueue01_attributes);

    /* creation of motorQueue02 */
    motorQueue02Handle = osMessageQueueNew(5, 2, &motorQueue02_attributes);

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

    /* creation of collisionTask01 */
    collisionTask01Handle = osThreadNew(StartCollisionTask01, NULL, &collisionTask01_attributes);

    /* creation of collisionTask02 */
    collisionTask02Handle = osThreadNew(StartCollisionTask02, NULL, &collisionTask02_attributes);

    /* creation of motorTask02 */
    motorTask02Handle = osThreadNew(StartMotorTask02, NULL, &motorTask02_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartPs2Task */
/**
 * @brief  PS2手柄数据采集与分发任务（全新混合控制架构）
 * @param  argument: Not used
 * @retval None
 * @note   任务优先级: osPriorityAboveNormal（次高优先级）
 *
 *         ========== 新控制方案架构 ==========
 *
 *         【底部电机】双摇杆混合控制 + 肩键辅助
 *         - motorQueue01: 左摇杆(0.6权重) + 右摇杆(0.4权重) + L1/L2(左轮) + R1/R2(右轮)
 *           * 左摇杆: leftY/leftX × 0.6（主控制）
 *           * 右摇杆: rightY/rightX × 0.4（辅助控制）
 *           * L1/L2: 控制左轮反转/正转（仅摇杆无输出时生效）
 *           * R1/R2: 控制右轮反转/正转（仅摇杆无输出时生效）
 *
 *         【舵机】方向键左侧控制
 *         - servoQueue: 方向键上/下（左侧按键）
 *           * 上键: 舵机正转
 *           * 下键: 舵机反转
 *
 *         【顶部电机】方向键右侧控制
 *         - motorQueue02: 三角形/X键（右侧按键）
 *           * 三角形: 顶部电机正转
 *           * X键: 顶部电机反转
 *
 *         【碰撞检测开关】L3按键（左摇杆按下）
 *         - L3按键: 切换碰撞检测功能的启用/禁用状态
 *           * 禁用（默认）: collisionTask01不工作，电机保持100%速度
 *           * 启用: collisionTask01正常工作，检测微动开关状态并自动纠偏
 *           * 防抖: 200ms防抖延迟，避免误触发
 *           * 建议: 需要自动纠偏时启用，平坦路面关闭以节省性能
 *
 *         数据包格式（motorQueue01 扩展为12字节）：
 *         [0-3]   uint32_t: 左摇杆数据 (高16位=leftY, 低16位=leftX)
 *         [4-7]   uint32_t: 右摇杆数据 (高16位=rightY, 低16位=rightX)
 *         [8-11]  uint32_t: 按键数据 (bit0=L1, bit1=L2, bit2=R1, bit3=R2, bit4=圆圈键)
 *
 *         安全保护机制：
 *         1. 初始化时发送锁死命令，确保上电后电机不误动作
 *         2. 只在模拟模式(0x79)或红灯模式(0x73)时处理摇杆数据
 *         3. 数字模式(0x41)时强制发送锁死命令
 *         4. PS2通信失败时发送锁死命令
 *
 *         采集周期: 20ms（50Hz）
 */
/* USER CODE END Header_StartPs2Task */
void StartPs2Task(void *argument)
{
    /* USER CODE BEGIN StartPs2Task */
    PS2_Data_t ps2Data;

    // 新数据包结构：12字节
    typedef struct {
        uint32_t leftJoystick;    // 左摇杆 (高16位=Y, 低16位=X)
        uint32_t rightJoystick;   // 右摇杆 (高16位=Y, 低16位=X)
        uint32_t shoulderButtons; // 肩键+圆圈键 (bit0=L1, bit1=L2, bit2=R1, bit3=R2, bit4=圆圈键)
    } MotorData_t;

    MotorData_t motorData;
    uint16_t buttonData; // 用于 motorQueue02 和 servoQueue（2字节）

    // L3按键状态跟踪（防抖）
    static uint8_t lastL3State      = 0;   // 上次L3按键状态
    static uint32_t lastL3Debounce  = 0;   // L3按键防抖时间
    const uint32_t L3_DEBOUNCE_TIME = 200; // L3按键防抖延迟（200ms）

    // 声明外部全局变量
    extern volatile uint8_t collisionDetectionEnabled;

    // 初始化 PS2 手柄（等待接收器和手柄建立连接）
    PS2_Init();

    // 初始化：发送"锁死"数据到所有队列
    motorData.leftJoystick    = 0;
    motorData.rightJoystick   = 0;
    motorData.shoulderButtons = 0;
    osMessageQueuePut(motorQueue01Handle, &motorData, 0, 0);

    buttonData = 0;
    osMessageQueuePut(servoQueueHandle, &buttonData, 0, 0);
    osMessageQueuePut(motorQueue02Handle, &buttonData, 0, 0);

    /* Infinite loop */
    for (;;) {
        // 读取 PS2 手柄数据
        if (PS2_ReadData(&ps2Data) == 0) {
            // ========== 0. 碰撞检测开关控制（L3按键） ==========
            uint8_t currentL3State = (ps2Data.buttons & PS2_L3) ? 1 : 0;
            uint32_t currentTime   = HAL_GetTick();

            // 检测到L3按键按下（边沿触发 + 防抖）
            if (currentL3State && !lastL3State) {
                if ((currentTime - lastL3Debounce) > L3_DEBOUNCE_TIME) {
                    // 切换碰撞检测状态
                    collisionDetectionEnabled = !collisionDetectionEnabled;
                    lastL3Debounce            = currentTime;
                }
            }
            lastL3State = currentL3State;

            // 检查PS2是否处于模拟模式（0x79）或红灯数字模式（0x73）
            if (ps2Data.mode == 0x79 || ps2Data.mode == 0x73) {
                // ========== 1. 底部电机数据包（双摇杆 + 肩键） ==========

                // 左摇杆数据转换（0-255 → -128~+127）
                int16_t leftY          = (int16_t)(ps2Data.leftY - 128);
                int16_t leftX          = (int16_t)(ps2Data.leftX - 128);
                motorData.leftJoystick = ((int32_t)leftY << 16) | (uint16_t)leftX;

                // 右摇杆数据转换（0-255 → -128~+127）
                int16_t rightY          = (int16_t)(ps2Data.rightY - 128);
                int16_t rightX          = (int16_t)(ps2Data.rightX - 128);
                motorData.rightJoystick = ((int32_t)rightY << 16) | (uint16_t)rightX;

                // 肩键数据（L1/L2控制左轮，R1/R2控制右轮，圆圈键控制锁止）
                motorData.shoulderButtons = 0;
                if (ps2Data.buttons & PS2_L1) motorData.shoulderButtons |= 0x01;
                if (ps2Data.buttons & PS2_L2) motorData.shoulderButtons |= 0x02;
                if (ps2Data.buttons & PS2_R1) motorData.shoulderButtons |= 0x04;
                if (ps2Data.buttons & PS2_R2) motorData.shoulderButtons |= 0x08;
                if (ps2Data.buttons & PS2_CIRCLE) motorData.shoulderButtons |= 0x10; // bit4=圆圈键

                osMessageQueueReset(motorQueue01Handle);
                osMessageQueuePut(motorQueue01Handle, &motorData, 0, 0);

                // ========== 2. 舵机控制（方向键上/下） ==========
                buttonData = 0;
                if (ps2Data.buttons & PS2_UP) buttonData |= 0x01;   // 方向键上
                if (ps2Data.buttons & PS2_DOWN) buttonData |= 0x02; // 方向键下

                osMessageQueueReset(servoQueueHandle);
                osMessageQueuePut(servoQueueHandle, &buttonData, 0, 0);

                // ========== 3. 顶部电机控制（三角形/X键） ==========
                buttonData = 0;
                if (ps2Data.buttons & PS2_TRIANGLE) buttonData |= 0x01; // 三角形键
                if (ps2Data.buttons & PS2_CROSS) buttonData |= 0x02;    // X键

                osMessageQueueReset(motorQueue02Handle);
                osMessageQueuePut(motorQueue02Handle, &buttonData, 0, 0);
            } else {
                // PS2处于数字模式（0x41）或其他异常模式，发送锁死命令
                motorData.leftJoystick    = 0;
                motorData.rightJoystick   = 0;
                motorData.shoulderButtons = 0;
                osMessageQueueReset(motorQueue01Handle);
                osMessageQueuePut(motorQueue01Handle, &motorData, 0, 0);

                buttonData = 0; // 无按键
                osMessageQueueReset(servoQueueHandle);
                osMessageQueuePut(servoQueueHandle, &buttonData, 0, 0);
                osMessageQueueReset(motorQueue02Handle);
                osMessageQueuePut(motorQueue02Handle, &buttonData, 0, 0);
            }
        } else {
            // PS2读取失败（连接断开或通信错误），发送锁死命令保护电机
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
 *         ========== 新控制方案 ==========
 *
 *         【摇杆混合控制】双摇杆加权平均
 *         - 左摇杆权重: 0.7（主控制）
 *         - 右摇杆权重: 0.3（辅助控制）
 *         - 最终速度 = (左摇杆结果 × 0.7) + (右摇杆结果 × 0.3)
 *
 *         算法流程：
 *         1. 左摇杆差速计算: L_left = leftY - leftX, L_right = leftY + leftX
 *         2. 右摇杆差速计算: R_left = rightY - rightX, R_right = rightY + rightX
 *         3. 加权混合: finalLeft = L_left × 0.7 + R_left × 0.3
 *                      finalRight = L_right × 0.7 + R_right × 0.3
 *         4. 动态归一化防溢出
 *
 *         【肩键辅助控制】优先级低于摇杆
 *         - L1: 左轮正转（仅当摇杆无输出时）
 *         - L2: 左轮反转（仅当摇杆无输出时）
 *         - R1: 右轮正转（仅当摇杆无输出时）
 *         - R2: 右轮反转（仅当摇杆无输出时）
 *         - 判断逻辑: 当左摇杆和右摇杆都在死区内时，才响应肩键
 *
 *         【圆圈键锁止模式】最高优先级
 *         - 按住圆圈键(○): 电机三阶段正反转平滑切换，实现物理锁止
 *         - 完整周期: 2秒（每个方向1个周期）
 *         - 三阶段模式:
 *           * 阶段1: 0→±64 (0.5s加速)
 *           * 阶段2: ±64→0 (0.5s减速)
 *           * 阶段3: 保持0速度 (0.5s停留)
 *           * 然后切换方向，重复循环
 *         - 目标速度: ±64（降低速度减少冲击）
 *         - 优先级: 高于摇杆和肩键，立即生效
 *         - 原理: 通过正反转交替+零速停留抵消惯性，防止车辆滑动
 *         - 实现: 状态机+线性插值，每10ms更新一次速度
 *
 *         死区处理：
 *         - 摇杆中心值128，转换后为0
 *         - 死区范围±10，避免机械抖动导致误触发
 *         - 摇杆在死区内时，电机主动锁死（IN1=HIGH, IN2=HIGH）
 *
 *         碰撞避障集成：
 *         - 速度值会乘以leftMotorScale/rightMotorScale（40%-100%）
 *         - 由collisionTask01动态调整，实现平滑避障
 *
 *         硬件连接：
 *         - 左电机: TIM2 CH2/CH3 (PA1/PA2) → L298N IN1/IN2
 *         - 右电机: TIM3 CH1/CH2 (PB4/PB5) → L298N IN1/IN2
 *         - PWM频率: 1kHz
 */
/* USER CODE END Header_StartMotorTask01 */
void StartMotorTask01(void *argument)
{
    /* USER CODE BEGIN StartMotorTask01 */
    // 启动TIM2 PWM通道2和3（左侧电机）
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

    // 启动TIM3 PWM通道1和2（右侧电机）
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    // 数据包结构定义（与ps2Task一致）
    typedef struct {
        uint32_t leftJoystick;    // 左摇杆
        uint32_t rightJoystick;   // 右摇杆
        uint32_t shoulderButtons; // 肩键
    } MotorData_t;

    MotorData_t motorData;
    int16_t leftY, leftX, rightY, rightX;      // 左右摇杆数据
    float leftSpeed_f, rightSpeed_f;           // 浮点计算中间值
    int16_t leftSpeed, rightSpeed;             // 计算后的左右电机速度
    int16_t scaledLeftSpeed, scaledRightSpeed; // 应用碰撞缩放后的速度
    uint32_t buttons;                          // 肩键数据

    const int16_t DEADZONE            = 10;   // 摇杆死区范围（±10）
    const int16_t BUTTON_SPEED        = 64;   // 肩键控制速度（50%最大值：127 × 0.5 ≈ 64）
    const int16_t LOCK_SPEED          = 64;   // 锁止模式目标速度（降低速度）
    const uint32_t LOCK_CYCLE_MS      = 2000; // 锁止模式切换周期（2000ms，正转→反转或反转→正转）
    const uint32_t LOCK_TRANSITION_MS = 500;  // 锁止模式平滑过渡时长（500ms）
    const uint32_t LOCK_HOLD_MS       = 500;  // 速度为0时的保持时长（500ms）

    // 锁止模式状态变量
    static uint32_t lockLastToggleTime  = 0;
    static int8_t lockDirection         = 1; // 目标方向: 1=正转, -1=反转
    static int16_t lockCurrentSpeed     = 0; // 当前锁止速度（平滑过渡中间值）
    static uint32_t lockTransitionStart = 0; // 过渡开始时间
    static int16_t lockStartSpeed       = 0; // 过渡起始速度
    static uint8_t lockPhase            = 0; // 当前阶段：0=加速到目标，1=减速到0，2=在0保持

    /* Infinite loop */
    for (;;) {
        // 尝试从队列读取电机数据（非阻塞）
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

            // ========== 3. 检测圆圈键锁止模式（最高优先级） ==========
            if (buttons & 0x10) { // 圆圈键按下（bit4）
                // 锁止模式：正反转平滑切换，速度为0时保持0.5s
                uint32_t currentTime = HAL_GetTick();

                // 检查是否到达周期切换时间（切换方向）
                if ((currentTime - lockLastToggleTime) >= LOCK_CYCLE_MS) {
                    lockDirection       = -lockDirection; // 切换目标方向
                    lockLastToggleTime  = currentTime;
                    lockTransitionStart = currentTime;
                    lockStartSpeed      = lockCurrentSpeed;
                    lockPhase           = 0; // 重置为加速阶段
                }

                uint32_t elapsedTime = currentTime - lockTransitionStart;

                // 状态机：0=加速到目标 → 1=减速到0 → 2=在0保持
                if (lockPhase == 0) {
                    // 阶段0：从0加速到目标速度（0.5s）
                    if (elapsedTime < LOCK_TRANSITION_MS) {
                        float progress      = (float)elapsedTime / (float)LOCK_TRANSITION_MS;
                        int16_t targetSpeed = LOCK_SPEED * lockDirection;
                        lockCurrentSpeed    = (int16_t)(lockStartSpeed + (targetSpeed - lockStartSpeed) * progress);
                    } else {
                        // 到达目标速度，进入减速阶段
                        lockCurrentSpeed    = LOCK_SPEED * lockDirection;
                        lockPhase           = 1;
                        lockTransitionStart = currentTime;
                        lockStartSpeed      = lockCurrentSpeed;
                    }
                } else if (lockPhase == 1) {
                    // 阶段1：从目标速度减速到0（0.5s）
                    if (elapsedTime < LOCK_TRANSITION_MS) {
                        float progress   = (float)elapsedTime / (float)LOCK_TRANSITION_MS;
                        lockCurrentSpeed = (int16_t)(lockStartSpeed + (0 - lockStartSpeed) * progress);
                    } else {
                        // 到达0速度，进入保持阶段
                        lockCurrentSpeed    = 0;
                        lockPhase           = 2;
                        lockTransitionStart = currentTime;
                    }
                } else if (lockPhase == 2) {
                    // 阶段2：在0保持（0.5s）
                    if (elapsedTime < LOCK_HOLD_MS) {
                        lockCurrentSpeed = 0;
                    } else {
                        // 保持结束，触发方向切换（回到周期判断）
                        lockLastToggleTime = currentTime - LOCK_CYCLE_MS; // 强制触发下一周期
                    }
                }

                // 两轮同时正反转（同方向，确保车体锁止）
                leftSpeed  = lockCurrentSpeed;
                rightSpeed = lockCurrentSpeed;

            } else {
                // 非锁止模式，重置状态
                lockLastToggleTime  = HAL_GetTick();
                lockDirection       = 1;
                lockCurrentSpeed    = 0;
                lockTransitionStart = HAL_GetTick();
                lockStartSpeed      = 0;
                lockPhase           = 0;

                // ========== 4. 判断是否使用摇杆控制 ==========
                int joystickActive = (leftY != 0 || leftX != 0 || rightY != 0 || rightX != 0);

                if (joystickActive) {
                    // ========== 摇杆模式：双摇杆混合控制 ==========

                    // 左摇杆差速计算
                    float L_left  = (float)leftY - (float)leftX;
                    float L_right = (float)leftY + (float)leftX;

                    // 右摇杆差速计算
                    float R_left  = (float)rightY - (float)rightX;
                    float R_right = (float)rightY + (float)rightX;

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

                    // 转换为整数
                    leftSpeed  = (int16_t)leftSpeed_f;
                    rightSpeed = (int16_t)rightSpeed_f;

                } else {
                    // ========== 肩键模式：仅当摇杆无输出时响应 ==========

                    leftSpeed  = 0;
                    rightSpeed = 0;

                    // L1/L2控制左轮（L1=正转，L2=反转）
                    if (buttons & 0x01) { // L1按下
                        leftSpeed = BUTTON_SPEED;
                    } else if (buttons & 0x02) { // L2按下
                        leftSpeed = -BUTTON_SPEED;
                    }

                    // R1/R2控制右轮（R1=正转，R2=反转）
                    if (buttons & 0x04) { // R1按下
                        rightSpeed = BUTTON_SPEED;
                    } else if (buttons & 0x08) { // R2按下
                        rightSpeed = -BUTTON_SPEED;
                    }
                }
            } // 结束圆圈键检测的else块

            // ========== 5. 最终安全限幅 ==========
            if (leftSpeed > 127) leftSpeed = 127;
            if (leftSpeed < -127) leftSpeed = -127;
            if (rightSpeed > 127) rightSpeed = 127;
            if (rightSpeed < -127) rightSpeed = -127;

            // ========== 6. 应用碰撞减速比例 ==========
            scaledLeftSpeed  = (int16_t)((int32_t)leftSpeed * leftMotorScale / 100);
            scaledRightSpeed = (int16_t)((int32_t)rightSpeed * rightMotorScale / 100);

            // ========== 7. 控制电机 ==========
            Motor_SetSpeedLeft(scaledLeftSpeed);
            Motor_SetSpeedRight(scaledRightSpeed);
        }

        osDelay(10); // 10ms延迟
    }
    /* USER CODE END StartMotorTask01 */
}

/* USER CODE BEGIN Header_StartCollisionTask01 */
/**
 * @brief  碰撞检测与避障控制任务（核心算法）
 * @param  argument: Not used
 * @retval None
 * @note   任务优先级: osPriorityHigh（最高优先级，确保及时响应碰撞）
 *
 *         硬件连接（NO常开型微动开关，常态为按下）：
 *         - PA3: 右侧微动开关 (COM→GND, NO→STM32 GPIO, 内部上拉)
 *         - PB6: 左侧微动开关 (COM→GND, NO→STM32 GPIO, 内部上拉)
 *
 *         逻辑状态（更新）：
 *         - GPIO_PIN_RESET (低电平) = 常态（按下状态，NO闭合接地）
 *         - GPIO_PIN_SET (高电平)   = 悬空触发（松开状态，NO断开，上拉电阻拉高）
 *
 *         避障策略（交叉控制）：
 *         - 右侧悬空 → 左侧电机减速（对侧控制） → 小车左转避障
 *         - 左侧悬空 → 右侧电机减速（对侧控制） → 小车右转避障
 *         - 两侧按下 → 电机速度恢复到100%
 *
 *         平滑过渡算法（加速）：
 *         - 目标速度: leftTargetScale / rightTargetScale (40% or 100%)
 *         - 当前速度: leftMotorScale / rightMotorScale (volatile变量)
 *         - 过渡步长: 20% 每10ms
 *         - 过渡时间: (100%-40%)/20% * 10ms = 30ms（快速响应）
 *
 *         防抖处理：
 *         - 防抖延迟: 50ms（避免开关机械抖动误触发）
 *         - 仅在状态变化且超过防抖时间后更新目标速度
 *
 *         功能开关：
 *         - 通过L3按键（左摇杆按下）切换碰撞检测启用/禁用
 *         - 禁用时自动恢复电机为100%速度
 *
 *         轮询周期: 10ms（保证快速响应和平滑过渡）
 */
/* USER CODE END Header_StartCollisionTask01 */
void StartCollisionTask01(void *argument)
{
    /* USER CODE BEGIN StartCollisionTask01 */
    extern volatile uint8_t leftMotorScale;
    extern volatile uint8_t rightMotorScale;
    extern volatile uint8_t collisionDetectionEnabled;

    GPIO_PinState rightSwitchState; // PA3 右侧微动开关当前状态
    GPIO_PinState leftSwitchState;  // PB6 左侧微动开关当前状态

    GPIO_PinState lastRightState = GPIO_PIN_RESET; // 上次状态，初始为按下（低电平）
    GPIO_PinState lastLeftState  = GPIO_PIN_RESET;

    uint32_t rightDebounceTime = 0; // 防抖计时器
    uint32_t leftDebounceTime  = 0;
    uint32_t currentTime;

    // 平滑过渡参数
    uint8_t leftTargetScale  = 100; // 左侧电机目标速度比例
    uint8_t rightTargetScale = 100; // 右侧电机目标速度比例

    const uint32_t DEBOUNCE_DELAY = 50;  // 50ms防抖时间
    const uint8_t COLLISION_SCALE = 40;  // 悬空时减速到40%
    const uint8_t NORMAL_SCALE    = 100; // 正常速度100%
    const uint8_t SMOOTH_STEP     = 20;  // 每次调整20%，快速过渡（30ms内完成）

    // 初始化为正常状态
    leftMotorScale  = 100;
    rightMotorScale = 100;

    /* Infinite loop */
    for (;;) {
        currentTime = HAL_GetTick();

        // 检查碰撞检测是否启用
        if (collisionDetectionEnabled) {
            // ========== 检测右侧微动开关 (PA3) ==========
            rightSwitchState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);

            // 状态发生变化且超过防抖时间
            if (rightSwitchState != lastRightState) {
                if ((currentTime - rightDebounceTime) > DEBOUNCE_DELAY) {
                    lastRightState    = rightSwitchState;
                    rightDebounceTime = currentTime;

                    if (rightSwitchState == GPIO_PIN_RESET) {
                        // 右侧按下（正常状态） -> 设置左侧电机目标为正常速度
                        leftTargetScale = NORMAL_SCALE;
                    } else {
                        // 右侧悬空（触发） -> 设置左侧电机目标为减速
                        leftTargetScale = COLLISION_SCALE;
                    }
                }
            }

            // ========== 检测左侧微动开关 (PB6) ==========
            leftSwitchState = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);

            // 状态发生变化且超过防抖时间
            if (leftSwitchState != lastLeftState) {
                if ((currentTime - leftDebounceTime) > DEBOUNCE_DELAY) {
                    lastLeftState    = leftSwitchState;
                    leftDebounceTime = currentTime;

                    if (leftSwitchState == GPIO_PIN_RESET) {
                        // 左侧按下（正常状态） -> 设置右侧电机目标为正常速度
                        rightTargetScale = NORMAL_SCALE;
                    } else {
                        // 左侧悬空（触发） -> 设置右侧电机目标为减速
                        rightTargetScale = COLLISION_SCALE;
                    }
                }
            }

            // ========== 平滑过渡到目标速度 ==========
            // 左侧电机平滑调整
            if (leftMotorScale < leftTargetScale) {
                // 当前速度低于目标，逐步加速
                leftMotorScale += SMOOTH_STEP;
                if (leftMotorScale > leftTargetScale) {
                    leftMotorScale = leftTargetScale; // 避免超调
                }
            } else if (leftMotorScale > leftTargetScale) {
                // 当前速度高于目标，逐步减速
                leftMotorScale -= SMOOTH_STEP;
                if (leftMotorScale < leftTargetScale) {
                    leftMotorScale = leftTargetScale; // 避免超调
                }
            }

            // 右侧电机平滑调整
            if (rightMotorScale < rightTargetScale) {
                // 当前速度低于目标，逐步加速
                rightMotorScale += SMOOTH_STEP;
                if (rightMotorScale > rightTargetScale) {
                    rightMotorScale = rightTargetScale; // 避免超调
                }
            } else if (rightMotorScale > rightTargetScale) {
                // 当前速度高于目标，逐步减速
                rightMotorScale -= SMOOTH_STEP;
                if (rightMotorScale < rightTargetScale) {
                    rightMotorScale = rightTargetScale; // 避免超调
                }
            }
        } else {
            // 碰撞检测禁用时，强制恢复为100%速度
            leftTargetScale  = NORMAL_SCALE;
            rightTargetScale = NORMAL_SCALE;
            leftMotorScale   = NORMAL_SCALE;
            rightMotorScale  = NORMAL_SCALE;
        }

        // 10ms轮询周期（快速过渡：从100%到40%需要约30ms，3次调整）
        osDelay(10);
    }
    /* USER CODE END StartCollisionTask01 */
}

/* USER CODE BEGIN Header_StartCollisionTask02 */
/**
 * @brief  顶部电机限位检测任务
 * @param  argument: Not used
 * @retval None
 * @note   任务优先级: osPriorityAboveNormal（次高优先级）
 *
 *         功能：检测顶部电机的上下限位开关，防止过度运行
 *
 *         硬件连接（三脚行程开关）：
 *         - PA9: 上部限位开关 (COM→GND, NO/NC→STM32 GPIO, 内部上拉)
 *         - PA10: 下部限位开关 (COM→GND, NO/NC→STM32 GPIO, 内部上拉)
 *
 *         逻辑状态（需要根据实际硬件测试确认）：
 *         - 如果接NO（常开）：
 *           * GPIO_PIN_SET = 未触发（开关未按下）
 *           * GPIO_PIN_RESET = 触发（开关按下，接地）
 *         - 如果接NC（常闭）：
 *           * GPIO_PIN_RESET = 未触发（开关未按下，接地）
 *           * GPIO_PIN_SET = 触发（开关按下，断开）
 *
 *         限位策略：
 *         - 上限位触发 → topLimitPressed = 1 → 禁止电机正转
 *         - 下限位触发 → bottomLimitPressed = 1 → 禁止电机反转
 *         - 两个限位都未触发 → 电机可自由运行
 *
 *         防抖处理：
 *         - 防抖延迟: 30ms（避免开关机械抖动误触发）
 *         - 仅在状态变化且超过防抖时间后更新状态
 *
 *         工作模式：
 *         - 仅当PS2控制顶部电机时才生效
 *         - motorTask02会读取这些状态并限制电机运动
 *
 *         轮询周期: 10ms（保证快速响应）
 */
/* USER CODE END Header_StartCollisionTask02 */
void StartCollisionTask02(void *argument)
{
    /* USER CODE BEGIN StartCollisionTask02 */
    extern volatile uint8_t topLimitPressed;
    extern volatile uint8_t bottomLimitPressed;

    GPIO_PinState topSwitchState;    // PA9 上限位开关当前状态
    GPIO_PinState bottomSwitchState; // PA10 下限位开关当前状态

    GPIO_PinState lastTopState    = GPIO_PIN_SET; // 上次状态（假设初始为未触发）
    GPIO_PinState lastBottomState = GPIO_PIN_SET;

    uint32_t topDebounceTime    = 0; // 防抖计时器
    uint32_t bottomDebounceTime = 0;
    uint32_t currentTime;

    const uint32_t DEBOUNCE_DELAY = 30; // 30ms防抖时间

    // 初始化为未触发状态
    topLimitPressed    = 0;
    bottomLimitPressed = 0;

    /* Infinite loop */
    for (;;) {
        currentTime = HAL_GetTick();

        // ========== 检测上限位开关 (PA9) ==========
        topSwitchState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);

        // 状态发生变化且超过防抖时间
        if (topSwitchState != lastTopState) {
            if ((currentTime - topDebounceTime) > DEBOUNCE_DELAY) {
                lastTopState    = topSwitchState;
                topDebounceTime = currentTime;

                // 根据实际硬件连接方式判断（这里假设接NO，触发时为低电平）
                // 如果实际是NC连接，需要反转判断逻辑
                if (topSwitchState == GPIO_PIN_RESET) {
                    // 上限位触发（开关按下）
                    topLimitPressed = 1;
                } else {
                    // 上限位释放（开关未按下）
                    topLimitPressed = 0;
                }
            }
        }

        // ========== 检测下限位开关 (PA10) ==========
        bottomSwitchState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);

        // 状态发生变化且超过防抖时间
        if (bottomSwitchState != lastBottomState) {
            if ((currentTime - bottomDebounceTime) > DEBOUNCE_DELAY) {
                lastBottomState    = bottomSwitchState;
                bottomDebounceTime = currentTime;

                // 根据实际硬件连接方式判断（这里假设接NO，触发时为低电平）
                // 如果实际是NC连接，需要反转判断逻辑
                if (bottomSwitchState == GPIO_PIN_RESET) {
                    // 下限位触发（开关按下）
                    bottomLimitPressed = 1;
                } else {
                    // 下限位释放（开关未按下）
                    bottomLimitPressed = 0;
                }
            }
        }

        // 10ms轮询周期
        osDelay(10);
    }
    /* USER CODE END StartCollisionTask02 */
}

/* USER CODE BEGIN Header_StartMotorTask02 */
/**
 * @brief  上部电机控制任务（三角形/X键控制 + 限位保护）
 * @param  argument: Not used
 * @retval None
 * @note   任务优先级: osPriorityNormal（普通优先级）
 *
 *         控制逻辑（带限位保护）：
 *         - 三角形键按下 → 正转最大速度 (+127)（除非上限位触发）
 *         - X键按下      → 反转最大速度 (-127)（除非下限位触发）
 *         - 两键同时按下 → 电机锁死 (0)
 *         - 无按键       → 电机锁死 (0)
 *
 *         限位保护（collisionTask02提供状态）：
 *         - topLimitPressed = 1 → 禁止正转，仅允许反转或停止
 *         - bottomLimitPressed = 1 → 禁止反转，仅允许正转或停止
 *         - 限位触发时强制电机速度为0，避免损坏机械结构
 *
 *         设计理念：
 *         - 上部电机通常只需要"开/关"两种状态，无需可调速
 *         - 正转用于收集/抓取，反转用于释放/清理
 *         - 锁死状态避免电机空转浪费电能
 *         - 限位开关提供硬件级保护，防止过度运行
 *
 *         硬件连接：
 *         - TIM3 CH3/CH4 (PB0/PB1) → L298N IN3/IN4
 *         - PWM频率: 1kHz
 *         - 最大速度: 127 (对应PWM占空比100%)
 */
/* USER CODE END Header_StartMotorTask02 */
void StartMotorTask02(void *argument)
{
    /* USER CODE BEGIN StartMotorTask02 */
    // 启动TIM3 PWM通道3和4（上部电机）
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    // 声明外部全局变量（限位开关状态）
    extern volatile uint8_t topLimitPressed;
    extern volatile uint8_t bottomLimitPressed;

    uint16_t buttonData = 0;
    int16_t motorSpeed  = 0;

    const int16_t MAX_SPEED = 127; // 最大速度

    /* Infinite loop */
    for (;;) {
        // 从队列接收三角形/X键状态（非阻塞）
        if (osMessageQueueGet(motorQueue02Handle, &buttonData, NULL, 0) == osOK) {
            // 检测三角形和X键状态
            uint8_t triangle_pressed = (buttonData & 0x01) ? 1 : 0; // 三角形键
            uint8_t x_pressed        = (buttonData & 0x02) ? 1 : 0; // X键

            // 控制逻辑
            if (triangle_pressed && x_pressed) {
                // 两键同时按下 -> 锁死
                motorSpeed = 0;
            } else if (triangle_pressed) {
                // 仅三角形键按下 -> 请求正转最大速度
                motorSpeed = MAX_SPEED;
            } else if (x_pressed) {
                // 仅X键按下 -> 请求反转最大速度
                motorSpeed = -MAX_SPEED;
            } else {
                // 两键都不按 -> 锁死
                motorSpeed = 0;
            }

            // ========== 限位保护逻辑 ==========
            // 仅当PS2有控制输入时才应用限位保护
            if (buttonData != 0) {
                if (motorSpeed > 0 && topLimitPressed) {
                    // 请求正转，但上限位已触发 -> 强制停止
                    motorSpeed = 0;
                } else if (motorSpeed < 0 && bottomLimitPressed) {
                    // 请求反转，但下限位已触发 -> 强制停止
                    motorSpeed = 0;
                }
            }

            // 控制上部电机
            Motor_SetSpeedTop(motorSpeed);
        }

        osDelay(10); // 10ms延迟
    }
    /* USER CODE END StartMotorTask02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
