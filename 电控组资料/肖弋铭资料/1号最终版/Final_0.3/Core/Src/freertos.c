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
// 碰撞减速比例控制变量（百分比：0-100）
volatile uint8_t leftMotorScale  = 100; // 左侧电机速度比例
volatile uint8_t rightMotorScale = 100; // 右侧电机速度比例
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
/* Definitions for collisionQueue02 */
osMessageQueueId_t collisionQueue02Handle;
const osMessageQueueAttr_t collisionQueue02_attributes = {
    .name = "collisionQueue02"};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartPs2Task(void *argument);
void StartServoTask(void *argument);
void StartMotorTask01(void *argument);
void StartCollisionTask01(void *argument);
void StartCollisionTask02(void *argument);

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
    motorQueue01Handle = osMessageQueueNew(3, 4, &motorQueue01_attributes);

    /* creation of motorQueue02 */
    motorQueue02Handle = osMessageQueueNew(5, 2, &motorQueue02_attributes);

    /* creation of servoQueue */
    servoQueueHandle = osMessageQueueNew(5, 2, &servoQueue_attributes);

    /* creation of collisionQueue02 */
    collisionQueue02Handle = osMessageQueueNew(3, 1, &collisionQueue02_attributes);

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

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartPs2Task */
/**
 * @brief  Function implementing the ps2Task thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartPs2Task */
void StartPs2Task(void *argument)
{
    /* USER CODE BEGIN StartPs2Task */
    PS2_Data_t ps2Data;
    int32_t motorData;   // 用于 motorQueue01（4字节）
    uint16_t buttonData; // 用于 motorQueue02 和 servoQueue（2字节）

    // 初始化 PS2 手柄（等待接收器和手柄建立连接）
    PS2_Init();

    /* Infinite loop */
    for (;;) {
        // 读取 PS2 手柄数据
        if (PS2_ReadData(&ps2Data) == 0) {
            // 1. 将左右摇杆的前后方向数据写入 motorQueue01
            // 左摇杆 Y 轴（前后）：ps2Data.leftY（0-255，中心128，向前<128，向后>128）
            // 右摇杆 Y 轴（前后）：ps2Data.rightY（0-255，中心128，向前<128，向后>128）
            // 转换为有符号值：-128 到 +127（向前为负，向后为正）
            int16_t leftY  = (int16_t)(ps2Data.leftY - 128);
            int16_t rightY = (int16_t)(ps2Data.rightY - 128);

            // 合并为一个 int32_t：高16位=左摇杆，低16位=右摇杆
            motorData = ((int32_t)leftY << 16) | (uint16_t)rightY;

            // 清空队列并写入最新数据，确保队列中数据始终为最新
            osMessageQueueReset(motorQueue01Handle);
            osMessageQueuePut(motorQueue01Handle, &motorData, 0, 0);

            // 2. 将前部左侧1号和2号按键状态写入 servoQueue（控制舵机）
            // L1 = 前部左侧1号按键，L2 = 前部左侧2号按键
            buttonData = 0;
            if (ps2Data.buttons & PS2_L1) buttonData |= 0x01; // L1按下
            if (ps2Data.buttons & PS2_L2) buttonData |= 0x02; // L2按下

            // 清空队列并写入最新数据
            osMessageQueueReset(servoQueueHandle);
            osMessageQueuePut(servoQueueHandle, &buttonData, 0, 0);

            // 3. 将前部右侧1号和2号按键状态写入 motorQueue02（预留给上部电机）
            // R1 = 前部右侧1号按键，R2 = 前部右侧2号按键
            buttonData = 0;
            if (ps2Data.buttons & PS2_R1) buttonData |= 0x01; // R1按下
            if (ps2Data.buttons & PS2_R2) buttonData |= 0x02; // R2按下

            // 清空队列并写入最新数据
            osMessageQueueReset(motorQueue02Handle);
            osMessageQueuePut(motorQueue02Handle, &buttonData, 0, 0);
        }

        osDelay(20);
    }
    /* USER CODE END StartPs2Task */
}

/* USER CODE BEGIN Header_StartServoTask */
/**
 * @brief Function implementing the servoTask thread.
 * @param argument: Not used
 * @retval None
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
            // 检测按键冲突：L1 和 L2 同时按下时停止转动
            if ((buttonData & 0x01) && (buttonData & 0x02)) {
                // 两个按键同时按下，舵机停止（不改变角度）
            }
            // L1 按键按下 -> 向 0° 转动
            else if (buttonData & 0x01) {
                if (currentAngle > 1) {
                    currentAngle -= 2; // 每次减少 2 度
                } else {
                    currentAngle = 0; // 接近边界时直接到 0
                }
            }
            // L2 按键按下 -> 向 180° 转动
            else if (buttonData & 0x02) {
                if (currentAngle < 179) {
                    currentAngle += 2; // 每次增加 2 度
                } else {
                    currentAngle = 180; // 接近边界时直接到 180
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
 * @brief Function implementing the motorTask01 thread.
 * @param argument: Not used
 * @retval None
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

    int32_t motorData;
    int16_t leftY, rightY;
    int16_t scaledLeftY, scaledRightY;

    /* Infinite loop */
    for (;;) {
        // 尝试从队列读取电机数据（非阻塞）
        if (osMessageQueueGet(motorQueue01Handle, &motorData, NULL, 0) == osOK) {
            // 解包数据：高16位是左摇杆Y轴，低16位是右摇杆Y轴
            leftY  = (int16_t)((motorData >> 16) & 0xFFFF);
            rightY = (int16_t)(motorData & 0xFFFF);

            // 应用碰撞减速比例
            scaledLeftY  = (int16_t)((int32_t)leftY * leftMotorScale / 100);
            scaledRightY = (int16_t)((int32_t)rightY * rightMotorScale / 100);

            // 控制左右电机
            Motor_SetSpeedLeft(scaledLeftY);
            Motor_SetSpeedRight(scaledRightY);
        }

        osDelay(10); // 10ms延迟，与ps2Task的20ms读取周期配合
    }
    /* USER CODE END StartMotorTask01 */
}

/* USER CODE BEGIN Header_StartCollisionTask01 */
/**
 * @brief Function implementing the collisionTask01 thread.
 * @param argument: Not used
 * @retval None
 *
 * 轮询方式检测微动开关状态（NO常开型，已配置上拉）：
 * 硬件连接: NO端接STM32, COM端接GND
 * - PA3 (右侧微动开关): HIGH=正常(NO断开), LOW=碰撞(NO闭合接地)
 * - PB6 (左侧微动开关): HIGH=正常(NO断开), LOW=碰撞(NO闭合接地)
 *
 * 控制逻辑（快速平滑过渡）:
 * - 右侧碰撞(LOW) -> 左侧电机快速减速到40% (约0.5s)
 * - 左侧碰撞(LOW) -> 右侧电机快速减速到40% (约0.5s)
 * - 恢复正常(HIGH) -> 电机快速恢复到100% (约0.5s)
 */
/* USER CODE END Header_StartCollisionTask01 */
void StartCollisionTask01(void *argument)
{
    /* USER CODE BEGIN StartCollisionTask01 */
    extern volatile uint8_t leftMotorScale;
    extern volatile uint8_t rightMotorScale;

    GPIO_PinState rightSwitchState; // PA3 右侧微动开关当前状态
    GPIO_PinState leftSwitchState;  // PB6 左侧微动开关当前状态

    GPIO_PinState lastRightState = GPIO_PIN_SET; // 上次状态，初始假设为上拉的高电平
    GPIO_PinState lastLeftState  = GPIO_PIN_SET;

    uint32_t rightDebounceTime = 0; // 防抖计时器
    uint32_t leftDebounceTime  = 0;
    uint32_t currentTime;

    // 平滑过渡参数
    uint8_t leftTargetScale  = 100; // 左侧电机目标速度比例
    uint8_t rightTargetScale = 100; // 右侧电机目标速度比例

    const uint32_t DEBOUNCE_DELAY = 50;  // 50ms防抖时间
    const uint8_t COLLISION_SCALE = 40;  // 碰撞时减速到40%（增大减速幅度）
    const uint8_t NORMAL_SCALE    = 100; // 正常速度100%
    const uint8_t SMOOTH_STEP     = 12;  // 每次调整12%，快速过渡（0.5s内完成）

    // 初始化为正常状态
    leftMotorScale  = 100;
    rightMotorScale = 100;

    /* Infinite loop */
    for (;;) {
        currentTime = HAL_GetTick();

        // ========== 检测右侧微动开关 (PA3) ==========
        rightSwitchState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);

        // 状态发生变化且超过防抖时间
        if (rightSwitchState != lastRightState) {
            if ((currentTime - rightDebounceTime) > DEBOUNCE_DELAY) {
                lastRightState    = rightSwitchState;
                rightDebounceTime = currentTime;

                if (rightSwitchState == GPIO_PIN_SET) {
                    // 右侧开关正常状态 -> 设置左侧电机目标为正常速度
                    leftTargetScale = NORMAL_SCALE;
                } else {
                    // 右侧开关触发（碰撞发生） -> 设置左侧电机目标为减速
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

                if (leftSwitchState == GPIO_PIN_SET) {
                    // 左侧开关正常状态 -> 设置右侧电机目标为正常速度
                    rightTargetScale = NORMAL_SCALE;
                } else {
                    // 左侧开关触发（碰撞发生） -> 设置右侧电机目标为减速
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

        // 10ms轮询周期（快速过渡：从100%到40%需要约50ms，5次调整）
        osDelay(10);
    }
    /* USER CODE END StartCollisionTask01 */
} /* USER CODE BEGIN Header_StartCollisionTask02 */
/**
 * @brief Function implementing the collisionTask02 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartCollisionTask02 */
void StartCollisionTask02(void *argument)
{
    /* USER CODE BEGIN StartCollisionTask02 */
    /* Infinite loop */
    for (;;) {
        osDelay(1);
    }
    /* USER CODE END StartCollisionTask02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
