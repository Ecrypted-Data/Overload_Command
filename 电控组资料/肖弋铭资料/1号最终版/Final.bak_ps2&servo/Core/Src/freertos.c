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

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of ps2Task */
    ps2TaskHandle = osThreadNew(StartPs2Task, NULL, &ps2Task_attributes);

    /* creation of servoTask */
    servoTaskHandle = osThreadNew(StartServoTask, NULL, &servoTask_attributes);

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

            // 2. 将前部左侧1号和2号按键状态写入 motorQueue02
            // L1 = 前部左侧1号按键，L2 = 前部左侧2号按键
            buttonData = 0;
            if (ps2Data.buttons & PS2_L1) buttonData |= 0x01; // L1按下
            if (ps2Data.buttons & PS2_L2) buttonData |= 0x02; // L2按下

            // 清空队列并写入最新数据
            osMessageQueueReset(motorQueue02Handle);
            osMessageQueuePut(motorQueue02Handle, &buttonData, 0, 0);

            // 3. 将前部右侧1号和2号按键状态写入 servoQueue
            // R1 = 前部右侧1号按键，R2 = 前部右侧2号按键
            buttonData = 0;
            if (ps2Data.buttons & PS2_R1) buttonData |= 0x01; // R1按下
            if (ps2Data.buttons & PS2_R2) buttonData |= 0x02; // R2按下

            // 清空队列并写入最新数据
            osMessageQueueReset(servoQueueHandle);
            osMessageQueuePut(servoQueueHandle, &buttonData, 0, 0);
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
            // 检测按键冲突：R1 和 R2 同时按下时停止转动
            if ((buttonData & 0x01) && (buttonData & 0x02)) {
                // 两个按键同时按下，舵机停止（不改变角度）
            }
            // R1 按键按下 -> 向 0° 转动
            else if (buttonData & 0x01) {
                if (currentAngle > 1) {
                    currentAngle -= 2; // 每次减少 2 度
                } else {
                    currentAngle = 0; // 接近边界时直接到 0
                }
            }
            // R2 按键按下 -> 向 180° 转动
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

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
