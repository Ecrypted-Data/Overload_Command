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
#include "queue.h"
#include "ax_ps2.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
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
QueueHandle_t ps2QueueHandle;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name       = "defaultTask",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartPs2Task(void *argument);
void StartCdcTask(void *argument);
void StartPs2Task(void *argument);
void StartCdcTask(void *argument);
void Motor_SetSpeed(int16_t speed);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
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

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    ps2QueueHandle = xQueueCreate(8, sizeof(JOYSTICK_TypeDef));
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of defaultTask */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    // 任务句柄
    osThreadId_t ps2TaskHandle;
    osThreadId_t cdcTaskHandle;

    // 属性
    const osThreadAttr_t ps2Task_attributes = {
        .name       = "ps2Task",
        .stack_size = 256 * 4,
        .priority   = (osPriority_t)osPriorityHigh,
    };
    const osThreadAttr_t cdcTask_attributes = {
        .name       = "cdcTask",
        .stack_size = 256 * 4,
        .priority   = (osPriority_t)osPriorityNormal,
    };

    // 创建任务
    ps2TaskHandle = osThreadNew(StartPs2Task, NULL, &ps2Task_attributes);
    cdcTaskHandle = osThreadNew(StartCdcTask, NULL, &cdcTask_attributes);
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
    /* init code for USB_DEVICE */
    MX_USB_DEVICE_Init();
    /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop */
    for (;;) {
        osDelay(1);
    }
    /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void StartPs2Task(void *argument)
{
    JOYSTICK_TypeDef js;
    AX_PS2_Init();
    for (;;) {
        AX_PS2_ScanKey(&js);
        xQueueSend(ps2QueueHandle, &js, 0);
        osDelay(20); // 20ms 周期
    }
}
void StartCdcTask(void *argument)
{
    JOYSTICK_TypeDef js;
    char buf[128];
    for (;;) {
        if (xQueueReceive(ps2QueueHandle, &js, portMAX_DELAY) == pdPASS) {
            // 1. 计算速度
            int16_t speed = 0;
            if (js.LJoy_UD < 100) {
                speed = (100 - js.LJoy_UD) * 10; // 前进
            } else if (js.LJoy_UD > 150) {
                speed = -(js.LJoy_UD - 150) * 10; // 后退
            } else {
                speed = 0; // 停止
            }

            // 限幅到 -999 ~ +999
            if (speed > 999) speed = 999;
            if (speed < -999) speed = -999;

            // 2. 设置电机
            Motor_SetSpeed(speed);

            // 3. 发送到 PC
            int len = snprintf(buf, sizeof(buf),
                               "LY=%02X speed=%d\r\n", js.LJoy_UD, speed);
            CDC_Transmit_FS((uint8_t *)buf, len);
        }
    }
}

extern TIM_HandleTypeDef htim3;
void Motor_SetSpeed(int16_t speed)
{
    // speed 范围：-999 ~ +999
    if (speed > 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    } else if (speed < 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, -speed);
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    }
}
/* USER CODE END Application */
