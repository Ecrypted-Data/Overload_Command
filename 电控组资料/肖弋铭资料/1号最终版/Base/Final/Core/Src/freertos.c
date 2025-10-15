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
  .name = "ps2Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for motorTask01 */
osThreadId_t motorTask01Handle;
const osThreadAttr_t motorTask01_attributes = {
  .name = "motorTask01",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for motorTask02 */
osThreadId_t motorTask02Handle;
const osThreadAttr_t motorTask02_attributes = {
  .name = "motorTask02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for servoTask */
osThreadId_t servoTaskHandle;
const osThreadAttr_t servoTask_attributes = {
  .name = "servoTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for xQueuePs2 */
osMessageQueueId_t xQueuePs2Handle;
const osMessageQueueAttr_t xQueuePs2_attributes = {
  .name = "xQueuePs2"
};
/* Definitions for xQueueCollision */
osMessageQueueId_t xQueueCollisionHandle;
const osMessageQueueAttr_t xQueueCollision_attributes = {
  .name = "xQueueCollision"
};
/* Definitions for xQueueLimit */
osMessageQueueId_t xQueueLimitHandle;
const osMessageQueueAttr_t xQueueLimit_attributes = {
  .name = "xQueueLimit"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartPs2Task(void *argument);
void StartMotorTask01(void *argument);
void StartMotorTask02(void *argument);
void StartServoTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
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
  /* creation of xQueuePs2 */
  xQueuePs2Handle = osMessageQueueNew (10, 8, &xQueuePs2_attributes);

  /* creation of xQueueCollision */
  xQueueCollisionHandle = osMessageQueueNew (5, 4, &xQueueCollision_attributes);

  /* creation of xQueueLimit */
  xQueueLimitHandle = osMessageQueueNew (3, 4, &xQueueLimit_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ps2Task */
  ps2TaskHandle = osThreadNew(StartPs2Task, NULL, &ps2Task_attributes);

  /* creation of motorTask01 */
  motorTask01Handle = osThreadNew(StartMotorTask01, NULL, &motorTask01_attributes);

  /* creation of motorTask02 */
  motorTask02Handle = osThreadNew(StartMotorTask02, NULL, &motorTask02_attributes);

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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartPs2Task */
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartMotorTask01 */
}

/* USER CODE BEGIN Header_StartMotorTask02 */
/**
* @brief Function implementing the motorTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorTask02 */
void StartMotorTask02(void *argument)
{
  /* USER CODE BEGIN StartMotorTask02 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartMotorTask02 */
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartServoTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

