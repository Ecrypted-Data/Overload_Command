/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define L298N_L_IN1_Pin GPIO_PIN_1
#define L298N_L_IN1_GPIO_Port GPIOA
#define L298N_L_IN2_Pin GPIO_PIN_2
#define L298N_L_IN2_GPIO_Port GPIOA
#define KW8_3_R_Pin GPIO_PIN_3
#define KW8_3_R_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define SERVO_Pin GPIO_PIN_8
#define SERVO_GPIO_Port GPIOA
#define L298N_R_IN1_Pin GPIO_PIN_4
#define L298N_R_IN1_GPIO_Port GPIOB
#define L298N_R_IN2_Pin GPIO_PIN_5
#define L298N_R_IN2_GPIO_Port GPIOB
#define KW8_3_L_Pin GPIO_PIN_6
#define KW8_3_L_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define PS2_CS_Pin       GPIO_PIN_4
#define PS2_CS_GPIO_Port GPIOA
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
