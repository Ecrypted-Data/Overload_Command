/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    tim.c
 * @brief   This file provides code for the configuration
 *          of the TIM instances.
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
#include "tim.h"

/* USER CODE BEGIN 0 */
/**
 * ============================================================================
 * 定时器PWM控制模块 - 舵机与电机驱动
 * ============================================================================
 *
 * 本模块负责所有PWM信号生成,控制1个舵机和3组直流电机
 *
 * 硬件资源分配：
 * ┌─────────┬──────────────┬─────────────┬─────────┬────────────────┐
 * │ 定时器  │ 通道         │ GPIO引脚    │ 频率    │ 控制对象       │
 * ├─────────┼──────────────┼─────────────┼─────────┼────────────────┤
 * │ TIM1    │ CH1          │ PA8         │ 50Hz    │ SG90舵机       │
 * │ TIM2    │ CH1/CH2      │ PA0/PA1     │ 1kHz    │ 左侧电机(L298N)│
 * │ TIM2    │ CH3/CH4      │ PA2/PA3     │ 1kHz    │ 开合电机(L298N)│
 * │ TIM3    │ CH1/CH2      │ PB4/PB5     │ 1kHz    │ 右侧电机(L298N)│
 * │ TIM3    │ CH3/CH4      │ PB0/PB1     │ 1kHz    │ 升降电机(L298N)│
 * └─────────┴──────────────┴─────────────┴─────────┴────────────────┘
 *
 * 函数接口：
 * - Servo_SetAngle(angle)       : 设置舵机角度 0°~180°
 * - Motor_SetSpeedLeft(speed)   : 控制底部左侧电机 -127~+127
 * - Motor_SetSpeedRight(speed)  : 控制底部右侧电机 -127~+127
 * - Motor_SetSpeedTop(speed)    : 控制升降电机     -127~+127
 * - Motor_SetSpeedGrip(speed)   : 控制开合电机     -127~+127 (NEW)
 *
 * 电机控制逻辑（L298N H桥驱动）：
 * - speed > 0  : 正转（IN1=PWM, IN2=0）
 * - speed < 0  : 反转（IN1=0, IN2=PWM）
 * - speed = 0  : 锁死（IN1=HIGH, IN2=HIGH）主动刹车,快速停车
 *
 * 引脚映射变更记录（2025-11-07）：
 * - 底部左侧电机: PA0(IN1) / PA1(IN2) → TIM2 CH1/CH2 (L298N_L)
 * - 底部右侧电机: PB4(IN1) / PB5(IN2) → TIM3 CH1/CH2 (L298N_R)
 * - 升降电机:     PB0(IN3) / PB1(IN4) → TIM3 CH3/CH4 (L298N_R)
 * - 开合电机(新): PA2(IN3) / PA3(IN4) → TIM2 CH3/CH4 (L298N_L)
 *
 * 注意事项：
 * - 舵机PWM必须是50Hz（20ms周期）,脉宽0.5ms~2.5ms对应0°~180°
 * - 电机PWM使用1kHz,频率太低会有啸叫,太高会降低效率
 * - 主动刹车模式(IN1=IN2=HIGH)比自由滑行(IN1=IN2=LOW)制动效果好
 * - 所有引脚均无冲突：L298N_L使用PA0-PA3，L298N_R使用PB0/PB1/PB4/PB5
 */

/* USER CODE END 0 */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* TIM1 init function */
void MX_TIM1_Init(void)
{

    /* USER CODE BEGIN TIM1_Init 0 */

    /* USER CODE END TIM1_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig               = {0};
    TIM_OC_InitTypeDef sConfigOC                        = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 71;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 19999;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 0;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime         = 0;
    sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM1_Init 2 */

    /* USER CODE END TIM1_Init 2 */
    HAL_TIM_MspPostInit(&htim1);
}
/* TIM2 init function */
void MX_TIM2_Init(void)
{

    /* USER CODE BEGIN TIM2_Init 0 */

    /* USER CODE END TIM2_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC          = {0};

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 71;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 999;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */

    /* USER CODE END TIM2_Init 2 */
    HAL_TIM_MspPostInit(&htim2);
}
/* TIM3 init function */
void MX_TIM3_Init(void)
{

    /* USER CODE BEGIN TIM3_Init 0 */

    /* USER CODE END TIM3_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC          = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 71;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 999;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM3_Init 2 */

    /* USER CODE END TIM3_Init 2 */
    HAL_TIM_MspPostInit(&htim3);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *tim_pwmHandle)
{

    if (tim_pwmHandle->Instance == TIM1) {
        /* USER CODE BEGIN TIM1_MspInit 0 */

        /* USER CODE END TIM1_MspInit 0 */
        /* TIM1 clock enable */
        __HAL_RCC_TIM1_CLK_ENABLE();
        /* USER CODE BEGIN TIM1_MspInit 1 */

        /* USER CODE END TIM1_MspInit 1 */
    } else if (tim_pwmHandle->Instance == TIM2) {
        /* USER CODE BEGIN TIM2_MspInit 0 */

        /* USER CODE END TIM2_MspInit 0 */
        /* TIM2 clock enable */
        __HAL_RCC_TIM2_CLK_ENABLE();
        /* USER CODE BEGIN TIM2_MspInit 1 */

        /* USER CODE END TIM2_MspInit 1 */
    } else if (tim_pwmHandle->Instance == TIM3) {
        /* USER CODE BEGIN TIM3_MspInit 0 */

        /* USER CODE END TIM3_MspInit 0 */
        /* TIM3 clock enable */
        __HAL_RCC_TIM3_CLK_ENABLE();
        /* USER CODE BEGIN TIM3_MspInit 1 */

        /* USER CODE END TIM3_MspInit 1 */
    }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *timHandle)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (timHandle->Instance == TIM1) {
        /* USER CODE BEGIN TIM1_MspPostInit 0 */

        /* USER CODE END TIM1_MspPostInit 0 */
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**TIM1 GPIO Configuration
        PA8     ------> TIM1_CH1
        */
        GPIO_InitStruct.Pin   = SERVO_Pin;
        GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(SERVO_GPIO_Port, &GPIO_InitStruct);

        /* USER CODE BEGIN TIM1_MspPostInit 1 */

        /* USER CODE END TIM1_MspPostInit 1 */
    } else if (timHandle->Instance == TIM2) {
        /* USER CODE BEGIN TIM2_MspPostInit 0 */

        /* USER CODE END TIM2_MspPostInit 0 */

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**TIM2 GPIO Configuration
        PA0-WKUP     ------> TIM2_CH1
        PA1     ------> TIM2_CH2
        PA2     ------> TIM2_CH3
        PA3     ------> TIM2_CH4
        */
        GPIO_InitStruct.Pin   = L298N_L_IN1_Pin | L298N_L_IN2_Pin | L298N_L_IN3_Pin | L298N_L_IN4_Pin;
        GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* USER CODE BEGIN TIM2_MspPostInit 1 */

        /* USER CODE END TIM2_MspPostInit 1 */
    } else if (timHandle->Instance == TIM3) {
        /* USER CODE BEGIN TIM3_MspPostInit 0 */

        /* USER CODE END TIM3_MspPostInit 0 */

        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**TIM3 GPIO Configuration
        PB0     ------> TIM3_CH3
        PB1     ------> TIM3_CH4
        PB4     ------> TIM3_CH1
        PB5     ------> TIM3_CH2
        */
        GPIO_InitStruct.Pin   = L298N_R_IN3_Pin | L298N_R_IN4_Pin | L298N_R_IN1_Pin | L298N_R_IN2_Pin;
        GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        __HAL_AFIO_REMAP_TIM3_PARTIAL();

        /* USER CODE BEGIN TIM3_MspPostInit 1 */

        /* USER CODE END TIM3_MspPostInit 1 */
    }
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *tim_pwmHandle)
{

    if (tim_pwmHandle->Instance == TIM1) {
        /* USER CODE BEGIN TIM1_MspDeInit 0 */

        /* USER CODE END TIM1_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM1_CLK_DISABLE();
        /* USER CODE BEGIN TIM1_MspDeInit 1 */

        /* USER CODE END TIM1_MspDeInit 1 */
    } else if (tim_pwmHandle->Instance == TIM2) {
        /* USER CODE BEGIN TIM2_MspDeInit 0 */

        /* USER CODE END TIM2_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM2_CLK_DISABLE();
        /* USER CODE BEGIN TIM2_MspDeInit 1 */

        /* USER CODE END TIM2_MspDeInit 1 */
    } else if (tim_pwmHandle->Instance == TIM3) {
        /* USER CODE BEGIN TIM3_MspDeInit 0 */

        /* USER CODE END TIM3_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM3_CLK_DISABLE();
        /* USER CODE BEGIN TIM3_MspDeInit 1 */

        /* USER CODE END TIM3_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */

/**
 * @brief  设置舵机角度
 * @param  angle: 舵机角度 (0-180)
 * @retval None
 * @note   TIM1 配置为 50Hz PWM
 *         Prescaler = 71, Period = 19999
 *         实际频率 = 72MHz / (71+1) / (19999+1) = 50Hz
 *         周期 = 20ms
 *         舵机控制：0.5ms(500us) = 0°, 1.5ms(1500us) = 90°, 2.5ms(2500us) = 180°
 *         CCR 值计算：
 *         0° = 500us  / 20000us * 20000 = 500
 *         90° = 1500us / 20000us * 20000 = 1500
 *         180° = 2500us / 20000us * 20000 = 2500
 */
void Servo_SetAngle(uint16_t angle)
{
    uint16_t pulse;

    // 限制角度范围
    if (angle > 180) {
        angle = 180;
    }

    // 计算 PWM 脉冲宽度
    // pulse = 500 + (angle / 180) * 2000
    // 即：0° -> 500, 90° -> 1500, 180° -> 2500
    pulse = 500 + (angle * 2000) / 180;

    // 设置 PWM 占空比
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
}

/**
 * @brief  控制左侧电机（TIM2 CH1/CH2 -> PA0/PA1）
 * @param  speed: 速度值 (-127 到 +127)
 *                负值：反转，正值：正转，0：锁死（主动刹车）
 * @retval None
 * @note   TIM2: Prescaler=71, Period=999
 *         PWM频率 = 72MHz / 72 / 1000 = 1kHz
 *         L298N_L控制逻辑：
 *         IN1=HIGH, IN2=LOW  -> 正转
 *         IN1=LOW,  IN2=HIGH -> 反转
 *         IN1=HIGH, IN2=HIGH -> 锁死（主动刹车）
 *         IN1=LOW,  IN2=LOW  -> 自由滑行（不使用）
 */
void Motor_SetSpeedLeft(int16_t speed)
{
    uint16_t pwm_value;

    if (speed > 0) {
        // 正转：IN1(PA0/CH1)=PWM, IN2(PA1/CH2)=0
        pwm_value = (speed > 127) ? 999 : (speed * 999 / 127);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_value);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    } else if (speed < 0) {
        // 反转：IN1(PA0/CH1)=0, IN2(PA1/CH2)=PWM
        pwm_value = (speed < -127) ? 999 : ((-speed) * 999 / 127);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_value);
    } else {
        // 锁死：IN1=999, IN2=999（主动刹车）
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 999);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 999);
    }
}

/**
 * @brief  控制右侧电机（TIM3 CH1/CH2 -> PB4/PB5）
 * @param  speed: 速度值 (-127 到 +127)
 *                负值：反转，正值：正转，0：锁死（主动刹车）
 * @retval None
 * @note   TIM3: Prescaler=71, Period=999
 *         PWM频率 = 72MHz / 72 / 1000 = 1kHz
 *         L298N控制逻辑：
 *         IN1=HIGH, IN2=LOW  -> 正转
 *         IN1=LOW,  IN2=HIGH -> 反转
 *         IN1=HIGH, IN2=HIGH -> 锁死（主动刹车）
 *         IN1=LOW,  IN2=LOW  -> 自由滑行（不使用）
 */
void Motor_SetSpeedRight(int16_t speed)
{
    uint16_t pwm_value;

    if (speed > 0) {
        // 正转：IN1(PB4/CH1)=PWM, IN2(PB5/CH2)=0
        pwm_value = (speed > 127) ? 999 : (speed * 999 / 127);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_value);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    } else if (speed < 0) {
        // 反转：IN1(PB4/CH1)=0, IN2(PB5/CH2)=PWM
        pwm_value = (speed < -127) ? 999 : ((-speed) * 999 / 127);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_value);
    } else {
        // 锁死：IN1=999, IN2=999（主动刹车）
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 999);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 999);
    }
}

/**
 * @brief  控制上部电机（TIM3 CH3/CH4 -> PB0/PB1）
 * @param  speed: 速度值 (-127 到 +127)
 *                负值：反转，正值：正转，0：锁死（主动刹车）
 * @retval None
 * @note   TIM3: Prescaler=71, Period=999
 *         PWM频率 = 72MHz / 72 / 1000 = 1kHz
 *         L298N控制逻辑：
 *         IN3=HIGH, IN4=LOW  -> 正转
 *         IN3=LOW,  IN4=HIGH -> 反转
 *         IN3=HIGH, IN4=HIGH -> 锁死（主动刹车）
 *         IN3=LOW,  IN4=LOW  -> 自由滑行（不使用）
 */
void Motor_SetSpeedTop(int16_t speed)
{
    uint16_t pwm_value;

    if (speed > 0) {
        // 正转：IN3(PB0/CH3)=PWM, IN4(PB1/CH4)=0
        pwm_value = (speed > 127) ? 999 : (speed * 999 / 127);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm_value);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    } else if (speed < 0) {
        // 反转：IN3(PB0/CH3)=0, IN4(PB1/CH4)=PWM
        pwm_value = (speed < -127) ? 999 : ((-speed) * 999 / 127);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm_value);
    } else {
        // 锁死：IN3=999, IN4=999（主动刹车）
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 999);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 999);
    }
}

/**
 * @brief  控制开合电机（TIM2 CH3/CH4 -> PA2/PA3）
 * @param  speed: 速度值 (-127 到 +127)
 *                负值：反转（关闭），正值：正转（打开），0：锁死（主动刹车）
 * @retval None
 * @note   TIM2: Prescaler=71, Period=999
 *         PWM频率 = 72MHz / 72 / 1000 = 1kHz
 *         L298N_L控制逻辑：
 *         IN3=HIGH, IN4=LOW  -> 正转（打开）
 *         IN3=LOW,  IN4=HIGH -> 反转（关闭）
 *         IN3=HIGH, IN4=HIGH -> 锁死（主动刹车）
 *         IN3=LOW,  IN4=LOW  -> 自由滑行（不使用）
 */
void Motor_SetSpeedGrip(int16_t speed)
{
    uint16_t pwm_value;

    if (speed > 0) {
        // 正转（打开）：IN3(PA2/CH3)=PWM, IN4(PA3/CH4)=0
        pwm_value = (speed > 127) ? 999 : (speed * 999 / 127);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm_value);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
    } else if (speed < 0) {
        // 反转（关闭）：IN3(PA2/CH3)=0, IN4(PA3/CH4)=PWM
        pwm_value = (speed < -127) ? 999 : ((-speed) * 999 / 127);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm_value);
    } else {
        // 锁死：IN3=999, IN4=999（主动刹车）
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 999);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 999);
    }
} /* USER CODE END 1 */
