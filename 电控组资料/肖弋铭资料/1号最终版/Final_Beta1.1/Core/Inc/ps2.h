/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : ps2.h
 * @brief          : Header for ps2.c file.
 *                   This file contains the PS2 controller driver definitions.
 ******************************************************************************
 */
/* USER CODE END Header */

#ifndef __PS2_H
#define __PS2_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"

/* PS2 Controller Defines ----------------------------------------------------*/
#define PS2_CS_PORT   GPIOA
#define PS2_CS_PIN    GPIO_PIN_4

#define PS2_CS_LOW()  HAL_GPIO_WritePin(PS2_CS_PORT, PS2_CS_PIN, GPIO_PIN_RESET)
#define PS2_CS_HIGH() HAL_GPIO_WritePin(PS2_CS_PORT, PS2_CS_PIN, GPIO_PIN_SET)

/* PS2 Commands */
#define PS2_CMD_INIT        0x01
#define PS2_CMD_POLL        0x42
#define PS2_CMD_CONFIG_MODE 0x43
#define PS2_CMD_ANALOG_MODE 0x44
#define PS2_CMD_VIBRATION   0x4D

/* PS2 Button Masks */
#define PS2_SELECT   0x0001
#define PS2_L3       0x0002
#define PS2_R3       0x0004
#define PS2_START    0x0008
#define PS2_UP       0x0010
#define PS2_RIGHT    0x0020
#define PS2_DOWN     0x0040
#define PS2_LEFT     0x0080
#define PS2_L2       0x0100
#define PS2_R2       0x0200
#define PS2_L1       0x0400
#define PS2_R1       0x0800
#define PS2_TRIANGLE 0x1000
#define PS2_CIRCLE   0x2000
#define PS2_CROSS    0x4000
#define PS2_SQUARE   0x8000

/* PS2 Data Structure */
typedef struct {
    uint8_t header;   // 数据头
    uint8_t mode;     // 模式：0x41=数字模式，0x73=红灯模式（数字），0x79=模拟模式
    uint16_t buttons; // 按键状态（按下为0）
    uint8_t rightX;   // 右摇杆 X 轴（0-255，中心128）
    uint8_t rightY;   // 右摇杆 Y 轴（0-255，中心128）
    uint8_t leftX;    // 左摇杆 X 轴（0-255，中心128）
    uint8_t leftY;    // 左摇杆 Y 轴（0-255，中心128）
} PS2_Data_t;

/* Function Prototypes -------------------------------------------------------*/
void PS2_Init(void);
uint8_t PS2_ReadData(PS2_Data_t *ps2Data);
uint8_t PS2_SendCommand(uint8_t *txData, uint8_t *rxData, uint8_t length);

#ifdef __cplusplus
}
#endif

#endif /* __PS2_H */
