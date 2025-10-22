/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : ps2.c
 * @brief          : PS2 controller driver implementation
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "ps2.h"
#include "cmsis_os.h"

/* Private variables ---------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;

/**
 * @brief  Delay in microseconds (approximate)
 * @param  us: Microseconds to delay
 * @retval None
 */
static void delay_us(uint32_t us)
{
    uint32_t ticks = us * (SystemCoreClock / 1000000) / 5;
    while (ticks--) {
        __NOP();
    }
}

/**
 * @brief  Initialize PS2 controller
 * @param  None
 * @retval None
 */
void PS2_Init(void)
{
    PS2_CS_HIGH();
    osDelay(500); // 等待接收器和手柄稳定连接
}

/**
 * @brief  Send command to PS2 controller via SPI
 * @param  txData: Pointer to transmit buffer
 * @param  rxData: Pointer to receive buffer
 * @param  length: Number of bytes to transfer
 * @retval Status: 0=Success, 1=Error
 */
uint8_t PS2_SendCommand(uint8_t *txData, uint8_t *rxData, uint8_t length)
{
    uint8_t i;

    // 确保 CS 为高电平至少 10us
    PS2_CS_HIGH();
    delay_us(10);

    // 拉低 CS，开始通信
    PS2_CS_LOW();
    delay_us(20); // CS setup time (至少 16us)

    // 逐字节发送接收（更稳定的方式）
    for (i = 0; i < length; i++) {
        if (HAL_SPI_TransmitReceive(&hspi1, &txData[i], &rxData[i], 1, 100) != HAL_OK) {
            PS2_CS_HIGH();
            return 1;
        }
        delay_us(10); // 字节间延时
    }

    delay_us(20); // CS hold time
    PS2_CS_HIGH();
    delay_us(20); // 帧间延时

    return 0;
}

/**
 * @brief  Read data from PS2 controller
 * @param  ps2Data: Pointer to PS2_Data_t structure to store data
 * @retval Status: 0=Success, 1=Error
 */
uint8_t PS2_ReadData(PS2_Data_t *ps2Data)
{
    uint8_t txBuf[9] = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t rxBuf[9] = {0};
    uint8_t retry    = 0;
    uint8_t result;

    // 最多重试 3 次
    for (retry = 0; retry < 3; retry++) {
        result = PS2_SendCommand(txBuf, rxBuf, 9);

        if (result == 0) {
            // 检查响应有效性（ID 字段应该是 0x41, 0x73 或 0x79）
            if (rxBuf[1] == 0x41 || rxBuf[1] == 0x73 || rxBuf[1] == 0x79) {
                // 解析数据
                ps2Data->header  = rxBuf[0];
                ps2Data->mode    = rxBuf[1];
                ps2Data->buttons = (uint16_t)(~((rxBuf[4] << 8) | rxBuf[3])); // 按键状态取反（按下为1）
                ps2Data->rightX  = rxBuf[5];
                ps2Data->rightY  = rxBuf[6];
                ps2Data->leftX   = rxBuf[7];
                ps2Data->leftY   = rxBuf[8];

                return 0; // 成功
            }
        }

        // 失败后稍微延时再重试
        osDelay(2);
    }

    return 1; // 所有重试都失败
}
