# PS2 手柄驱动说明

## 硬件连接

- **SPI1 SCK (PA5)** → PS2 手柄 CLK
- **SPI1 MISO (PA6)** → PS2 手柄 DATA (从手柄输出)
- **SPI1 MOSI (PA7)** → PS2 手柄 CMD (向手柄输入命令)
- **PA4 (GPIO)** → PS2 手柄 CS (片选，低电平有效)
- **VCC** → PS2 手柄 VCC (3.3V)
- **GND** → PS2 手柄 GND

## SPI 配置

- 模式：主机模式
- 时钟极性：高电平
- 时钟相位：第一边沿采样
- 波特率预分频：256（约 281.25 kHz）
- 数据位：8 位
- 位顺序：LSB 先行

## PS2 红灯模式（数字模式）

当前驱动配置为红灯模式（mode = 0x73），支持所有按键但不支持模拟量。
如需模拟模式（绿灯），mode 字段会返回 0x79，并且摇杆数据有效。

## 队列数据格式

**重要特性：队列数据始终保持最新**  
每次发送新数据前，会先清空队列（`osMessageQueueReset()`），确保接收端获取的永远是最新的 PS2 手柄状态，不会读到旧数据。

### 1. motorQueue01

- **数据类型**：`int32_t`（4 字节）
- **数据格式**：
  - 高 16 位（bits 31-16）：左摇杆 Y 轴（前后方向）
  - 低 16 位（bits 15-0）：右摇杆 Y 轴（前后方向）
- **数值范围**：-128 到 +127
  - 负值：向前推
  - 0：中心位置
  - 正值：向后拉

**示例代码（接收端）：**

```c
int32_t motorData;
if (osMessageQueueGet(motorQueue01Handle, &motorData, NULL, 100) == osOK)
{
    int16_t leftY = (int16_t)(motorData >> 16);   // 左摇杆前后
    int16_t rightY = (int16_t)(motorData & 0xFFFF); // 右摇杆前后

    // 使用摇杆数据控制电机
    // leftY: -128(全速前进) 到 +127(全速后退)
    // rightY: -128(全速前进) 到 +127(全速后退)
}
```

### 2. motorQueue02

- **数据类型**：`uint16_t`（2 字节）
- **数据格式**：
  - bit 0：L1 按键状态（前部左侧 1 号按键）
  - bit 1：L2 按键状态（前部左侧 2 号按键）
- **数值**：1 = 按下，0 = 释放

**示例代码（接收端）：**

```c
uint16_t leftButtons;
if (osMessageQueueGet(motorQueue02Handle, &leftButtons, NULL, 100) == osOK)
{
    if (leftButtons & 0x01) {
        // L1 按键按下
    }
    if (leftButtons & 0x02) {
        // L2 按键按下
    }
}
```

### 3. servoQueue

- **数据类型**：`uint16_t`（2 字节）
- **数据格式**：
  - bit 0：R1 按键状态（前部右侧 1 号按键）
  - bit 1：R2 按键状态（前部右侧 2 号按键）
- **数值**：1 = 按下，0 = 释放

**示例代码（接收端）：**

```c
uint16_t rightButtons;
if (osMessageQueueGet(servoQueueHandle, &rightButtons, NULL, 100) == osOK)
{
    if (rightButtons & 0x01) {
        // R1 按键按下
    }
    if (rightButtons & 0x02) {
        // R2 按键按下
    }
}
```

## PS2 手柄按键定义

```c
PS2_SELECT      // SELECT 按键
PS2_L3          // 左摇杆按下
PS2_R3          // 右摇杆按下
PS2_START       // START 按键
PS2_UP          // 方向键上
PS2_RIGHT       // 方向键右
PS2_DOWN        // 方向键下
PS2_LEFT        // 方向键左
PS2_L2          // 前部左侧2号按键（已使用）
PS2_R2          // 前部右侧2号按键（已使用）
PS2_L1          // 前部左侧1号按键（已使用）
PS2_R1          // 前部右侧1号按键（已使用）
PS2_TRIANGLE    // △ 按键
PS2_CIRCLE      // ○ 按键
PS2_CROSS       // × 按键
PS2_SQUARE      // □ 按键
```

## 任务配置

- **任务名称**：ps2Task
- **优先级**：osPriorityAboveNormal
- **堆栈大小**：1024 字节（256 \* 4）
- **采样频率**：50Hz（每 20ms 读取一次）

## 注意事项

1. PS2 手柄需要 3.3V 供电，请确保电压正确
2. 首次使用需要初始化延迟（100ms）
3. 如果读取失败，队列不会更新数据
4. 摇杆中心值为 128（原始值），转换后为 0
5. 红灯模式下，即使摇杆数据可读，但可能精度较低
6. CS 信号需要保持足够的建立和保持时间

## 调试建议

1. 使用示波器或逻辑分析仪检查 SPI 时序
2. 检查 PS2 手柄是否正确上电（红灯或绿灯）
3. 确认 SPI 配置与手柄匹配（LSB 先行，时钟极性/相位）
4. 可以在 ps2Task 中添加 LED 指示来显示连接状态
