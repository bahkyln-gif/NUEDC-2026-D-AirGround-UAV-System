# 小车硬件接线与引脚分配（STM32F103C8T6 + HAL）

> 以 STM32CubeMX 工程为准，下面为推荐引脚。`firmware/car` 源码与之一致。

## 1. 引脚总表

| 功能 | 引脚 | 外设 | 说明 |
|---|---|---|---|
| 左电机 PWM | PA0 | TIM2_CH1 | |
| 左电机方向 | PA1 / PA2 | GPIO | TB6612 AIN1/AIN2 |
| 右电机 PWM | PA3 | TIM2_CH4 | |
| 右电机方向 | PA4 / PA5 | GPIO | TB6612 BIN1/BIN2 |
| 左编码器 A/B | PA6 / PA7 | TIM3_CH1/CH2 | 编码器模式（4 倍频） |
| 右编码器 A/B | PB6 / PB7 | TIM4_CH1/CH2 | 编码器模式（4 倍频） |
| 灰度 1~8 | PB8~PB15 | GPIO 输入（上拉） | 8 路数字量 |
| 启动按键 | PA8 | EXTI9_5 | 短按启动/长按切模式 |
| 蜂鸣器 | PA11 | GPIO 输出 | 提示音 |
| ESP32 TX | PA9 | USART1_TX | 接 ESP32 RXD |
| ESP32 RX | PA10 | USART1_RX | 接 ESP32 TXD |
| 状态 LED | PC13 | GPIO | 板载 |

## 2. 模块接线

### 2.1 灰度传感器（8 路 TCRT5000 数字输出）

```
VCC ── 5V        GND ── GND
D1~D8 ── PB8~PB15（上拉输入，检测到黑线输出低/高视模块而定，软件内可配置极性）
```

### 2.2 TB6612 电机驱动

```
VM ── 电池 7.4V      VCC ── 5V       GND ── GND
PWMA ── PA0(PWM)    AIN1 ── PA1     AIN2 ── PA2
PWMB ── PA3(PWM)    BIN1 ── PA4     BIN2 ── PA5
AO1/AO2 ── 左电机    BO1/BO2 ── 右电机
STBY ── 3.3V
```

### 2.3 编码器电机

```
左电机编码器 A ── PA6(TIM3_CH1)    B ── PA7(TIM3_CH2)
右电机编码器 A ── PB6(TIM4_CH1)    B ── PB7(TIM4_CH2)
VCC ── 3.3V    GND ── GND
```

### 2.4 ESP32-01S（WiFi 透传）

```
ESP32 RXD ── PA9 (USART1_TX)
ESP32 TXD ── PA10 (USART1_RX)
VCC ── 3.3V    GND ── GND    EN ── 3.3V（10k 上拉）
```

ESP32 侧使用 AT 固件（`AT+CIPMUX=0`，`AT+CIPSTART="UDP",...`）或刷写透传固件，波特率 115200。

## 3. 电源

```
电池 7.4V ──┬── TB6612 VM
            ├── MP1584(5V) ──┬── 灰度 VCC / TB6612 VCC / ESP32(经 3.3V)
            └── AMS1117-3.3 ── STM32 3.3V
```

## 4. 平台

- 亚克力板 60×60cm 固定于小车底盘上
- 中心图案：直径 50cm 外圈 + 30cm 内圈 + 十字，线宽 2cm（黑色）
- 重心控制：电池靠中心低放，抛投物收纳盒置于平台侧边
