# 小车固件（STM32F103C8T6 + HAL）

循线小车完整固件：8 路灰度循线 PID + 编码器里程计（轨迹地图定位）+ 一键启动 + ESP32 无线通信。

## 目录

```
Core/
├── Inc/
│   ├── car_config.h    全局参数（轨迹几何/速度档位/引脚）
│   ├── pid.h           PID 控制器
│   ├── line_sensor.h   灰度传感器阵列
│   ├── motor.h         电机 PWM + 编码器
│   ├── odometry.h      里程计 + 轨迹地图定位
│   └── comms.h         UART 通信（ESP32 透传）
└── Src/                对应实现 + main.c
```

## 工程搭建（STM32CubeMX）

1. 新建 STM32F103C8T6 工程，时钟 72MHz（HSE 8M ×9）
2. 外设配置：
   - TIM2：CH1(PA0) 左电机 PWM、CH4(PA3) 右电机 PWM，频率 20kHz
   - TIM3：Encoder Mode（CH1=PA6, CH2=PA7, CH3=PB0, CH4=PB1），4 倍频
   - USART1：115200-8-N-1（PA9/PA10 → ESP32）
   - GPIO：PA8 按键（下拉+EXTI）、PA11 蜂鸣器、PB8~PB15 灰度输入（上拉）、PC13 LED
3. 把本目录 `Core/Inc`、`Core/Src` 覆盖进工程（保留 CubeMX 生成的 `stm32f1xx_hal_msp.c` 等），编译烧录。

## 与 ESP32 的串口协议

小车侧为简单 ASCII 协议（115200 波特率），ESP32 负责 WiFi UDP 转发：

| 方向 | 帧 | 说明 |
|---|---|---|
| 车→ESP32 | `S,<MODE>\n` | 一键启动，MODE=DELIVERY/DYNAMIC_LAND |
| 车→ESP32 | `C,<mode>,<x>,<y>,<s>,<v>,<state>\n` | 遥测 10Hz |
| 车→ESP32 | `F,<lap_time>\n` | 任务完成 |
| ESP32→车 | `G,ESTOP\n` | 地面站急停 |
| ESP32→车 | `G,RESUME\n` | 恢复 |

ESP32 侧映射：

- `S,<MODE>` → UDP 广播 `UAV_START`（type=UAV_START, mode=...）到无人机端口 14552
- `C,...` → UDP JSON `CAR_TELEMETRY` 到地面站 14551
- 收到地面站 `GS_CMD` → 转 `G,ESTOP` 等
- ESP32 固件见仓库根 `tools/esp32_bridge/`（可用 AT 固件 + 脚本，或 Arduino 透传固件）

## 关键参数（可调）

| 参数 | 默认 | 说明 |
|---|---|---|
| 循线 PID | Kp=0.60 Ki=0.02 Kd=0.80 | 位置式，差速修正 |
| 控制频率 | 200Hz | 5ms 周期 |
| 里程计 | 编码器 13 线 ×4 倍频，轮径 6.5cm | 需现场标定 |
| 抛投任务速度 | 12/15/18 cm/s | 按 A→B / B→D / D→A 分段 |
| 动态起降速度 | 26/16/18 cm/s | 首段冲 B 点 |
| 软启动 | 限幅 20 cm/s² | 防打滑 |
