/**
 * @file    car_config.h
 * @brief   小车全局参数：轨迹几何、速度档位、引脚、控制参数
 */
#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

/* ===================== 场地轨迹几何（cm） ===================== */
/* 环线：上下 R=75cm 半圆 + 左右 150cm 直道，A 点位于下半个圆底部，顺时针行驶 */
#define TRACK_R          75.0f   /* 半圆半径 */
#define TRACK_STRAIGHT   150.0f  /* 直道长度 */
#define TRACK_L          (2.0f * TRACK_STRAIGHT + 2.0f * 3.14159265f * TRACK_R) /* 771.2cm */

/* 顺时针累计里程关键点（A 为 0） */
#define SEG_B            385.6f  /* A→B */
#define SEG_C            503.4f  /* A→C */
#define SEG_D            653.4f  /* A→D */

/* ===================== 车轮与编码器 ===================== */
#define WHEEL_D_CM       6.5f    /* 驱动轮直径 cm */
#define WHEEL_CIRC      (3.14159265f * WHEEL_D_CM) /* 20.42cm/圈 */
#define ENCODER_PPR      52u     /* 13 线 × 4 倍频 */
#define PULSE_PER_CM    (ENCODER_PPR / WHEEL_CIRC) /* ≈2.55 脉冲/cm */

/* ===================== 控制参数 ===================== */
#define CONTROL_HZ       200u    /* 控制频率 */
#define CTRL_PERIOD_MS   (1000u / CONTROL_HZ)
#define PID_KP           0.60f
#define PID_KI           0.02f
#define PID_KD           0.80f
#define PID_ILIM         50.0f
#define PID_OUTLIM       60.0f  /* 差速上限 cm/s */

/* 速度 → PWM 系数（需现场标定：PWM 值 / 实测 cm/s） */
#define PWM_PER_CMPS     55.0f

/* 软启动加速度限制 cm/s² */
#define ACCEL_LIMIT      20.0f

/* ===================== 任务模式 ===================== */
#define MODE_DELIVERY    0u      /* 抛投任务 */
#define MODE_DYN_LAND    1u      /* 动态起降任务 */

/* ===================== 速度档位（cm/s） ===================== */
#define V_DEL_S1         12.0f   /* 抛投 A→B */
#define V_DEL_S2         15.0f   /* 抛投 B→D */
#define V_DEL_S3         18.0f   /* 抛投 D→A */
#define V_DYN_S1         26.0f   /* 动态起降 A→B（≤15s 到 B） */
#define V_DYN_S2         16.0f   /* 动态起降 B→D（配合降落） */
#define V_DYN_S3         18.0f   /* 动态起降 D→A */

/* ===================== 通信 ===================== */
#define UART_BAUD        115200u
#define TELE_PERIOD_MS   100u    /* 遥测周期 */
#define RX_BUF_SIZE      128u
#define TX_BUF_SIZE      256u

/* ===================== 引脚（与 car_pinout.md 一致） ===================== */
/* 左电机 PWM: TIM2_CH1 = PA0；右电机 PWM: TIM2_CH4 = PA3 */
#define L_PWM_PORT       GPIOA
#define L_PWM_PIN        GPIO_PIN_0
#define R_PWM_PORT       GPIOA
#define R_PWM_PIN        GPIO_PIN_3

/* 方向引脚 */
#define L_DIR1_PORT      GPIOA
#define L_DIR1_PIN       GPIO_PIN_1
#define L_DIR2_PORT      GPIOA
#define L_DIR2_PIN       GPIO_PIN_2
#define R_DIR1_PORT      GPIOA
#define R_DIR1_PIN       GPIO_PIN_4
#define R_DIR2_PORT      GPIOA
#define R_DIR2_PIN       GPIO_PIN_5

/* 编码器：TIM3 Encoder（左 PA6/PA7），TIM4 Encoder（右 PB6/PB7） */
#define ENC_PORT_A       GPIOA
#define ENC_L_A_PIN      GPIO_PIN_6
#define ENC_L_B_PIN      GPIO_PIN_7
#define ENC_PORT_B       GPIOB
#define ENC_R_A_PIN      GPIO_PIN_6
#define ENC_R_B_PIN      GPIO_PIN_7

/* 灰度 8 路：PB8~PB15 */
#define GRAY_PORT        GPIOB
#define GRAY_PIN_BASE    GPIO_PIN_8
#define GRAY_COUNT       8u

/* 按键 PA8（下拉输入，按下=高），蜂鸣器 PA11，LED PC13 */
#define BTN_PORT         GPIOA
#define BTN_PIN          GPIO_PIN_8
#define BUZ_PORT         GPIOA
#define BUZ_PIN          GPIO_PIN_11
#define LED_PORT         GPIOC
#define LED_PIN          GPIO_PIN_13

/* 按键长按切换模式阈值 ms */
#define BTN_LONG_MS      1000u

#endif /* CAR_CONFIG_H */
