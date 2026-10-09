/**
 * @file    line_sensor.c
 * @brief   8 路灰度传感器：PB8~PB15 输入
 */
#include "main.h"
#include "car_config.h"
#include "line_sensor.h"

/* 传感器极性：TCRT5000 数字模块检测到黑线通常输出低电平，
 * 上拉输入读到 0 = 黑线。若模块为高电平表示黑线，反转此宏。 */
#define BLACK_LEVEL       0u

static uint8_t gray_pins[GRAY_COUNT];
static uint8_t gray_active;  /* 当前有效路数（黑线覆盖数） */

void line_sensor_init(void)
{
    /* 引脚已在 MX_GPIO_Init 配置为上拉输入；这里登记引脚表 */
    for (uint8_t i = 0; i < GRAY_COUNT; i++) {
        gray_pins[i] = (uint8_t)(GRAY_PIN_BASE << i); /* PB8<<i = PB8..PB15 */
    }
    gray_active = 0;
}

uint8_t line_sensor_read_raw(void)
{
    uint8_t raw = 0;
    for (uint8_t i = 0; i < GRAY_COUNT; i++) {
        GPIO_PinState s = HAL_GPIO_ReadPin(GRAY_PORT, gray_pins[i]);
        uint8_t black = (s == BLACK_LEVEL) ? 1u : 0u;
        raw |= (uint8_t)(black << i);
    }
    return raw;
}

float line_sensor_error(void)
{
    uint8_t raw = line_sensor_read_raw();
    float sum = 0.0f;
    uint8_t cnt = 0;

    for (uint8_t i = 0; i < GRAY_COUNT; i++) {
        if (raw & (1u << i)) {
            sum += (float)i;   /* 传感器序号 0..7 */
            cnt++;
        }
    }
    gray_active = cnt;

    if (cnt == 0) {
        /* 丢线：返回上一周期符号保持，由上层策略处理（维持上次方向） */
        return 0.0f;
    }
    /* 中心在 3.5（4、5 路之间） */
    return sum / (float)cnt - 3.5f;
}

uint8_t line_sensor_on_track(void)
{
    return (gray_active > 0) ? 1u : 0u;
}
