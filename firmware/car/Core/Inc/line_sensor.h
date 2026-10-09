/**
 * @file    line_sensor.h
 * @brief   8 路灰度传感器：读取与偏差计算
 */
#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdint.h>

void line_sensor_init(void);

/* 读取 8 路原始值（1=检测到黑线，极性可在此模块内反转） */
uint8_t line_sensor_read_raw(void);

/* 加权中心偏差：范围 [-3.5, +3.5]，黑线居中时≈0 */
float line_sensor_error(void);

/* 是否检测到黑线 */
uint8_t line_sensor_on_track(void);

#endif /* LINE_SENSOR_H */
