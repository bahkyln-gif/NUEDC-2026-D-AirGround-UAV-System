/**
 * @file    motor.h
 * @brief   电机 PWM 控制 + 编码器读取
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

void motor_init(void);

/* 以 cm/s 设定左右轮速度（负值后退） */
void motor_set_speed(float left_cmps, float right_cmps);

/* 读取编码器在本周期内增量（脉冲数）并清零：左/右 */
void motor_encoder_delta(int32_t *dl, int32_t *dr);

/* 立即停车 */
void motor_stop(void);

#endif /* MOTOR_H */
