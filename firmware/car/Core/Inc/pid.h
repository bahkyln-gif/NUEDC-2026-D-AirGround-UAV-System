/**
 * @file    pid.h
 * @brief   位置式 PID 控制器
 */
#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    float kp, ki, kd;
    float integral;
    float i_limit;   /* 积分限幅 */
    float out_limit; /* 输出限幅 */
    float prev_err;
    uint8_t first;
} pid_t;

void pid_init(pid_t *p, float kp, float ki, float kd, float i_lim, float o_lim);
void pid_reset(pid_t *p);
/* 位置式 PID：输入误差，输出控制量 */
float pid_update(pid_t *p, float err, float dt);

#endif /* PID_H */
