/**
 * @file    pid.c
 * @brief   位置式 PID
 */
#include "pid.h"

void pid_init(pid_t *p, float kp, float ki, float kd, float i_lim, float o_lim)
{
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->i_limit = i_lim; p->out_limit = o_lim;
    pid_reset(p);
}

void pid_reset(pid_t *p)
{
    p->integral = 0.0f;
    p->prev_err = 0.0f;
    p->first = 1;
}

float pid_update(pid_t *p, float err, float dt)
{
    float out;

    if (dt <= 0.0f) dt = 0.005f;

    /* 积分项 + 限幅 */
    p->integral += err * dt;
    if (p->integral >  p->i_limit) p->integral =  p->i_limit;
    if (p->integral < -p->i_limit) p->integral = -p->i_limit;

    /* 微分项（首拍不微分，抑制突变） */
    float d = 0.0f;
    if (!p->first) {
        d = (err - p->prev_err) / dt;
    }
    p->prev_err = err;
    p->first = 0;

    out = p->kp * err + p->ki * p->integral + p->kd * d;

    if (out >  p->out_limit) out =  p->out_limit;
    if (out < -p->out_limit) out = -p->out_limit;
    return out;
}
