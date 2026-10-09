/**
 * @file    odometry.c
 * @brief   轨迹地图定位实现
 *
 * 轨迹参数化（场地坐标，原点左下角，x 右 y 上，单位 cm）：
 *   环线 = 左右直道 150cm + 上下半圆 R=75cm（跑道形）
 *   左直道 x=125，右直道 x=275（间距 150=2R），直道 y: 175→325
 *   上半圆中心(200,325)，过 B(200,400)；下半圆中心(200,175)，过 A(200,100)
 *   A 位于下半个圆底部（起点/终点），顺时针：
 *   段0: 下半圆左半程 A→左接点，弧长 117.8
 *   段1: 左直道向上       117.8→267.8，长 150
 *   段2: 上半圆(过 B)    267.8→503.4，弧长 235.6
 *   段3: 右直道向下       503.4→653.4，长 150
 *   段4: 下半圆右半程     653.4→771.2，弧长 117.8
 */
#include "car_config.h"
#include "odometry.h"
#include <math.h>

#define PI_F 3.14159265f
/* 分段里程 */
#define S0_END  117.8f
#define S1_END  267.8f
#define S2_END  503.4f   /* = SEG_C */
#define S3_END  653.4f   /* = SEG_D */

void odom_reset(odom_t *o)
{
    o->s = 0.0f;
    o->x = 200.0f; o->y = 100.0f; o->psi = 180.0f;
    o->v = 0.0f;
}

void track_position(float s, float *x, float *y, float *psi)
{
    float ss = s;
    while (ss >= TRACK_L) ss -= TRACK_L;
    while (ss < 0.0f)     ss += TRACK_L;

    float arc, th, dx, dy;

    if (ss < S0_END) {
        /* 段0：下半圆左半程，中心(200,175)，θ: 270°→180° */
        arc = ss;
        th = 1.5f * PI_F - arc / TRACK_R;
        *x = 200.0f + TRACK_R * cosf(th);
        *y = 175.0f + TRACK_R * sinf(th);
        dx = sinf(th); dy = -cosf(th);
    } else if (ss < S1_END) {
        /* 段1：左直道向上，y 175→325 */
        arc = ss - S0_END;
        *x = 125.0f;
        *y = 175.0f + arc;
        dx = 0.0f; dy = 1.0f;
    } else if (ss < S2_END) {
        /* 段2：上半圆，中心(200,325)，θ: 180°→0°，过 B(200,400) */
        arc = ss - S1_END;
        th = PI_F - arc / TRACK_R;
        *x = 200.0f + TRACK_R * cosf(th);
        *y = 325.0f + TRACK_R * sinf(th);
        dx = sinf(th); dy = -cosf(th);
    } else if (ss < S3_END) {
        /* 段3：右直道向下，y 325→175 */
        arc = ss - S2_END;
        *x = 275.0f;
        *y = 325.0f - arc;
        dx = 0.0f; dy = -1.0f;
    } else {
        /* 段4：下半圆右半程，中心(200,175)，θ: 0°→-90°，回到 A(200,100) */
        arc = ss - S3_END;
        th = -arc / TRACK_R;
        *x = 200.0f + TRACK_R * cosf(th);
        *y = 175.0f + TRACK_R * sinf(th);
        dx = sinf(th); dy = -cosf(th);
    }

    *psi = atan2f(dy, dx) * 180.0f / PI_F;
}

void odom_update(odom_t *o, float ds, float dt)
{
    o->s += ds;
    if (o->s >= TRACK_L) o->s -= TRACK_L;   /* 一圈归位，保持 0~L 表示相对 A 的进度 */

    float x, y, psi;
    track_position(o->s, &x, &y, &psi);
    o->x = x; o->y = y; o->psi = psi;

    if (dt > 0.0f) o->v = ds / dt;
}
