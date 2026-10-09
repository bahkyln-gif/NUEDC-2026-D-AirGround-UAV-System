/**
 * @file    odometry.h
 * @brief   里程计 + 轨迹地图定位
 *
 * 小车严格沿已知轨迹行驶，只需累计里程 s(t)，再由轨迹参数方程
 * (x, y, psi) = Track(s mod L) 反查全场位置，抗漂移能力强。
 */
#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>

typedef struct {
    float s;      /* 累计里程 cm（0 ~ TRACK_L，起点 A） */
    float x, y;   /* 场地坐标 cm（原点左下角，x 右 y 上） */
    float psi;    /* 航向 deg：0=+x，90=+y */
    float v;      /* 速度 cm/s */
} odom_t;

void odom_reset(odom_t *o);

/* 按左右轮里程增量更新（cm），dt 秒 */
void odom_update(odom_t *o, float ds, float dt);

/* 由里程反查轨迹位置（供 odom_update 内部与测试使用） */
void track_position(float s, float *x, float *y, float *psi);

#endif /* ODOMETRY_H */
