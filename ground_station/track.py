# -*- coding: utf-8 -*-
"""轨迹几何（与小车固件 odometry.c 保持一致）

环线 = 左右直道 150cm + 上下半圆 R=75cm（跑道形）：
  左直道 x=125，右直道 x=275（间距 150=2R），直道 y: 175→325
  上半圆：中心(200,325)，过 B(200,400)
  下半圆：中心(200,175)，过 A(200,100)
A 位于下半个圆底部（起点/终点），B 位于上半个圆顶部，C 右上接点，D 右下接点。
场地坐标：原点左下角，x 右 y 上，单位 cm。顺时针行驶。
"""
import math

TRACK_R = 75.0
TRACK_STRAIGHT = 150.0
TRACK_L = 2 * TRACK_STRAIGHT + 2 * math.pi * TRACK_R   # ≈771.2

SEG_A0 = 117.8   # A→左接点（下半圆左半程）
SEG_A1 = 267.8   # →左接点+左直道（= 上半圆起点）
SEG_B = 385.6    # A→B（上半圆中点）
SEG_C = 503.4    # A→C（上半圆终点=右上接点）
SEG_D = 653.4    # A→D（右直道终点=右下接点）
# 全圈 771.2

HOME_X, HOME_Y = 126.0, 105.0   # 起降点 H（场地左下区域，环外）


def track_position(s):
    """里程 s（cm）→ (x, y, psi_deg)，psi 为场地航向（0=+x, 90=+y）"""
    ss = s % TRACK_L
    if ss < SEG_A0:
        # 下半圆左半程：中心(200,175)，θ: 270°→180°
        th = 1.5 * math.pi - ss / TRACK_R
        x = 200 + TRACK_R * math.cos(th)
        y = 175 + TRACK_R * math.sin(th)
        dx, dy = math.sin(th), -math.cos(th)
    elif ss < SEG_A1:
        # 左直道向上：y 175→325
        x = 125.0
        y = 175.0 + (ss - SEG_A0)
        dx, dy = 0.0, 1.0
    elif ss < SEG_C:
        # 上半圆：中心(200,325)，θ: 180°→0°，过 B(200,400)
        th = math.pi - (ss - SEG_A1) / TRACK_R
        x = 200 + TRACK_R * math.cos(th)
        y = 325 + TRACK_R * math.sin(th)
        dx, dy = math.sin(th), -math.cos(th)
    elif ss < SEG_D:
        # 右直道向下：y 325→175
        x = 275.0
        y = 325.0 - (ss - SEG_C)
        dx, dy = 0.0, -1.0
    else:
        # 下半圆右半程：中心(200,175)，θ: 0°→270°，回到 A(200,100)
        th = -(ss - SEG_D) / TRACK_R
        x = 200 + TRACK_R * math.cos(th)
        y = 175 + TRACK_R * math.sin(th)
        dx, dy = math.sin(th), -math.cos(th)
    return x, y, math.degrees(math.atan2(dy, dx))
