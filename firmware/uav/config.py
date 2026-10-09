# -*- coding: utf-8 -*-
"""全局参数：网络、飞控、任务"""
import os

# ==================== 运行模式 ====================
SIMULATE = os.environ.get("UAV_SIM", "0") == "1"

# ==================== 网络 ====================
GS_IP = "192.168.1.100"        # 地面站 IP（同一 WiFi 局域网）
GS_PORT = 14551
UAV_PORT = 14552               # 本机接收端口
BROADCAST = "255.255.255.255"

# ==================== 飞控（MAVLink） ====================
# ArduPilot SITL: "tcp:127.0.0.1:5760"；真实飞控经伴机串口或 UDP
MAVLINK_CONN = os.environ.get("MAVLINK_CONN", "udpin:0.0.0.0:14550")
MAV_SYSID = 1                  # 飞控系统 ID
GCS_SYSID = 255

# ==================== 场地（cm） ====================
FIELD_W, FIELD_H = 400.0, 500.0
HOME_X, HOME_Y = 126.0, 105.0  # 起降点 H 的场地坐标（左下区域，环外）

def ned_from_field(fx, fy):
    """场地坐标(cm) → 本地 NED(m，原点 H)：x 东, y 北, z 下"""
    return (fx - HOME_X) / 100.0, (fy - HOME_Y) / 100.0

def field_from_ned(nx, ny):
    """本地 NED(m) → 场地坐标(cm)"""
    return nx * 100.0 + HOME_X, ny * 100.0 + HOME_Y

# ==================== 任务参数 ====================
CRUISE_ALT = 1.5               # 巡航高度 m（150cm）
HOVER_TIME = 3.0               # 悬停秒数
FORMATION_OFFSET = 0.0         # 伴飞水平偏移 m（默认正上方）
FORM_K_XY = 0.8                # 伴飞位置增益
FORM_K_V = 0.6                 # 伴飞速度前馈增益
DROP_REL_SPEED_MAX = 0.05      # 抛投相对速度阈值 m/s
DROP_XY_TOL = 0.20             # 抛投水平对准阈值 m
DELIVERY_HOLD = 1.0            # 抛投后保持时间 s
DYN_ALIGN_ALT = 0.60           # 动态降落视觉对准高度 m
DYN_DESCENT_SPEED = 0.30       # 降落下降速度 m/s
DYN_LAND_XY_TOL = 0.05         # 降落对准阈值 m
PLATFORM_STAY = 5.0            # 平台停留时间 s
TOUCHDOWN_Z = 0.06             # 判定触地高度 m

# ==================== 控制环 ====================
CTRL_HZ = 20
TELE_HZ = 10
