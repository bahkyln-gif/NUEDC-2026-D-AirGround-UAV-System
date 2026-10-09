# -*- coding: utf-8 -*-
"""地面站全局参数"""

# ==================== 网络 ====================
BIND_IP = "0.0.0.0"
GS_PORT = 14551
UAV_IP = "192.168.1.101"     # 无人机伴机 IP（发 GS_CMD 用）
CAR_IP = "192.168.1.102"     # 小车 ESP32 IP
BROADCAST = "255.255.255.255"

# ==================== 场地显示 ====================
FIELD_W, FIELD_H = 400.0, 500.0
CANVAS_W, CANVAS_H = 800, 1000
SCALE = min(CANVAS_W / FIELD_W, CANVAS_H / FIELD_H)   # cm → px
MARGIN_X = (CANVAS_W - FIELD_W * SCALE) / 2.0
MARGIN_Y = 10

# ==================== 颜色 ====================
COL_BG = "#fdfdfd"
COL_LINE = "#000000"
COL_CAR = "#1f6fb2"
COL_UAV = "#2e7d32"
COL_UAV_DANGER = "#c62828"
COL_STATUS = "#b26a00"
COL_GRID = "#d0d0d0"

# ==================== 仿真 ====================
SIM_HZ = 50
