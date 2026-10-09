# -*- coding: utf-8 -*-
"""无人机任务状态机（核心逻辑，不依赖硬件）

状态：IDLE → TAKEOFF → HOVER → FORMATION → DELIVERY → RETURN → LAND
                          ↘ DYN_ALIGN → DYN_DESCEND → DYN_HOLD → RECHOVER → RETURN
"""
import time

import config as C
from comms import Comms
from mavlink_ctl import MavlinkCtl
from vision import DownVision
from delivery import Delivery

# 状态名（与通信协议一致）
ST_IDLE = "IDLE"
ST_TAKEOFF = "TAKEOFF"
ST_HOVER = "HOVER"
ST_FORMATION = "FORMATION"
ST_DELIVERY = "DELIVERY"
ST_RETURN = "RETURN"
ST_LAND = "LAND"
ST_DYN_ALIGN = "DYN_ALIGN"
ST_DYN_DESCEND = "DYN_DESCEND"
ST_DYN_HOLD = "DYN_HOLD"
ST_RECHOVER = "RECHOVER"
ST_DONE = "DONE"
ST_FAIL = "FAIL"


class StateMachine:
    def __init__(self, comms, mav, vision, delivery):
        self.c = comms
        self.mav = mav
        self.vis = vision
        self.deliv = delivery

        self.state = ST_IDLE
        self.t0 = 0.0
        self.dropped = False
        self.hold_start = 0.0
        self.fail_reason = ""

    # ---------------- 辅助 ----------------
    def _enter(self, st):
        self.state = st
        self.t0 = time.time()
        print(f"[fsm] -> {st}")

    def _elapsed(self):
        return time.time() - self.t0

    def _home_xy_ned(self):
        return 0.0, 0.0   # 本地 NED 原点即起降点 H

    # ---------------- 主更新 ----------------
    def update(self, dt):
        """返回 (tx, ty, tz, yaw, action)；action ∈ {none, drop, land, arm, disarm}"""
        pos = self.mav.get_state()
        car, car_ts = self.c.get_car()
        mode = self.c.get_mode()
        estop = self.c.is_estop()

        # 急停：保持悬停
        if estop and self.state not in (ST_IDLE, ST_DONE):
            return pos["x"], pos["y"], pos["z"], 0.0, "none"

        # 小车数据过期保护
        car_ok = (car is not None) and (time.time() - car_ts < 1.0)

        if self.state == ST_IDLE:
            if mode is not None:
                self._enter(ST_TAKEOFF)
                self.mav.arm()
                self.mav.set_mode("GUIDED")
                self.mav.takeoff(C.CRUISE_ALT)
            return pos["x"], pos["y"], pos["z"], 0.0, "none"

        if self.state == ST_TAKEOFF:
            hx, hy = self._home_xy_ned()
            if pos["z"] <= -(C.CRUISE_ALT - 0.1):   # 到达 150cm
                self._enter(ST_HOVER)
            # 起飞阶段同时下发高度设定点（与 NAV_TAKEOFF 双保险，仿真亦可用）
            return hx, hy, -C.CRUISE_ALT, 0.0, "none"

        if self.state == ST_HOVER:
            hx, hy = self._home_xy_ned()
            if self._elapsed() >= C.HOVER_TIME:
                self._enter(ST_FORMATION if car_ok else ST_FAIL)
                if self.state == ST_FAIL:
                    self.fail_reason = "no car telemetry"
            return hx, hy, -C.CRUISE_ALT, 0.0, "none"

        if self.state == ST_FORMATION:
            if not car_ok:
                # 丢车：悬停等待
                return pos["x"], pos["y"], pos["z"], 0.0, "none"
            cx, cy = C.ned_from_field(car["x"], car["y"])
            yaw = 90.0 - float(car.get("psi", 90.0))   # 场地角 → NED 航向
            dx = cx - pos["x"]
            dy = cy - pos["y"]
            dist = (dx * dx + dy * dy) ** 0.5
            # 小车速度 cm/s → m/s
            vx = float(car.get("v", 0.0)) / 100.0
            vy = 0.0

            # 抛投任务：对准且相对速度低 → 抛投
            if mode == "DELIVERY" and dist < C.DROP_XY_TOL and (vx ** 2 + vy ** 2) ** 0.5 < C.DROP_REL_SPEED_MAX:
                self._enter(ST_DELIVERY)
                return cx, cy, -C.CRUISE_ALT, yaw, "drop"
            # 动态起降：接近小车 → 转入视觉对准下降
            if mode == "DYNAMIC_LAND" and dist < 0.30:
                self._enter(ST_DYN_ALIGN)
            return cx, cy, -C.CRUISE_ALT, yaw, "none"

        if self.state == ST_DELIVERY:
            cx, cy = C.ned_from_field(car["x"], car["y"]) if car_ok else (pos["x"], pos["y"])
            yaw = 90.0 - float(car.get("psi", 90.0)) if car_ok else 0.0
            if self._elapsed() > C.DELIVERY_HOLD:
                self._enter(ST_RETURN)
            return cx, cy, -C.CRUISE_ALT, yaw, "none"

        if self.state == ST_DYN_ALIGN:
            if not car_ok:
                return pos["x"], pos["y"], pos["z"], 0.0, "none"
            cx, cy = C.ned_from_field(car["x"], car["y"])
            yaw = 90.0 - float(car.get("psi", 90.0))
            dx_cm, dy_cm, found = self.vis.detect()
            # 对准：把平台圆心拉到图像中心
            tx = cx - dx_cm / 100.0
            ty = cy - dy_cm / 100.0
            if found and (abs(dx_cm) < C.DYN_LAND_XY_TOL * 100.0 and abs(dy_cm) < C.DYN_LAND_XY_TOL * 100.0):
                self._enter(ST_DYN_DESCEND)
            return tx, ty, -C.DYN_ALIGN_ALT, yaw, "none"

        if self.state == ST_DYN_DESCEND:
            dx_cm, dy_cm, found = self.vis.detect()
            cx, cy = C.ned_from_field(car["x"], car["y"]) if car_ok else (pos["x"], pos["y"])
            tx = cx - dx_cm / 100.0
            ty = cy - dy_cm / 100.0
            tz = pos["z"] + C.DYN_DESCENT_SPEED * dt   # 持续下降 0.3m/s
            if tz > -C.TOUCHDOWN_Z:
                tz = -C.TOUCHDOWN_Z
            if pos["z"] >= -C.TOUCHDOWN_Z:
                self._enter(ST_DYN_HOLD)
            yaw = 90.0 - float(car.get("psi", 90.0)) if car_ok else 0.0
            return tx, ty, tz, yaw, "none"

        if self.state == ST_DYN_HOLD:
            # 落地保持 5s（油门最低，桨保持转动提供稳定）
            if self._elapsed() >= C.PLATFORM_STAY:
                self._enter(ST_RECHOVER)
                self.mav.takeoff(C.CRUISE_ALT)
            return pos["x"], pos["y"], -C.TOUCHDOWN_Z, 0.0, "none"

        if self.state == ST_RECHOVER:
            hx, hy = self._home_xy_ned()
            if pos["z"] <= -(C.CRUISE_ALT - 0.1):
                self._enter(ST_RETURN)
            return hx, hy, -C.CRUISE_ALT, 0.0, "none"

        if self.state == ST_RETURN:
            hx, hy = self._home_xy_ned()
            dx = hx - pos["x"]
            dy = hy - pos["y"]
            if (dx * dx + dy * dy) ** 0.5 < 0.30:
                self._enter(ST_LAND)
                self.mav.land()
            return hx, hy, -C.CRUISE_ALT, 0.0, "none"

        if self.state == ST_LAND:
            if pos["z"] >= -C.TOUCHDOWN_Z:
                self._enter(ST_DONE)
                self.mav.disarm()
            return pos["x"], pos["y"], 0.0, 0.0, "none"

        # ST_DONE / ST_FAIL / 其他
        return pos["x"], pos["y"], pos["z"], 0.0, "none"

    def state_text(self):
        return self.state
