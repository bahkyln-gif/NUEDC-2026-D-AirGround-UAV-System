# -*- coding: utf-8 -*-
"""MAVLink 控制封装（pymavlink → ArduPilot 飞控）

对外提供：连接/解锁/起飞/悬停/位置控制/降落/状态读取。
支持 --sim 仿真模式（无飞控时用运动学模型代替）。
"""
import math
import threading
import time

import config as C


class MavlinkCtl:
    def __init__(self, simulate=False):
        self.sim = simulate
        self.armed = False
        self.mode = "STABILIZE"
        self._pos = {"x": 0.0, "y": 0.0, "z": 0.0}   # 本地 NED，原点=起降点
        self._vel = {"vx": 0.0, "vy": 0.0, "vz": 0.0}
        self._att = {"yaw": 0.0}
        self._batt = None
        self._last_hb = 0.0
        self._mav = None
        self._setpoint = None
        self._stop = False

        if not self.sim:
            from pymavlink import mavutil
            self._mav = mavutil.mavlink_connection(C.MAVLINK_CONN, source_system=C.GCS_SYSID)
            self._mav.wait_heartbeat(timeout=10)
            print("[mav] connected, waiting heartbeat ok")

    # ---------- 内部线程 ----------
    def start(self):
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        if self.sim:
            self._sim_loop()
            return
        while not self._stop:
            try:
                m = self._mav.recv_match(blocking=True, timeout=0.1)
            except Exception:
                time.sleep(0.05)
                continue
            if m is None:
                continue
            t = m.get_type()
            if t == "LOCAL_POSITION_NED":
                self._pos = {"x": m.x, "y": m.y, "z": m.z}
                self._vel = {"vx": m.vx, "vy": m.vy, "vz": m.vz}
            elif t == "ATTITUDE":
                self._att["yaw"] = math.degrees(m.yaw)
            elif t == "HEARTBEAT":
                self._last_hb = time.time()
            elif t == "BATTERY_STATUS":
                if m.voltages and m.voltages[0] != 65535:
                    self._batt = m.voltages[0] / 1000.0

    def _sim_loop(self):
        """仿真运动学：一阶跟踪设定点（便于无硬件联调逻辑）"""
        dt = 0.05
        while not self._stop:
            sp = self._setpoint
            if self.armed and sp is not None:
                k = 1.5
                for key in ("x", "y", "z"):
                    self._pos[key] += (sp[key] - self._pos[key]) * k * dt
                self._vel["vx"] = (sp["x"] - self._pos["x"]) * k
                self._vel["vy"] = (sp["y"] - self._pos["y"]) * k
                self._vel["vz"] = (sp["z"] - self._pos["z"]) * k
            time.sleep(dt)

    # ---------- 状态 ----------
    def get_state(self):
        return {
            "x": self._pos["x"], "y": self._pos["y"], "z": self._pos["z"],
            "vx": self._vel["vx"], "vy": self._vel["vy"], "vz": self._vel["vz"],
            "yaw": self._att["yaw"],
            "armed": self.armed,
            "batt": self._batt,
        }

    # ---------- 控制 ----------
    def arm(self):
        self.armed = True
        if not self.sim:
            self._mav.mav.command_long_send(
                self._mav.target_system, self._mav.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        print("[mav] arm")

    def disarm(self):
        self.armed = False
        if not self.sim:
            self._mav.mav.command_long_send(
                self._mav.target_system, self._mav.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
        print("[mav] disarm")

    def set_mode(self, mode_name):
        if self.sim:
            self.mode = mode_name
            return
        mode_id = self._mav.mode_mapping().get(mode_name)
        if mode_id is None:
            return
        self._mav.set_mode_apm(mode_id)
        self.mode = mode_name

    def set_position_ned(self, x, y, z, yaw=None):
        """位置设定点（本地 NED），OFFBOARD/GUIDED 下由飞控内环跟踪"""
        sp = {"x": x, "y": y, "z": z}
        self._setpoint = sp
        if self.sim:
            return
        yaw_rad = math.radians(yaw) if yaw is not None else None
        self._mav.mav.set_position_target_local_ned_send(
            0, self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b110111111000,          # 仅使能 x/y/z/yaw
            x, y, z, 0, 0, 0, 0, 0, 0,
            yaw_rad if yaw_rad is not None else 0.0, 0.0)

    def set_velocity_ned(self, vx, vy, vz, yaw=None):
        """速度设定点（配合位置外环）"""
        if self.sim:
            self._setpoint["x"] += vx * 0.1
            self._setpoint["y"] += vy * 0.1
            self._setpoint["z"] += vz * 0.1
            return
        yaw_rad = math.radians(yaw) if yaw is not None else 0.0
        self._mav.mav.set_position_target_local_ned_send(
            0, self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b110111000111,
            0, 0, 0, vx, vy, vz, 0, 0, 0, yaw_rad, 0.0)

    def land(self):
        if self.sim:
            self._setpoint = {"x": self._pos["x"], "y": self._pos["y"], "z": 0.0}
            return
        self._mav.mav.command_long_send(
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)

    def takeoff(self, alt):
        if self.sim:
            self.armed = True
            return
        self._mav.mav.command_long_send(
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)

    def stop(self):
        self._stop = True
