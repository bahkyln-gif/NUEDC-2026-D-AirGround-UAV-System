# -*- coding: utf-8 -*-
"""仿真器：模拟小车与无人机，按通信协议向地面站发送消息（无需硬件演示）"""
import math
import threading
import time

import config as C
import track


class Simulator:
    def __init__(self, network, mode="DELIVERY"):
        self.net = network
        self.mode = mode
        self.running = False
        self._thread = None

    def start(self):
        self.running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self.running = False

    # ---------- 小车仿真 ----------
    def _car_profile(self, s):
        if self.mode == "DELIVERY":
            return 12.0 if s < track.SEG_B else (15.0 if s < track.SEG_D else 18.0)
        return 26.0 if s < track.SEG_B else (16.0 if s < track.SEG_D else 18.0)

    def _run(self):
        t0 = time.time()
        s = 0.0
        v = 0.0
        uav_state = "IDLE"
        uav_s = 0.0          # 无人机进度（简化：滞后小车一段）
        uav_alt = 0.0
        delivered = False
        landed = False
        dyn_phase = "fly"    # fly / land / hold / return

        while self.running:
            dt = 1.0 / C.SIM_HZ
            now = time.time()

            # ---- 小车 ----
            if s < track.TRACK_L:
                target = self._car_profile(s)
                v += (target - v) * min(dt * 2.0, 1.0)
                s += v * dt
            else:
                v = 0.0
            x, y, psi = track.track_position(s)
            state = "RUNNING" if s < track.TRACK_L else "DONE"

            car_msg = {
                "type": "CAR_TELEMETRY", "ts": now, "from": "car",
                "data": {"x": round(x, 1), "y": round(y, 1), "s": round(s, 1),
                         "v": round(v, 2), "psi": round(psi, 1), "state": state},
            }
            self.net.sock.sendto(str(car_msg).replace("'", '"').encode("utf-8"),
                                 ("127.0.0.1", C.GS_PORT))

            # ---- 无人机（简化时序脚本） ----
            elapsed = now - t0
            if self.mode == "DELIVERY":
                if elapsed < 3:
                    uav_state, uav_alt = "TAKEOFF", min(1.5, elapsed / 3 * 1.5)
                elif elapsed < 6:
                    uav_state, uav_alt = "HOVER", 1.5
                elif s < track.SEG_D:
                    uav_state, uav_alt = "FORMATION", 1.5
                elif not delivered:
                    uav_state, uav_alt = "DELIVERY", 1.5
                    delivered = True
                elif not landed:
                    uav_state, uav_alt = "RETURN", 1.5
                    if elapsed > 30:
                        uav_state, uav_alt = "LAND", 0.0
                        landed = True
                else:
                    uav_state, uav_alt = "DONE", 0.0
            else:
                if elapsed < 3:
                    uav_state, uav_alt = "TAKEOFF", min(1.5, elapsed / 3 * 1.5)
                elif dyn_phase == "fly":
                    uav_state, uav_alt = "DYN_ALIGN", 0.6 if s > 200 else 1.5
                    if s > 260:
                        dyn_phase = "land"
                elif dyn_phase == "land":
                    uav_state, uav_alt = "DYN_DESCEND", max(0.0, uav_alt - 0.3 * dt)
                    if uav_alt <= 0.0:
                        dyn_phase = "hold"
                elif dyn_phase == "hold":
                    uav_state, uav_alt = "DYN_HOLD", 0.0
                    if elapsed > 12:
                        dyn_phase = "return"
                else:
                    uav_state, uav_alt = "RETURN" if not landed else "DONE", 1.5
                    if elapsed > 20 and not landed:
                        uav_state, uav_alt = "LAND", 0.0
                        landed = True

            # 无人机位置：小车位置 + 小偏移
            ux, uy = x, y
            uav_msg = {
                "type": "UAV_TELEMETRY", "ts": now, "from": "uav",
                "data": {"x": round(ux, 1), "y": round(uy, 1), "alt": round(uav_alt, 2),
                         "vx": round(v, 2), "vy": 0.0, "psi": round(psi, 1),
                         "state": uav_state, "batt": 11.6},
            }
            self.net.sock.sendto(str(uav_msg).replace("'", '"').encode("utf-8"),
                                 ("127.0.0.1", C.GS_PORT))

            # 完成事件
            if s >= track.TRACK_L and state == "DONE" and not self._done_sent:
                done = {"type": "CAR_DONE", "ts": now, "from": "car",
                        "data": {"lap_time": round(now - t0, 1)}}
                self.net.sock.sendto(str(done).replace("'", '"').encode("utf-8"),
                                     ("127.0.0.1", C.GS_PORT))
                self._done_sent = True

            time.sleep(dt)

    _done_sent = False
