# -*- coding: utf-8 -*-
"""UDP 通信：接收 UAV_START / CAR_TELEMETRY，上报 UAV_TELEMETRY"""
import json
import socket
import threading
import time

import config as C


class Comms:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("", C.UAV_PORT))
        self.sock.settimeout(0.05)

        self._lock = threading.Lock()
        self.mode = None            # DELIVERY / DYNAMIC_LAND
        self.car = None             # 最近小车位置 dict(x,y,s,v,psi)
        self.car_last_ts = 0.0
        self.estop = False
        self._stop = False

    def start(self):
        threading.Thread(target=self._rx_loop, daemon=True).start()

    def _rx_loop(self):
        while not self._stop:
            try:
                data, addr = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                msg = json.loads(data.decode("utf-8"))
            except Exception:
                continue
            self._handle(msg)

    def _handle(self, msg):
        t = msg.get("type")
        with self._lock:
            if t == "UAV_START":
                self.mode = msg.get("data", {}).get("mode", "DELIVERY")
                self.car_last_ts = time.time()
            elif t == "CAR_TELEMETRY":
                self.car = msg.get("data", {})
                self.car_last_ts = time.time()
            elif t == "GS_CMD":
                cmd = msg.get("data", {}).get("cmd")
                if cmd == "ESTOP":
                    self.estop = True
                elif cmd == "RESUME":
                    self.estop = False

    def get_car(self):
        with self._lock:
            return self.car, self.car_last_ts

    def get_mode(self):
        with self._lock:
            return self.mode

    def is_estop(self):
        with self._lock:
            return self.estop

    def send_telemetry(self, x, y, alt, vx, vy, psi, state, batt=None):
        msg = {
            "type": "UAV_TELEMETRY",
            "ts": time.time(),
            "from": "uav",
            "data": {
                "x": round(x, 1), "y": round(y, 1),
                "alt": round(alt, 2),
                "vx": round(vx, 2), "vy": round(vy, 2),
                "psi": round(psi, 1), "state": state,
            },
        }
        if batt is not None:
            msg["data"]["batt"] = round(batt, 2)
        try:
            self.sock.sendto(json.dumps(msg).encode("utf-8"), (C.GS_IP, C.GS_PORT))
        except OSError:
            pass

    def stop(self):
        self._stop = True
        try:
            self.sock.close()
        except OSError:
            pass
