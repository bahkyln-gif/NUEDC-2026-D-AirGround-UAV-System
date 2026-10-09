# -*- coding: utf-8 -*-
"""UDP 网络：接收小车/无人机遥测，发送地面站指令"""
import json
import socket
import threading
import time

import config as C


class Network:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((C.BIND_IP, C.GS_PORT))
        self.sock.settimeout(0.05)

        self.lock = threading.Lock()
        self.car = None
        self.uav = None
        self.car_ts = 0.0
        self.uav_ts = 0.0
        self.log = []          # 最近消息（供界面日志）
        self.csv_file = None
        self._stop = False

    def start(self):
        threading.Thread(target=self._rx_loop, daemon=True).start()
        self._open_csv()

    def _open_csv(self):
        import datetime
        import os
        os.makedirs("logs", exist_ok=True)
        name = "logs/gs_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"
        self.csv_file = open(name, "w", encoding="utf-8")
        self.csv_file.write("ts,car_x,car_y,car_s,car_v,uav_x,uav_y,uav_alt,uav_state\n")

    def _rx_loop(self):
        while not self._stop:
            try:
                data, addr = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                msg = json.loads(data.decode("utf-8"))
            except Exception:
                continue
            self._handle(msg, addr)

    def _handle(self, msg, addr):
        t = msg.get("type")
        d = msg.get("data", {})
        now = time.time()
        with self.lock:
            if t == "CAR_TELEMETRY":
                self.car = d
                self.car_ts = now
            elif t == "UAV_TELEMETRY":
                self.uav = d
                self.uav_ts = now
            elif t in ("CAR_DONE", "UAV_DONE"):
                self.log.append(f"[{time.strftime('%H:%M:%S')}] {t} {d}")
                if len(self.log) > 200:
                    self.log.pop(0)
        self._write_csv(now)

    def _write_csv(self, now):
        if self.csv_file is None:
            return
        with self.lock:
            c = self.car or {}
            u = self.uav or {}
            line = f"{now:.1f},{c.get('x','')},{c.get('y','')},{c.get('s','')},{c.get('v','')}," \
                   f"{u.get('x','')},{u.get('y','')},{u.get('alt','')},{u.get('state','')}\n"
        try:
            self.csv_file.write(line)
            self.csv_file.flush()
        except OSError:
            pass

    def send_cmd(self, cmd, target="car"):
        msg = {"type": "GS_CMD", "ts": time.time(), "from": "gs",
               "data": {"cmd": cmd, "target": target}}
        data = json.dumps(msg).encode("utf-8")
        # 广播 + 定向，保证到达
        try:
            self.sock.sendto(data, (C.BROADCAST, C.GS_PORT))
            if target == "uav":
                self.sock.sendto(data, (C.UAV_IP, C.GS_PORT + 1))
            elif target == "car":
                self.sock.sendto(data, (C.CAR_IP, C.GS_PORT + 2))
        except OSError:
            pass

    def snapshot(self):
        with self.lock:
            return self.car, self.uav, self.car_ts, self.uav_ts, list(self.log)

    def stop(self):
        self._stop = True
        try:
            self.sock.close()
        except OSError:
            pass
        if self.csv_file:
            self.csv_file.close()
