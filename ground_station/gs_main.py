# -*- coding: utf-8 -*-
"""地面站主界面（Tkinter）：场地实时位置 + 无人机状态文字

用法：
    python gs_main.py --sim       # 仿真模式
    python gs_main.py             # 接收真实遥测
"""
import argparse
import math
import time
import tkinter as tk

import config as C
import track
from network import Network
from simulator import Simulator


def to_canvas(x_cm, y_cm):
    px = C.MARGIN_X + x_cm * C.SCALE
    py = C.MARGIN_Y + (C.FIELD_H - y_cm) * C.SCALE
    return px, py


class GroundStationApp:
    def __init__(self, root, net, simulate):
        self.root = root
        self.net = net
        self.simulate = simulate
        self.mode = "DELIVERY"
        self.sim = None

        root.title("地面站 · 陆空协同无人机系统 (D题)")
        root.geometry("1180x1080")
        root.configure(bg="#f0f0f0")

        top = tk.Frame(root, bg="#f0f0f0")
        top.pack(fill="x", padx=8, pady=6)

        tk.Label(top, text="任务模式:", bg="#f0f0f0").pack(side="left")
        self.mode_var = tk.StringVar(value="DELIVERY")
        tk.Radiobutton(top, text="抛投任务", variable=self.mode_var, value="DELIVERY",
                       command=self._on_mode, bg="#f0f0f0").pack(side="left")
        tk.Radiobutton(top, text="动态起降", variable=self.mode_var, value="DYNAMIC_LAND",
                       command=self._on_mode, bg="#f0f0f0").pack(side="left")

        self.btn_start = tk.Button(top, text="启动仿真", command=self._start_sim,
                                   bg="#1f6fb2", fg="white", state="normal")
        self.btn_start.pack(side="left", padx=10)
        tk.Button(top, text="急停 ESTOP", command=lambda: self.net.send_cmd("ESTOP", "car"),
                  bg="#c62828", fg="white").pack(side="left", padx=2)
        tk.Button(top, text="恢复 RESUME", command=lambda: self.net.send_cmd("RESUME", "car"),
                  bg="#2e7d32", fg="white").pack(side="left", padx=2)

        self.link_lbl = tk.Label(top, text="链路: 等待数据...", bg="#f0f0f0", fg="#888")
        self.link_lbl.pack(side="right")

        body = tk.Frame(root)
        body.pack(fill="both", expand=True)

        # 场地画布
        self.canvas = tk.Canvas(body, width=C.CANVAS_W, height=C.CANVAS_H, bg=C.COL_BG,
                                highlightthickness=1, highlightbackground="#bbb")
        self.canvas.pack(side="left", padx=8, pady=8)
        self._draw_static()

        # 右侧面板
        right = tk.Frame(body, width=340)
        right.pack(side="left", fill="y", padx=8, pady=8)
        right.pack_propagate(False)

        tk.Label(right, text="无人机状态", font=("Microsoft YaHei", 12, "bold")).pack(pady=4)
        self.status_lbl = tk.Label(right, text="IDLE", font=("Consolas", 26, "bold"),
                                   fg=C.COL_STATUS, bg="#fff", width=16, relief="solid")
        self.status_lbl.pack(pady=4)

        tk.Label(right, text="— 数据面板 —", font=("Microsoft YaHei", 10, "bold")).pack(pady=(14, 2))
        self.data_lbl = tk.Label(right, text="", font=("Consolas", 11), justify="left",
                                 anchor="w", bg="#fff", relief="solid", width=34, height=9)
        self.data_lbl.pack(pady=4)

        tk.Label(right, text="— 事件日志 —", font=("Microsoft YaHei", 10, "bold")).pack(pady=(14, 2))
        self.log_lbl = tk.Label(right, text="", font=("Consolas", 10), justify="left",
                                anchor="nw", bg="#fff", relief="solid", width=42, height=16)
        self.log_lbl.pack(pady=4)

        self._last_redraw = 0.0

    # ---------------- 静态场景 ----------------
    def _draw_static(self):
        cv = self.canvas
        cv.delete("static")
        # 场地边框
        x0, y0 = to_canvas(0, 0)
        x1, y1 = to_canvas(C.FIELD_W, C.FIELD_H)
        cv.create_rectangle(x0, y0, x1, y1, outline=C.COL_LINE, width=2, tags="static")
        cv.create_text((x0 + x1) / 2, y0 - 14, text="400cm", tags="static")
        cv.create_text(x0 - 16, (y0 + y1) / 2, text="500cm", tags="static")

        # 轨迹：按 track_position 采样绘制（与固件/仿真完全一致）
        pts = []
        step = 5.0
        s = 0.0
        while s <= track.TRACK_L + 0.01:
            x, y, _ = track.track_position(s)
            pts.extend(to_canvas(x, y))
            s += step
        cv.create_line(*pts, fill=C.COL_LINE, width=2, tags="static")

        # 关键点
        for name, (fx, fy) in {"A": (200, 100), "B": (200, 400), "C": (275, 325),
                               "D": (275, 175)}.items():
            px, py = to_canvas(fx, fy)
            cv.create_oval(px - 5, py - 5, px + 5, py + 5, fill="#fff",
                           outline="#000", width=1.5, tags="static")
            cv.create_text(px, py - 14, text=name, font=("Microsoft YaHei", 12, "bold"),
                           tags="static")
        # 起降点 H
        px, py = to_canvas(track.HOME_X, track.HOME_Y)
        cv.create_rectangle(px - 15, py - 15, px + 15, py + 15, outline="#000",
                            fill="#fff", width=1.5, tags="static")
        cv.create_text(px, py - 20, text="H", font=("Microsoft YaHei", 12, "bold"), tags="static")

    # ---------------- 动态绘制 ----------------
    def _draw_dynamic(self, car, uav):
        cv = self.canvas
        cv.delete("dyn")
        if car:
            x, y = car.get("x", 0), car.get("y", 0)
            psi = car.get("psi", 90)
            px, py = to_canvas(x, y)
            a = math.radians(psi)
            # 小车：朝向箭头 + 平台同心圆
            lx, ly = px + 18 * math.cos(a), py - 18 * math.sin(a)
            cv.create_line(px, py, lx, ly, fill=C.COL_CAR, width=4,
                           arrow="last", arrowshape=(10, 12, 4), tags="dyn")
            for r in (15.0, 25.0):
                rr = r * C.SCALE
                cv.create_oval(px - rr, py - rr, px + rr, py + rr,
                               outline=C.COL_CAR, width=2, tags="dyn")
            cv.create_line(px - rr, py, px + rr, py, fill=C.COL_CAR, width=1, tags="dyn")
            cv.create_line(px, py - rr, px, py + rr, fill=C.COL_CAR, width=1, tags="dyn")
            cv.create_text(px, py - 34, text=f"小车 {x:.0f},{y:.0f}",
                           font=("Consolas", 9), fill=C.COL_CAR, tags="dyn")

        if uav:
            x, y = uav.get("x", 0), uav.get("y", 0)
            alt = uav.get("alt", 0)
            px, py = to_canvas(x, y)
            r = 10
            cv.create_oval(px - r, py - r, px + r, py + r, fill=C.COL_UAV,
                           outline="#000", tags="dyn")
            cv.create_line(px - r, py, px + r, py, fill="#000", tags="dyn")
            cv.create_line(px, py - r, px, py + r, fill="#000", tags="dyn")
            cv.create_text(px, py - 20, text=f"无人机 高{alt:.1f}m",
                           font=("Consolas", 9), fill=C.COL_UAV, tags="dyn")

    # ---------------- 状态文字 ----------------
    def _state_color(self, st):
        danger = {"DYN_DESCEND", "LAND", "FAIL", "DELIVERY"}
        ok = {"HOVER", "FORMATION", "DYN_HOLD", "DONE"}
        if st in danger:
            return C.COL_UAV_DANGER
        if st in ok:
            return "#2e7d32"
        return C.COL_STATUS

    def _refresh(self):
        car, uav, cts, uts, logs = self.net.snapshot()
        now = time.time()
        self._draw_dynamic(car, uav)

        # 链路状态
        link = []
        if car is not None and now - cts < 2:
            link.append("小车 ✓")
        if uav is not None and now - uts < 2:
            link.append("无人机 ✓")
        self.link_lbl.config(text="链路: " + ("  ".join(link) if link else "等待数据..."),
                             fg="#2e7d32" if len(link) == 2 else "#c62828")

        # 无人机状态
        st = (uav or {}).get("state", "IDLE")
        self.status_lbl.config(text=st, fg=self._state_color(st))

        # 数据面板
        c = car or {}
        u = uav or {}
        self.data_lbl.config(text=(
            f"小车里程 s : {c.get('s', '-'):>7} cm\n"
            f"小车速度 v : {c.get('v', '-'):>7} cm/s\n"
            f"小车位置   : ({c.get('x', '-'):>6}, {c.get('y', '-'):>6})\n"
            f"无人机高度 : {u.get('alt', '-'):>7} m\n"
            f"无人机速度 : ({u.get('vx', 0):>5}, {u.get('vy', 0):>5}) m/s\n"
            f"无人机电池 : {u.get('batt', '-')} V\n"
            f"任务模式   : {self.mode}"))

        # 日志
        self.log_lbl.config(text="\n".join(logs[-14:]))

        self.root.after(100, self._refresh)

    # ---------------- 仿真 ----------------
    def _on_mode(self):
        self.mode = self.mode_var.get()

    def _start_sim(self):
        if self.sim is not None:
            self.sim.stop()
        self.sim = Simulator(self.net, mode=self.mode)
        self.sim.start()
        self.btn_start.config(text="仿真运行中...", state="disabled")

    def run(self):
        self._refresh()
        self.root.mainloop()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", action="store_true", help="仿真模式")
    args = ap.parse_args()

    net = Network()
    net.start()

    root = tk.Tk()
    app = GroundStationApp(root, net, simulate=args.sim)
    app.run()


if __name__ == "__main__":
    main()
