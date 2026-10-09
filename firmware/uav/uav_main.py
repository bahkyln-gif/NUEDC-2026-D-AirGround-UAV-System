# -*- coding: utf-8 -*-
"""无人机端主程序：20Hz 控制环 + 10Hz 遥测

用法：
    python3 uav_main.py          # 真实运行
    python3 uav_main.py --sim    # 仿真模式（无飞控/相机/舵机）
"""
import argparse
import time

import config as C
from comms import Comms
from mavlink_ctl import MavlinkCtl
from vision import DownVision
from delivery import Delivery
from state_machine import StateMachine


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", action="store_true", help="仿真模式")
    args = ap.parse_args()
    simulate = args.sim or C.SIMULATE

    comms = Comms()
    mav = MavlinkCtl(simulate=simulate)
    vis = DownVision(simulate=simulate)
    deliv = Delivery(simulate=simulate)
    fsm = StateMachine(comms, mav, vis, deliv)

    comms.start()
    mav.start()
    deliv.stow()

    print("[uav] start, sim =", simulate)

    ctrl_dt = 1.0 / C.CTRL_HZ
    tele_t = 0.0
    last = time.time()

    try:
        while True:
            now = time.time()
            dt = min(now - last, 0.1)
            last = now

            tx, ty, tz, yaw, action = fsm.update(dt)

            # 执行动作
            if action == "drop":
                deliv.drop()
            elif action == "land":
                mav.land()
            elif action == "arm":
                mav.arm()
            elif action == "disarm":
                mav.disarm()

            # 发送位置设定点（GUIDED 位置控制）
            if fsm.state != "IDLE":
                mav.set_position_ned(tx, ty, tz, yaw)

            # 遥测 10Hz
            tele_t += dt
            if tele_t >= 1.0 / C.TELE_HZ:
                tele_t = 0.0
                st = mav.get_state()
                fx = C.HOME_X + st["x"] * 100.0   # NED(m) → 场地(cm)
                fy = C.HOME_Y + st["y"] * 100.0
                alt = -st["z"]
                comms.send_telemetry(
                    fx, fy, alt, st["vx"], st["vy"], st["yaw"],
                    fsm.state_text(), batt=st.get("batt"))

            time.sleep(ctrl_dt / 2.0)
    except KeyboardInterrupt:
        print("[uav] exit")
    finally:
        deliv.cleanup()
        vis.release()
        comms.stop()
        mav.stop()


if __name__ == "__main__":
    main()
