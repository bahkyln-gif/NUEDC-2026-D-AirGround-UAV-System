# -*- coding: utf-8 -*-
"""轨迹几何校验工具：验证分段距离、关键点坐标与闭环一致性"""
import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ground_station"))
import track


def main():
    print("=== 轨迹几何校验 ===")
    print(f"总长 TRACK_L = {track.TRACK_L:.1f} cm（理论 771.2）")

    # 关键点里程
    keys = {"A(起点)": 0.0, "B": track.SEG_B, "C": track.SEG_C, "D": track.SEG_D,
            "A(一圈)": track.TRACK_L}
    print("\n关键点位置（场地坐标 cm）：")
    for name, s in keys.items():
        x, y, psi = track.track_position(s)
        print(f"  {name:>8} s={s:6.1f} → ({x:6.1f}, {y:6.1f}) psi={psi:6.1f}°")

    # 一致性检查
    x0, y0, _ = track.track_position(0.0)
    x1, y1, _ = track.track_position(track.TRACK_L)
    assert abs(x0 - x1) < 1e-6 and abs(y0 - y1) < 1e-6, "闭环不闭合！"
    assert abs(track.TRACK_L - 771.2) < 1.0, "总长异常！"

    # 分段距离（与设计报告一致）
    print("\n分段距离：")
    segs = [("A→B", 0, track.SEG_B), ("A→C", 0, track.SEG_C), ("A→D", 0, track.SEG_D)]
    for name, a, b in segs:
        print(f"  {name}: {b - a:6.1f} cm")

    # 方向连续性（相邻采样航向变化 < 60°/步）
    print("\n航向连续性（5cm 步长）：")
    prev_psi = None
    max_delta = 0.0
    for s in range(0, int(track.TRACK_L), 5):
        _, _, psi = track.track_position(float(s))
        if prev_psi is not None:
            d = abs(psi - prev_psi)
            d = min(d, 360 - d)
            max_delta = max(max_delta, d)
        prev_psi = psi
    print(f"  最大相邻航向跳变 = {max_delta:.1f}°（应 < 60°）")
    assert max_delta < 60, "航向跳变过大！"

    print("\n✓ 轨迹几何校验通过")


if __name__ == "__main__":
    main()
