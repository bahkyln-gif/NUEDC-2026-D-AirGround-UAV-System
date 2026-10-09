# -*- coding: utf-8 -*-
"""下视同心圆识别（动态降落视觉引导）

利用平台图案（外圆直径 50cm）按"检测半径 vs 真实半径"自标定地面分辨率，
无需逐高度标定。输出图像中心到外圆圆心的偏差（cm）。

cv2/numpy 采用惰性导入：未安装 OpenCV 的环境也可导入本模块（如逻辑测试），
实际使用相机时才需要 opencv-python。
"""
PLATFORM_OUTER_R = 25.0   # 外圆半径 cm（直径 50cm）


class DownVision:
    def __init__(self, simulate=False, cam_index=0, width=640, height=480):
        self.sim = simulate
        self.cap = None
        self.cv2 = None
        self.np = None
        self.W = width
        self.H = height
        if not simulate:
            try:
                import cv2
                import numpy as np
            except ImportError:
                raise RuntimeError("需要 opencv-python：pip install opencv-python")
            self.cv2 = cv2
            self.np = np
            self.cap = cv2.VideoCapture(cam_index)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def read_frame(self):
        if self.sim or self.cap is None:
            return None
        ok, frame = self.cap.read()
        return frame if ok else None

    def detect(self, frame=None):
        """返回 (dx_cm, dy_cm, found)；偏差=图像中心→外圆圆心的向量，单位 cm"""
        if self.sim or self.cv2 is None:
            return 0.0, 0.0, False
        cv2 = self.cv2
        np = self.np
        if frame is None:
            frame = self.read_frame()
            if frame is None:
                return 0.0, 0.0, False

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # 黑图案：低灰度阈值二值化
        _, th = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV)
        cnts, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        for c in cnts:
            area = cv2.contourArea(c)
            if area < 200:
                continue
            (cx, cy), r = cv2.minEnclosingCircle(c)
            # 圆形度筛选
            perimeter = cv2.arcLength(c, True)
            circularity = 4.0 * np.pi * area / (perimeter * perimeter + 1e-6)
            if circularity < 0.6:
                continue
            if best is None or r > best[2]:
                best = (cx, cy, r)

        if best is None:
            return 0.0, 0.0, False

        cx, cy, r_px = best
        # 自标定：k = 真实半径 / 像素半径（cm/px）
        k = PLATFORM_OUTER_R / r_px
        dx_cm = (cx - self.W / 2.0) * k
        dy_cm = (cy - self.H / 2.0) * k
        return dx_cm, dy_cm, True

    def release(self):
        if self.cap is not None:
            self.cap.release()
