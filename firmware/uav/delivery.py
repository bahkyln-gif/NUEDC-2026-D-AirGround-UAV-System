# -*- coding: utf-8 -*-
"""舵机抛投机构：拨杆释放 25±5g 软质物"""
import time

try:
    import RPi.GPIO as GPIO
    _HAS_GPIO = True
except ImportError:
    _HAS_GPIO = False

SERVO_PIN = 17      # 对应 uav_pinout.md GPIO17 (pin 11)
STOW_DUTY = 2.5     # 舵机收拢（托住）
DROP_DUTY = 7.5     # 舵机放开（释放）


class Delivery:
    def __init__(self, simulate=False):
        self.sim = simulate
        self.dropped = False
        if not simulate and _HAS_GPIO:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(SERVO_PIN, GPIO.OUT)
            self.pwm = GPIO.PWM(SERVO_PIN, 50)
            self.pwm.start(STOW_DUTY)
        else:
            self.pwm = None

    def _duty(self, d):
        if self.pwm is not None:
            self.pwm.ChangeDutyCycle(d)
        time.sleep(0.2)

    def stow(self):
        """收拢拨杆，准备装载"""
        self.dropped = False
        self._duty(STOW_DUTY)

    def drop(self):
        """释放软质物"""
        self.dropped = True
        print("[delivery] DROP")
        self._duty(DROP_DUTY)
        time.sleep(0.5)
        self._duty(STOW_DUTY)

    def cleanup(self):
        if self.pwm is not None:
            self.pwm.stop()
        if not self.sim and _HAS_GPIO:
            GPIO.cleanup()
