# !/usr/bin/env python

class PID:
    def __init__(self, kp, ki, kd, target):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target = target
        self.last_err = 0
        self.previous_err = 0

    def update(self, value):
        pass

    def clear(self):
        pass


class PD:
    def __init__(self, kp, kd, target):
        self.kp = kp
        self.kd = kd
        self.target = target

    def update(self):
        pass

    def clear(self):
        pass

    def set_target(self, target):
        self.target = target
