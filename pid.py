#!/usr/bin/env python

import time


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

        # template variable
        self.current_time = time.time()
        self.last_time = self.current_time
        self.PTerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.output = 0.0

    def update(self, feedback_value):
        # for steer, err = feedback - target, so kp < 0 and kd < 0
        # for speed, err = feedback - target, so kp > 0 and kd > 0
        error = self.target - feedback_value
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error
        self.PTerm = self.kp * error
        self.DTerm = 0.0
        if delta_time > 0:
            self.DTerm = delta_error / delta_time
        self.last_time = self.current_time
        self.last_error = error
        self.output = self.target + self.PTerm + (self.kd * self.DTerm)
        return self.output

    def clear(self):
        self.DTerm = 0.0

    def set_target(self, target):
        self.target = target
