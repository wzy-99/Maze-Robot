# !/usr/bin/env python

import time
import RPi.GPIO as GPIO
from pid import PD
from constant import DirctionEnum, EncodeInfo


class Motor:
    def __init__(self, pin1, pin2, enable, a, b):
        dir_enum = DirctionEnum()
        self.forward = dir_enum.dir_forward
        self.back = dir_enum.dir_back
        self.left = dir_enum.dir_left
        self.right = dir_enum.dir_right
        del dir_enum

        # config
        self.kp = 1.0
        self.kd = 1.0
        self.max_speed = 100
        self.min_speed = 0
        self.pwm_k = 0.01
        self.pwm_frq = 1000

        self.pin1 = pin1
        self.pin2 = pin2
        self.enable = enable

        self.pwm_value = 0
        self.target_speed = 0
        self.current_speed = 0
        self.dir = self.forward

        self.encode = Encoder(a, b)
        self.pwm = GPIO.PWM(pin1, self.pwm_frq)
        self.pd = PD(self.kp, self.kd, self.target_speed)

        self.init()

    def init(self):
        # TODO resistance pull up or down
        GPIO.setup(self.pin1, GPIO.OUT, )
        GPIO.setup(self.pin2, GPIO.OUT, )
        if self.enable is not None:
            GPIO.setup(self.enable, GPIO.OUT, )

    def set_speed(self, speed):
        self.target_speed = min(max(speed, self.min_speed), self.max_speed)
        self.pd.set_target(self.target_speed)

    def set_pwm(self, speed):
        self.pwm_value = speed * self.pwm_k
        self.pwm = self.pwm.start(self.pwm)

    def set_forword(self):
        self.pwm.stop()
        self.pwm = GPIO.PWM(self.pin1, self.pwm_frq)
        GPIO.output(self.pin2, 0)

    def set_back(self):
        self.pwm.stop()
        self.pwm = GPIO.PWM(self.pin2, self.pwm_frq)
        GPIO.output(self.pin1, 0)

    def set_direction(self, dirction):
        if dirction == self.forward:
            self.set_forword()
        elif dirction == self.back:
            self.set_back()

    def spin(self):
        self.current_speed = self.encode.get_actual_speed()
        speed = self.pd.update(self.current_speed)
        self.set_pwm(speed)

    def __del__(self):
        pass


class Steer:
    def __init__(self, pin):
        self.pin = pin

    def set_angle(self):
        pass

    def set_pwm(self):
        pass

    def __del__(self):
        pass


class InfraRed:
    def __init__(self, pin):
        self.pin = pin
        self.obstacle = False

        self.init()

    def init(self):
        # TODO resistance pull up or down
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def get_distance(self):
        pass

    def get_pwm(self):
        pass

    def check_obstacle(self):
        self.obstacle = GPIO.input(self.pin)
        return self.obstacle

    def __del__(self):
        pass


class Radar:
    def __init__(self, pin):
        self.pin = pin

    def get_distance(self):
        pass

    def get_pwm(self):
        pass

    def check_obstacle(self):
        pass


class Led:
    def __init__(self, pin):
        self.pin = pin

    def set_brightness(self):
        pass

    def set_pwm(self):
        pass


class Encoder:
    def __init__(self, a, b):
        self.A = a
        self.B = b
        self.init()

        info = EncodeInfo()
        self.count_per_cm = info.count_per_cm
        self.cm_per_grid = info.cm_per_grid
        del info

        self.count = 0
        self.last_count = 0
        self.grid = 0
        self.last_grid = 0
        self.distance = 0
        self.renew = False
        self.time = time.time()

    def init(self):
        # TODO resistance pull up or down
        GPIO.setup(self.A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.A, GPIO.RISING, callback=self.count_callback)

    def count_callback(self):
        current_a = GPIO.input(self.A)
        current_b = GPIO.input(self.B)
        if current_a:
            if current_b:
                self.count = self.count - 1
            else:
                self.count = self.count + 1

    def get_count(self):
        return self.count

    def get_distance(self):
        self.distance = self.count / self.count_per_cm
        return self.distance

    def get_actual_speed(self):
        d_count = self.count - self.last_count
        d_distance = d_count / self.count_per_cm
        current_time = time.time()
        d_time = current_time - self.time
        vel = d_distance / d_time
        self.last_count = self.count
        self.time = current_time
        return vel

    def get_grid(self):
        self.distance = self.count / self.count_per_cm
        self.grid = round(self.distance / self.cm_per_grid)
        if self.last_grid != self.grid:
            self.renew = True
            self.last_grid = self.grid
        return self.grid

    def new_grid(self):
        if self.renew:
            self.renew = False
            return True
        else:
            return False
