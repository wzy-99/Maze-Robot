# !/usr/bin/env python

import time
import RPi.GPIO as GPIO
from pid import PD
from constant import DirctionEnum, EncodeInfo, Pin


class Motor:
    def __init__(self, pin1, pin2, enable, a, b, kp=1.0, kd=1.0):
        enum = DirctionEnum()
        self.forward = enum.dir_forward
        self.back = enum.dir_back
        del enum

        # config
        self.kp = kp
        self.kd = kd
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
        GPIO.setup(self.pin1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.pin2, GPIO.OUT, initial=GPIO.LOW)
        if self.enable is not None:
            GPIO.setup(self.enable, GPIO.OUT, initial=GPIO.High)

    def set_speed(self, speed):
        self.target_speed = min(max(speed, self.min_speed), self.max_speed)
        self.pd.set_target(self.target_speed)

    def set_pwm(self, speed):
        # TODO stop last pwm
        self.pwm_value = speed * self.pwm_k
        self.pwm = self.pwm.start(self.pwm_value)

    def set_forword(self):
        # TODO delect last pwm object
        self.pwm.stop()
        self.pwm = GPIO.PWM(self.pin1, self.pwm_frq)
        GPIO.output(self.pin2, 0)

    def set_back(self):
        # TODO delect last pwm object
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
        speed = min(max(speed, self.min_speed), self.max_speed)
        self.set_pwm(speed)

    def __del__(self):
        pass


class MotorOpen:
    def __init__(self, pin1, pin2, enable):
        enum = DirctionEnum()
        self.forward = enum.dir_forward
        self.back = enum.dir_back
        del enum

        # config
        self.max_speed = 100
        self.min_speed = 0
        self.pwm_k = 0.01
        self.pwm_frq = 1000

        self.pin1 = pin1
        self.pin2 = pin2
        self.enable = enable

        self.pwm_value = 0
        self.speed = 0
        self.pwm = None
        self.dir = self.forward

        self.init()

    def init(self):
        # GPIO.setup(self.pin1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.pin2, GPIO.OUT, initial=GPIO.LOW)
        if self.enable is not None:
            GPIO.setup(self.enable, GPIO.OUT, initial=GPIO.High)
        self.pwm = GPIO.PWM(self.pin1, self.pwm_frq)
        self.pwm.start(0)

    def set_speed(self, speed):
        self.speed = min(max(speed, self.min_speed), self.max_speed)

    def set_pwm(self):
        self.pwm_value = self.speed * self.pwm_k

    def pwm_output(self):
        self.pwm.chaneDuty(self.pwm_value)

    def set_forword(self):
        # TODO delect last pwm object
        self.pwm.stop()
        self.pwm = GPIO.PWM(self.pin1, self.pwm_frq)
        GPIO.output(self.pin2, GPIO.LOW)

    def set_back(self):
        # TODO delect last pwm object
        self.pwm.stop()
        self.pwm = GPIO.PWM(self.pin2, self.pwm_frq)
        GPIO.output(self.pin1, GPIO.LOW)

    def set_direction(self, dirction):
        if dirction == self.forward:
            self.set_forword()
        elif dirction == self.back:
            self.set_back()

    def spin(self):
        self.pwm_output()

    def __del__(self):
        if self.pwm is not None:
            self.pwm.stop()
            del self.pwm


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
        GPIO.setup(self.pin, GPIO.IN)

    def check_obstacle(self):
        self.obstacle = GPIO.input(self.pin)
        return self.obstacle

    def spin(self):
        self.obstacle = GPIO.input(self.pin)
        return self.obstacle

    def __del__(self):
        pass


class Radar:
    def __init__(self, trig, echo, thres):
        self.trig = trig
        self.echo = echo

        self.distance = 0
        self.thres_distance = thres
        self.time_start = None
        self.time_end = None
        self.obstacle = False

        self.max_try_time = 100000

    def init(self):
        GPIO.setup(self.trig, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.echo, GPIO.IN, initial=GPIO.LOW)

    def get_distance(self):
        GPIO.output(self.trig, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.trig, GPIO.LOW)
        max_try_time = self.max_try_time
        try_time = 0
        while GPIO.input(self.trig) == 0:
            try_time = try_time + 1
            if try_time > max_try_time:
                self.distance = 100000
                return self.distance
            else:
                continue
        self.time_start = time.time()
        try_time = 0
        while GPIO.input(self.trig) == 1:
            try_time = try_time + 1
            if try_time > max_try_time:
                self.distance = 100000
                return self.distance
            else:
                continue
        self.time_end = time.time()
        self.distance = (self.time_start - self.time_end) * 17150
        return self.distance

    def check_obstacle(self):
        if self.distance < self.thres_distance:
            self.obstacle = True
        else:
            self.obstacle = False
        return self.obstacle

    def spin(self):
        return self.get_distance()


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
        GPIO.setup(self.A, GPIO.IN)
        GPIO.setup(self.B, GPIO.IN)
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


# for test
if __name__ == '__main__':
    pin_conf = Pin()
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pin_conf.left_pin1, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(pin_conf.left_pin2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(pin_conf.left_enale, GPIO.OUT, initial=GPIO.HIGH)

    # left_motor = Motor(pin_conf.left_pin1,
    #                    pin_conf.left_pin2,
    #                    pin_conf.left_enale,
    #                    None, None)
