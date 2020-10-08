#!/usr/bin/env python

import time
import math
from constant import StateEnum
from pid import PD


# In this class, kinematic control method is defined, such as turn back, turn right and so on.
# And you should know that belowed methond is high-level control one rather than basic control methond.
class KinematicControl:
    def __init__(self, motor_instance):

        state = StateEnum()
        self.stop = state.stop
        self.run_forward = state.run_forward
        self.run_back = state.run_forward
        self.turn_left = state.turn_left
        self.turn_right = state.turn_right
        self.turn_back = state.turn_back
        self.turn_left_unblock = state.turn_left_unblock
        self.turn_right_unblock = self.turn_right_unblock
        del state

        # config variable
        self.turn_speed = 100
        self.turn_dtime = 1.0
        self.adjust_k = 0.05
        self.angle_pd = PD(kd=-1.0, kp=-1.0, target=50.0)

        # motor instance
        self.left_motor, self.right_motor = motor_instance

        # inital state is forward
        # inital speed is 0
        # inital angle is 50

        # local variable
        self.angle = 50
        self.speed = 0
        self.left_speed = 0
        self.right_speed = 0
        self.state = self.run_forward
        self.last_state = self.run_forward
        self.state_change = False
        self.start_time = 0
        self.end_time = 0
        self.block = False

    def set_state(self, state):
        if self.state != state and self.block is False:
            if self.state == self.run_forward or \
                    self.state == self.run_back or \
                    self.state == self.stop:
                self.last_state = self.state
            self.state = state
            self.state_change = True

    def set_angle(self, angle=50):
        self.angle = int(max(0, min(angle, 50)))

    def set_speed(self, speed=0):
        self.speed = int(max(0, min(speed, 100)))

    def turn_right_(self):
        if self.state_change:
            self.left_motor.set_forword()
            self.right_motor.set_back()
            self.state_change = True
            self.left_speed = self.turn_speed
            self.right_speed = self.turn_speed
            self.left_motor.set_speed(self.left_speed)
            self.right_motor.set_speed(self.right_speed)
            self.start_time = time.time()
            self.end_time = self.start_time + self.turn_dtime
            self.block = True
        else:
            if time.time() < self.end_time:
                self.left_speed = self.turn_speed
                self.right_speed = self.turn_speed
                self.left_motor.set_speed(self.left_speed)
                self.right_motor.set_speed(self.right_speed)
            else:
                self.start_time = 0
                self.end_time = 0
                self.state = self.last_state
                self.state_change = True
                self.block = False

    def turn_left_(self):
        if self.state_change:
            self.left_motor.set_back()
            self.right_motor.set_forword()
            self.state_change = False
            self.left_speed = self.turn_speed
            self.right_speed = self.turn_speed
            self.left_motor.set_speed(self.left_speed)
            self.right_motor.set_speed(self.right_speed)
            self.start_time = time.time()
            self.end_time = self.start_time + self.turn_dtime
            self.block = True
        else:
            if time.time() < self.end_time:
                self.left_speed = self.turn_speed
                self.right_speed = self.turn_speed
                self.left_motor.set_speed(self.left_speed)
                self.right_motor.set_speed(self.right_speed)
            else:
                self.start_time = 0
                self.end_time = 0
                self.state = self.last_state
                self.state_change = True
                self.block = False

    def turn_left_unblock_(self):
        if self.state_change:
            self.left_motor.set_back()
            self.right_motor.set_forword()
            self.state_change = True
            self.left_speed = self.turn_speed
            self.right_speed = self.turn_speed
            self.left_motor.set_speed(self.left_speed)
            self.right_motor.set_speed(self.right_speed)
        else:
            self.left_speed = self.turn_speed
            self.right_speed = self.turn_speed
            self.left_motor.set_speed(self.left_speed)
            self.right_motor.set_speed(self.right_speed)

    def turn_right_unblock_(self):
        if self.state_change:
            self.left_motor.set_back()
            self.right_motor.set_forword()
            self.state_change = False
            self.left_speed = self.turn_speed
            self.right_speed = self.turn_speed
            self.left_motor.set_speed(self.left_speed)
            self.right_motor.set_speed(self.right_speed)
        else:
            self.left_speed = self.turn_speed
            self.right_speed = self.turn_speed
            self.left_motor.set_speed(self.left_speed)
            self.right_motor.set_speed(self.right_speed)

    def turn_back_(self):
        if self.state_change:
            self.left_motor.set_forword()
            self.right_motor.set_back()
            self.state_change = False
            self.left_speed = self.turn_speed
            self.right_speed = self.turn_speed
            self.left_motor.set_speed(self.left_speed)
            self.right_motor.set_speed(self.right_speed)
            self.start_time = time.time()
            self.end_time = self.start_time + self.turn_dtime * 2
            self.block = True
        else:
            if time.time() < self.end_time:
                self.left_speed = self.turn_speed
                self.right_speed = self.turn_speed
                self.left_motor.set_speed(self.left_speed)
                self.right_motor.set_speed(self.right_speed)
            else:
                self.start_time = 0
                self.end_time = 0
                self.state = self.last_state
                self.state_change = True
                self.block = False

    def run_forward_(self):
        if self.state_change:
            self.left_motor.set_forword()
            self.right_motor.set_forword()
            self.state_change = False
        else:
            self.dif_speed(self.speed, self.angle)
            self.left_motor.set_speed(self.left_speed)
            self.right_motor.set_speed(self.right_speed)

    def run_back_(self):
        if self.state_change:
            self.left_motor.set_back()
            self.right_motor.set_back()
            self.state_change = False
        else:
            self.dif_speed(self.speed, 100 - self.angle)
            self.left_motor.set_speed(self.left_speed)
            self.right_motor.set_speed(self.right_speed)

    def stop_(self):
        if self.state_change:
            self.left_speed = 0
            self.right_speed = 0
            self.left_motor.set_speed(self.left_speed)
            self.right_motor.set_speed(self.right_speed)
            self.state_change = False
        else:
            self.left_speed = 0
            self.right_speed = 0
            self.left_motor.set_speed(self.left_speed)
            self.right_motor.set_speed(self.right_speed)

    def adjust_angle(self, left_distance, right_distance):
        d_distance = left_distance - right_distance
        self.angle = self.angle_pd.update(d_distance * self.adjust_k)

    def dif_speed(self, speed, angle):
        # TODO dif speed algorithm
        angle = (1 - angle / 100.0) * math.pi
        y_speed = speed * math.sin(angle)
        x_speed = speed * math.cos(angle)
        # when car turn left, angle > 90, x_speed < 0;
        # when car turn right, angle < 90, x_speed > 0.
        self.left_speed = y_speed + x_speed // 2
        self.right_speed = y_speed - x_speed // 2
        self.left_speed = int(max(0, min(100, self.left_speed)))
        self.right_speed = int(max(0, min(100, self.right_speed)))

    def spin(self):
        if self.state == self.run_forward:
            self.run_forward_()
        elif self.state == self.run_back:
            self.run_back_()
        elif self.state == self.turn_left:
            self.turn_left_()
        elif self.state == self.turn_right:
            self.turn_right_()
        elif self.state == self.turn_back:
            self.turn_back_()
        elif self.state == self.stop:
            self.stop_()
        elif self.state == self.turn_left_unblock:
            self.turn_left_unblock_()
        elif self.state == self.turn_right_unblock:
            self.turn_right_unblock_()
        self.left_motor.spin()
        self.right_motor.spin()


# for test
if __name__ == '__main__':
    import RPi.GPIO as GPIO
    from constant import Pin
    from peripheral import MotorOpen
    pin_conf = Pin()
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    motor_left = MotorOpen(pin_conf.left_pin1, pin_conf.left_pin2, pin_conf.left_enale)
    motor_right = MotorOpen(pin_conf.right_pin1, pin_conf.right_pin2, pin_conf.right_enale)
    kinematic = KinematicControl((motor_left, motor_right))
    kinematic.set_speed(100)
    kinematic.set_state(4)
    while 1:
        print('speed', kinematic.speed)
        print('angle', kinematic.angle)
        print('state', kinematic.state)
        print('chang', kinematic.state_change)
        print('left ', kinematic.left_speed, kinematic.left_motor.speed, kinematic.left_motor.pwm_value)
        print('right', kinematic.right_speed, kinematic.right_motor.speed, kinematic.right_motor.pwm_value)
        print('time1', kinematic.end_time, 'time2', time.time())
        print('****************************')
        kinematic.spin()


# Car will turn left, turn right, (turn back,) go ahead, go back and stop.
# When car turn left or turn right, main program will be blocked until turn over.
# When car go ahead or go back, speed parameter and angle parameter should be given,
# which are computed by deviation distance between two sides.
# Thus, a deviation node should be create to computed this and can be blocked when needed.
# And turn left or turn left can be called by maze solution node.
