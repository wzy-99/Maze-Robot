# !/usr/bin/env python

import time
from peripheral import Motor, Encoder
from constant import DirctionEnum


# In this class, kinematic control method is defined, such as turn back, turn right and so on.
# And you should know that belowed methond is high-level control one rather than basic control methond.
class KinematicControl:
    def __init__(self, motor_instance):

        dir_enum = DirctionEnum()
        self.run_forward = dir_enum.dir_forward
        self.run_back = dir_enum.dir_back
        self.stop = dir_enum.dir_none
        self.turn_left = dir_enum.dir_left
        self.turn_right = dir_enum.dir_right
        del dir_enum

        # config variable
        self.turn_speed = 100
        self.turn_dtime = 10

        # motor instance
        self.left_motor, self.right_motor = motor_instance

        # local variable
        self.angle = 50
        self.speed = 0
        self.left_speed = 0
        self.right_speed = 0
        self.state = self.run_forward
        # self.last_state = self.run_forward
        self.state_change = False
        self.time = None
        self.end_time = None

    def set_state(self, state):
        if self.state != state:
            self.state = state
            self.state_change = True

    def set_angle(self, angle=50):
        self.angle = int(max(0, min(angle, 50)))

    def set_speed(self, speed=0):
        self.speed = int(max(0, min(speed, 100)))

    def go_right(self):
        if self.state_change:
            self.left_motor.set_forword()
            self.right_motor.set_back()
            self.state_change = True
            self.left_motor.set_speed(self.turn_speed)
            self.right_motor.set_speed(self.turn_speed//2)

    def go_left(self):
        if self.state_change:
            self.left_motor.set_back()
            self.right_motor.set_forword()
            self.state_change = False
            self.left_motor.set_speed(self.turn_speed//2)
            self.right_motor.set_speed(self.turn_speed)
            self.time = time.time()
            self.end_time = self.time + self.turn_dtime
        else:
            if self.time < self.end_time:
                pass
            else:
                self.time = 0
                self.end_time = 0
                self.state = self.run_forward

    def go_back(self):
        if self.state_change:
            self.left_motor.set_forword()
            self.right_motor.set_back()
            self.state_change = False
            self.left_motor.set_speed(self.turn_speed)
            self.right_motor.set_back(self.turn_speed//2)
            self.time = time.time()
            self.end_time = self.time + self.turn_dtime * 2
        else:
            if self.time < self.end_time:
                pass
            else:
                self.time = None
                self.end_time = None
                self.state = self.run_forward

    def go_forward(self):
        if self.state_change:
            self.left_motor.set_forword()
            self.right_motor.set_forword()
            self.state_change = False
        else:
            self.dif_speed(self.speed, self.angle)
            self.left_motor.set_speed(self.left_speed)
            self.right_motor.set_speed(self.right_speed)

    def dif_speed(self, speed, angle):
        # TODO dif speed algorithm
        self.left_speed = speed * angle / 100
        self.right_speed = speed * angle / 100

    def spin(self):
        if self.state == self.run_forward:
            self.go_forward()
        elif self.state == self.turn_left:
            self.go_left()
        elif self.state == self.turn_right:
            self.go_right()
        elif self.state == self.run_back:
            self.go_back()


# Car will turn left, turn right, (turn back,) go ahead, go back and stop.
# When car turn left or turn right, main program will be blocked until turn over.
# When car go ahead or go back, speed parameter and angle parameter should be given,
# which are computed by deviation distance between two sides.
# Thus, a deviation node should be create to computed this and can be blocked when needed.
# And turn left or turn left can be called by maze solution node.
#
