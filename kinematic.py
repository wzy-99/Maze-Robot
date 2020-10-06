# !/usr/bin/env python

import time
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
        del state

        # config variable
        self.turn_speed = 100
        self.turn_dtime = 10
        self.adjust_k = 0.05
        self.angle_pd = PD(kd=-1.0, kp=-1.0, target=50.0)

        # motor instance
        self.left_motor, self.right_motor = motor_instance

        # local variable
        self.angle = 50
        self.speed = 0
        self.left_speed = 0
        self.right_speed = 0
        self.state = self.run_forward
        self.state_change = False
        self.time = None
        self.end_time = None
        self.block = False

    def set_state(self, state):
        if self.state != state and self.block == False:
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
            self.time = time.time()
            self.end_time = self.time + self.turn_dtime
            self.block = True
        else:
            if self.time < self.end_time:
                pass
            else:
                self.time = 0
                self.end_time = 0
                self.state = self.run_forward
                self.block = False

    def go_left(self):
        if self.state_change:
            self.left_motor.set_back()
            self.right_motor.set_forword()
            self.state_change = False
            self.left_motor.set_speed(self.turn_speed//2)
            self.right_motor.set_speed(self.turn_speed)
            self.time = time.time()
            self.end_time = self.time + self.turn_dtime
            self.block = True
        else:
            if self.time < self.end_time:
                pass
            else:
                self.time = 0
                self.end_time = 0
                self.state = self.run_forward
                self.block = False

    def go_back(self):
        if self.state_change:
            self.left_motor.set_forword()
            self.right_motor.set_back()
            self.state_change = False
            self.left_motor.set_speed(self.turn_speed)
            self.right_motor.set_back(self.turn_speed//2)
            self.time = time.time()
            self.end_time = self.time + self.turn_dtime * 2
            self.block = True
        else:
            if self.time < self.end_time:
                pass
            else:
                self.time = None
                self.end_time = None
                self.state = self.run_forward
                self.block = False

    def go_forward(self):
        if self.state_change:
            self.left_motor.set_forword()
            self.right_motor.set_forword()
            self.state_change = False
        else:
            self.dif_speed(self.speed, self.angle)
            self.left_motor.set_speed(self.left_speed)
            self.right_motor.set_speed(self.right_speed)

    def stop(self):
        if self.state_change:
            self.left_speed = 0
            self.right_speed = 0
            self.angle = 50
            self.state_change = False
        else:
            self.left_motor.set_speed(0)
            self.right_motor.set_speed(0)

    def adjust_angle(self, left_distance, right_distance):
        d_distance = left_distance - right_distance
        self.angle = self.angle_pd.update(d_distance * self.adjust_k)

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
        elif self.state == self.stop:
            self.stop()


# Car will turn left, turn right, (turn back,) go ahead, go back and stop.
# When car turn left or turn right, main program will be blocked until turn over.
# When car go ahead or go back, speed parameter and angle parameter should be given,
# which are computed by deviation distance between two sides.
# Thus, a deviation node should be create to computed this and can be blocked when needed.
# And turn left or turn left can be called by maze solution node.
