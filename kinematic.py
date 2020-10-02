# !/usr/bin/env python

from peripheral import Motor, Encoder
from constant import DirctionEnum


# In this class, kinematic control method is defined, such as turn back, turn right and so on.
# And you should know that belowed methond is high-level control one rather than basic control methond.
class KinematicControl:
    def __init__(self, motor_instance):

        dir_enum = DirctionEnum()
        self.forward = dir_enum.dir_forward
        self.left = dir_enum.dir_left
        self.right = dir_enum.dir_right
        self.back = dir_enum.dir_back
        del dir_enum

        # config variable
        self.turn_speed = 10

        # motor instance
        self.left_motor, self.right_motor = motor_instance

        # local variable
        self.left_speed = 0
        self.right_speed = 0
        # positin direction which contains 4 kinds of direction
        # notice this is not drive angle whice is from 0 to 100
        self.direction = self.forward

    def turn_right(self):
        self.left_motor.set_forword()
        self.right_motor.set_back()
        pass

    def turn_left(self):
        self.left_motor.set_forword()
        self.right_motor.set_back()
        pass

    def turn_back(self):
        pass

    def turn(self, dirction):
        if self.direction == dirction:
            return
        else:
            if dirction == self.right:
                self.turn_right()
            elif dirction == self.left:
                self.turn_left()
            else:
                pass

    def go_ahead(self, speed, diretion):
        self.left_motor.set_forword()
        self.right_motor.set_forword()
        self.dif_speed(speed, diretion)
        self.left_motor.set_speed(self.left_speed)
        self.right_motor.set_speed(self.right_speed)

    def go_back(self, speed, diretion):
        self.left_motor.set_back()
        self.right_motor.set_back()
        self.dif_speed(speed, diretion)
        self.left_motor.set_speed(self.left_speed)
        self.right_motor.set_speed(self.right_speed)

    def dif_speed(self, speed, angle):
        # TODO dif speed algorithm
        self.left_speed = speed * angle / 100
        self.right_speed = speed * angle / 100

