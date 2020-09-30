# !/usr/bin/env python

from peripheral import Motor, Encoder
from constant import DirctionEnum


class KinematicControl:
    def __init__(self):

        dir_enum = DirctionEnum()
        self.forward = dir_enum.dir_forward
        self.left = dir_enum.dir_left
        self.right = dir_enum.dir_right
        self.back = dir_enum.dir_back
        del dir_enum

        self.direction = self.forward

        self.left_motor = Motor()
        self.right_motor = Motor()

        self.turn_speed = 10

        # temp
        self.left_speed = 0
        self.right_speed = 0

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

    def dif_speed(self, speed, diretion):
        pass

    def encode_run(self):

