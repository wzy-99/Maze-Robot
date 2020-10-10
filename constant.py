#!/usr/bin/env python


class Command:
    def __init__(self):
        self.head = b'@'
        self.speed = b's'
        self.angle = b'd'
        self.state = b't'
        self.arm = b'a'
        self.kill = b'k'


class MazeInfo:
    def __init__(self):
        self.map_size = None
        self.maze_wall = -1
        self.maze_road = 0
        self.maze_pass = 1
        self.maze_unknown = 5


class RunMode:
    def __init__(self):
        self.mode_maze = 1
        self.mode_treasure = 2


class MotionInfo:
    def __init__(self):
        self.turn_speed = 10


class StateEnum:
    def __init__(self):
        self.stop = 0
        self.run_forward_time = 1
        self.run_back_time = 2
        self.turn_left_time = 3
        self.turn_right_time = 4
        # self.turn_left = 5
        # self.turn_right = 6
        # self.turn_back = 7
        self.run_forward_unblock = 8
        self.run_back_unblock = 9
        self.turn_left_unblock = 10
        self.turn_right_unblock = 11


class DirctionEnum:
    def __init__(self):
        self.dir_forward = 1
        self.dir_left = 0
        self.dir_right = 2
        self.dir_back = -1
        self.dir_up = 3
        self.dir_down = 4
        self.dir_none = 5

        self.dir_b = 0
        self.dir_f = 1
        self.dir_l = 2
        self.dir_r = 3

        self.N = 0
        self.E = 1
        self.S = 2
        self.W = 3


class EncodeInfo:
    def __init__(self):
        self.count_per_cm = 50
        self.cm_per_grid = 50


class Pin:
    def __init__(self):

        # bcm pins
        self.IN1 = 19
        self.IN2 = 16
        self.IN3 = 21
        self.IN4 = 26
        self.ENA = 13
        self.ENB = 20

        # motor pins
        self.left_pin1 = self.IN4
        self.left_pin2 = self.IN3
        self.left_enale = self.ENA
        self.right_pin1 = self.IN1
        self.right_pin2 = self.IN2
        self.right_enale = self.ENB

        self.IR_L = 27
        self.IR_R = 18

        # infrad pin
        self.left_infra_pin = self.IR_L
        self.right_infra_pin = self.IR_R

        self.left_radar_echo = 4
        self.left_radar_trig = 17
