# !/usr/bin/env python


class Command:
    def __init__(self):
        self.command_speed = ord('s')
        self.command_dirtion = ord('d')
        self.command_left_speed = ord('l')
        self.command_right_speed = ord('r')
        self.command_stop = ord('s')
        self.command_run = ord('r')


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


class DirctionEnum:
    def __init__(self):
        self.dir_forward = 1
        self.dir_left = 0
        self.dir_right = 2
        self.dir_back = -1
        self.dir_up = 3
        self.dir_down = 4
        self.dir_none = 5


class EncodeInfo:
    def __init__(self):
        self.count_per_cm = 50
        self.cm_per_grid = 50


class Pin:
    def __init__(self):

        # # physical pins
        # self.IN1_ = 36
        # self.IN2_ = 35
        # self.IN3_ = 39
        # self.IN4_ = 38
        # self.ENA_ = 34
        # self.ENB_ = 37

        # bcm pins
        self.IN1 = 19
        self.IN2 = 16
        self.IN3 = 21
        self.IN4 = 26
        self.ENA = 13
        self.ENB = 20

        # motor pins
        self.left_pin1 = self.IN1
        self.left_pin2 = self.IN2
        self.left_enale = self.ENA
        self.right_pin1 = self.IN3
        self.right_pin2 = self.IN4
        self.right_enale = self.ENB
