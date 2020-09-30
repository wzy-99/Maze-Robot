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


class EncodeInfo:
    def __init__(self):
        self.count_per_cm = 50
        self.cm_per_grid = 50
