#!/usr/bin/env python

import traceback
import numpy as np
from constant import MazeInfo, DirctionEnum, StateEnum
import random

import rospy
from std_msgs.msg import Int32

state = StateEnum()


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.road = {0: 0,
                     1: 0,
                     2: 0,
                     3: 0}

        self.valid = {}
        self.visited = False
        self.is_valid = False
        self.inPath = False

    def show(self):
        print("road", self.road)


class MazeSolution:
    def __init__(self):
        info = MazeInfo()
        self.maze_wall = info.maze_wall
        self.maze_road = info.maze_road
        self.maze_pass = info.maze_pass
        self.maze_unknown = info.maze_unknown
        del info

        enum = DirctionEnum()
        self.N = enum.N
        self.S = enum.S
        self.E = enum.E
        self.W = enum.W

        self.B = False

        del enum

        # loacl variale
        self.maze_map = None
        self.maze = []
        self.create_map([self.maze_unknown, self.maze_unknown])
        self.cur_loction = self.maze[0][0]
        self.next_loction = self.maze[0][0]
        self.cur_direction = self.E
        self.next_direction = self.E
        self.cur_grid = 0
        self.is_new_grid = False
        self.change = False

        self.state = -2

        self.path = [self.maze[0][0]]
        self.info = []
        self.exit = [self.maze[3][3], self.maze[4][2]]
        self.over = False

        self.pub_turn = rospy.Publisher("/state", Int32, queue_size=1)
        rospy.Subscriber("/grid", Int32, self.gridcallback, queue_size=3)
        rospy.Subscriber("/detect", Int32, self.accept_info, queue_size=3)

        self.Xchange = {0: 0,
                        1: 2,
                        2: 3,
                        3: 1}

    def create_map(self, size):
        """
        create a new maze map
        :param size: the size of map
        """
        self.maze_map = np.zeros(shape=size, dtype=np.int32)
        for i in range(5):
            m = []
            for j in range(5):
                m.append(Node(i, j))
            self.maze.append(m)

    def gridcallback(self, msg):
        if self.cur_grid == msg.data:
            pass
        else:
            self.cur_grid = msg.data
            self.change = True

    def accept_info(self, msg):
        self.change = True
        data = msg.data
        font_detect = int(data / 1000)
        back_detect = int((data - 1000 * font_detect) / 100)
        left_detect = int((data - 1000 * font_detect - 100 * back_detect) / 10)
        right_detect = int(data - 1000 * font_detect - 100 * back_detect - 10 * left_detect)
        self.info = [font_detect, back_detect, left_detect, right_detect]

    def update_map(self):
        """
        update the maze map.
        :param :self
        :return:None
        """
        self.maze_map[self.cur_loction.x][self.cur_loction.y] = 1
        if not self.maze[self.cur_loction.x][self.cur_loction.y].visited:
            for i in range(4):
                offset = self.Xchange[i]
                real_direction = (self.cur_direction + offset) % 4
                self.maze[self.cur_loction.x][self.cur_loction.y].road[real_direction] = self.info[i]
            self.maze[self.cur_loction.x][self.cur_loction.y].visited = True
        if not self.maze[self.cur_loction.x][self.cur_loction.y].is_valid:
            print('not in path')
            back_direction = (self.cur_direction + 2) % 4
            self.maze[self.cur_loction.x][self.cur_loction.y].road[
                back_direction] = -1
            for k, v in self.maze[self.cur_loction.x][self.cur_loction.y].road.items():
                if v == -1 or v == 0:
                    continue
                else:
                    self.maze[self.cur_loction.x][self.cur_loction.y].valid.update({k: v})

            self.maze[self.cur_loction.x][self.cur_loction.y].is_valid = True

    def get_next_direction(self):
        """
        get_next_direction
        :param info: [back,forward,left,right]
        :return:
        """
        self.B = False  # very important by zst
        if len(self.maze[self.cur_loction.x][self.cur_loction.y].valid) > 0:
            key = random.choice(list(self.maze[self.cur_loction.x][self.cur_loction.y].valid))
            self.next_direction = key
            self.maze[self.cur_loction.x][self.cur_loction.y].valid.pop(key)

        else:
            print('back!!!')
            self.B = True

    def walk(self):
        if self.B:
            self.path.pop()
            self.maze[self.cur_loction.x][self.cur_loction.y].is_valid = False
            self.maze[self.cur_loction.x][self.cur_loction.y].inPath = False
            self.cur_loction = self.path[-1]
        else:
            if self.next_direction == self.N:
                self.next_loction = self.maze[self.cur_loction.x - 1][self.cur_loction.y]
            elif self.next_direction == self.E:
                self.next_loction = self.maze[self.cur_loction.x][self.cur_loction.y + 1]
            elif self.next_direction == self.S:
                self.next_loction = self.maze[self.cur_loction.x + 1][self.cur_loction.y]
            elif self.next_direction == self.W:
                self.next_loction = self.maze[self.cur_loction.x][self.cur_loction.y - 1]
            print('walk_next_next_direction', self.next_direction)
            self.cur_loction = self.next_loction
        self.turn()
        self.cur_direction = self.next_direction

        if not self.maze[self.cur_loction.x][self.cur_loction.y].inPath:
            self.maze[self.cur_loction.x][self.cur_loction.y].inPath = True
            self.path.append(self.maze[self.cur_loction.x][self.cur_loction.y])

    def send(self):
        self.pub_turn.publish(self.state)

    def show_map(self):
        #     for i in range(self.maze_map.shape[0]):
        #         for j in range(self.maze_map.shape[1]):
        #             if i == self.cur_loction.x and j == self.cur_loction.y:
        #                 self.draw()
        #             else:
        #                 print(self.maze_map[i][j], end='     ')
        #         print('\n')
        print('-----------------------------------------\n')
        print('cur dir:', self.cur_direction)
        print('cur pos:', self.cur_loction.x, self.cur_loction.y)
        print('cur road:', self.maze[self.cur_loction.x][self.cur_loction.y].road)
        print('cur valid:', self.maze[self.cur_loction.x][self.cur_loction.y].valid)

    # def draw(self):
    #     if self.cur_direction == self.N:
    #         print('\033[1;35m^\033[0m', end='     ')
    #     elif self.cur_direction == self.E:
    #         print('\033[1;35m>\033[0m', end='     ')
    #     elif self.cur_direction == self.S:
    #         print('\033[1;35mV\033[0m', end='     ')
    #     elif self.cur_direction == self.W:
    #         print('\033[1;35m<\033[0m', end='     ')

    def is_out(self):
        for loc in self.exit:
            if self.cur_loction.x == loc.x and self.cur_loction.y == loc.y:
                self.over = True
                break

    def solve(self):
        if self.change:
            self.update_map()
            self.get_next_direction()
            self.walk()
            self.send()
            self.show_map()
            self.is_out()
            self.change = False
        else:
            pass

    def show_path(self):
        for i in self.path:
            print("X:", i.x, "Y:", i.y)

    def turn(self):
        delt = self.cur_direction - self.next_direction
        if abs(delt) == 1:
            if delt > 0:
                print('left')
                self.state = state.turn_left_time
            else:
                print('right')
                self.state = state.turn_right_time
        elif abs(delt) == 3:
            if delt > 0:
                print('right')
                self.state = state.turn_right_time
            else:
                print('left')
                self.state = state.turn_left_time
        elif abs(delt) == 2:
            print('back')
            # self.state = state.turn_back
        elif abs(delt) == 0:
            if self.B:
                print('back')
                self.state = state.run_back_time
            else:
                print('run')
                self.state = state.run_forward_time

# if __name__ == '__main__':
#     maze = MazeSolution()
#     # maze.show_map()
#     while maze.over is False:
#         maze.solve()
#     # maze.show_path()
