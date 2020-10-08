import traceback
import numpy as np
from constant import MazeInfo, DirctionEnum, StateEnum
import random

import rospy
from std_msgs.msg import Int32


state = StateEnum()

class Node:
    def __init__(self, x, y):
        self.x = x  # 方块位置:行
        self.y = y  # 方块位置:列
        self.road = {0: 0,  # 键：方向 NESW   键值:通行状态
                     1: 0,
                     2: 0,
                     3: 0}  # 通行状态说明: 0   不可通行
        #              1  可通行但还未被访问过
        #             -1  可通行但已经被访问过

        self.valid = {}  # 有效(可通行)的方向
        self.visited = False  # 方块是否被访问过，若已经被访问过，则该方块东南西北的通行情况已经被确定，不再更新
        self.is_valid = False  # 方块的有效可通方向，若该块被弹出，其状态可归零
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
        self.cur_direction = self.E  # 初始时设它朝东
        self.next_direction = self.E
        self.cur_grid = 0
        self.is_new_grid = False

        self.state = -2

        self.path = [self.maze[0][0]]
        self.info = []
        self.exit = [self.maze[3][3], self.maze[4][3]]
        self.over = False

        self.pub_turn = rospy.Publish("/turn", Int32, queue_size=1)
        rospy.Subscriber("/grid", Int32, self.gridcallback, queue_size=3)
        rospy.Subscriber("/detect", Int32, self.accept_info, queue_size=3)

        # [forward,back,left,right]  偏移量设定，方向转换时用到
        self.Xchange = {0: 0,
                        1: 2,
                        2: 3,
                        3: 1}

    def create_map(self, size):
        """
        create a new maze map
        :param size: the size of map
        """
        # TODO: Create a maze whose inner grids are unknown and around grids are wall.
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

        # 为了方便debug,此处信息是由人手输入的，要换成从传感器接收数据，形式:[0,1,0,1],分别表示 [前,后,左,右] 有无障碍物，0：有障碍，1：无障碍
        # 如：[0,1,0,1]表示相对于车来说，其 前方有障碍物，后方没有，左方有障碍，右方没有障碍

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
        # TODO: Update the maze when accepting the infomation from the transducer.
        self.maze_map[self.cur_loction.x][self.cur_loction.y] = 1
        if not self.maze[self.cur_loction.x][self.cur_loction.y].visited:  # 节点还未访问过
            # NESW 方向可通行状况确定，一旦确定，不再更新
            for i in range(4):
                offset = self.Xchange[i]  # 方向(地址)偏移量
                real_direction = (self.cur_direction + offset) % 4  # 真正的方向(地址) = （当前方向 + 偏移量）% 系统的模
                self.maze[self.cur_loction.x][self.cur_loction.y].road[real_direction] = self.info[i]  # NESW 方向可通行状况确定
            self.maze[self.cur_loction.x][self.cur_loction.y].visited = True  # 标记节点已经访问过，以后不再更新，
            # 一个节点的 NESW 方向的通行状况是恒定不变的的，不必每次更新
            # 节点有效方向确定
        if not self.maze[self.cur_loction.x][self.cur_loction.y].is_valid:  # 该块状态未更新
            print('not in path')
            back_direction = (self.cur_direction + 2) % 4  # 转换为与前向相背离的方向
            self.maze[self.cur_loction.x][self.cur_loction.y].road[
                back_direction] = -1  # 可能回溯的方向，不能再往这个方向走，即可选的有效方向中不包括回溯方向
            for k, v in self.maze[self.cur_loction.x][self.cur_loction.y].road.items():
                if v == -1 or v == 0:
                    continue
                else:
                    self.maze[self.cur_loction.x][self.cur_loction.y].valid.update({k: v})  # 确定有效的可行方向

            self.maze[self.cur_loction.x][self.cur_loction.y].is_valid = True  # 该块状态更新!!!!

    def get_next_direction(self):
        """
        get_next_direction
        :param info: [back,forward,left,right]
        :return:
        """
        self.B = False  # 新一轮一定要重置  回退状态！！！！！！！！BUG
        if len(self.maze[self.cur_loction.x][self.cur_loction.y].valid) > 0:  # 如果有方向可走
            # print('go!!!')
            # print('当前节点:', self.cur_loction)
            # print('当前有效通行状况1:', self.maze[self.cur_loction.x][self.cur_loction.y].valid)
            key = random.choice(list(self.maze[self.cur_loction.x][self.cur_loction.y].valid))  # 随机选一个可走的方向，走
            self.next_direction = key  # 方向更新
            # print('下个方向:', self.next_direction)
            self.maze[self.cur_loction.x][self.cur_loction.y].valid.pop(key)  # 已访问过，从可走的路中剔除
            # print('当前有效通行状况2:', self.maze[self.cur_loction.x][self.cur_loction.y].valid)


        else:
            print('back!!!')  # 无前路可走
            self.B = True  # 回退标记

    def walk(self):  # 行走函数，仅涉及位置的移动，未写出车辆具体行走动作，如转弯，前行
        if self.B:
            self.path.pop()
            self.maze[self.cur_loction.x][self.cur_loction.y].is_valid = False  # 状态回归零
            self.maze[self.cur_loction.x][self.cur_loction.y].inPath = False  # 记录出栈
            # print("path:",self.path)
            # for i in self.maze:
            # print(i)
            self.cur_loction = self.path[-1]  # 回溯
        else:
            if self.next_direction == self.N:
                self.next_loction = self.maze[self.cur_loction.x - 1][self.cur_loction.y]
            elif self.next_direction == self.E:
                self.next_loction = self.maze[self.cur_loction.x][self.cur_loction.y + 1]
            elif self.next_direction == self.S:
                self.next_loction = self.maze[self.cur_loction.x + 1][self.cur_loction.y]
            elif self.next_direction == self.W:
                self.next_loction = self.maze[self.cur_loction.x][self.cur_loction.y - 1]

            self.cur_loction = self.next_loction  # 位置更新
        self.cur_direction = self.next_direction  # 方向更新(回溯，方向不变；前进，方向改变)
        print('last 方向:', self.cur_direction, 'next 方向:', self.next_direction)
        self.turn()

        if not self.maze[self.cur_loction.x][self.cur_loction.y].inPath:
            self.maze[self.cur_loction.x][self.cur_loction.y].inPath = True
            self.path.append(self.maze[self.cur_loction.x][self.cur_loction.y])  # 记录路径

    def send(self):
        self.pub_turn.publish(self.state)

    def show_map(self):
        for i in range(self.maze_map.shape[0]):
            for j in range(self.maze_map.shape[1]):
                if i == self.cur_loction.x and j == self.cur_loction.y:
                    self.draw()
                else:
                    print(self.maze_map[i][j], end='     ')
            print('\n')
        print('-----------------------------------------\n')
        print('当前方向:', self.cur_direction)
        print('当前位置:', self.cur_loction.x, self.cur_loction.y)
        print('当前通行状况:', self.maze[self.cur_loction.x][self.cur_loction.y].road)
        print('当前有效通行状况3:', self.maze[self.cur_loction.x][self.cur_loction.y].valid)

    def draw(self):
        if self.cur_direction == self.N:
            print('\033[1;35m^\033[0m', end='     ')
        elif self.cur_direction == self.E:
            print('\033[1;35m>\033[0m', end='     ')
        elif self.cur_direction == self.S:
            print('\033[1;35mV\033[0m', end='     ')
        elif self.cur_direction == self.W:
            print('\033[1;35m<\033[0m', end='     ')

    def is_out(self):  # 判断是否走出迷宫
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
            # self.show_map()
            # self.is_out()
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
                print('左转！！')
                self.state = state.turn_left  # 左
            else:
                print('右转！！')
                self.state = state.turn_right  # 右
        elif abs(delt) == 3:
            if delt > 0:
                print('右转！！')
                self.state = state.turn_right  # 右
            else:
                print('左转！！')
                self.state = state.turn_left  # 左
        elif abs(delt) == 2:
            print('掉头！！')
            self.state = state.turn_back  # 掉头
        elif abs(delt) == 0:
            if self.B:
                print('回退！！')
                self.state = state.run_back  # 回退
            else:
                print('直行！！')
                self.state = state.run_forward  # 直行


if __name__ == '__main__':
    maze = MazeSolution()
    maze.show_map()
    while maze.over is False:
        maze.solve()
    print('迷宫寻路结束，路径为:')
    #maze.show_path()
