#!/usr/bin/env python

import time
import math
from constant import StateEnum, DirctionEnum

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# state constant
state = StateEnum()
# dir = DirctionEnum()


class KinematicControl:
    def __init__(self):
        # self.current_dir = dir.E
        # speed config
        self.forward_speed = 0.5
        self.back_speed = -0.5
        # self.angle_speed = 1.0

        # distance config
        self.Node_X_distance = 0.30
        self.Node_Y_distance = 0.25
        # self.Node_back_distance = 0.25
        # self.Rotate_angle = math.pi / 2 - 0.5
        # self.Rotate_right_angle = math.pi/2 - 0.5
        # self.Rotate_left_angle = math.pi/2 - 0.5

        # velocity template variable
        self.x_speed = 0.5
        self.y_speed = 0
        self.z_angle = 0

        # state variable
        self.state = state.stop

        # timer and odometor
        self.start_time = 0
        self.end_time = 0
        self.start_distance = 0
        self.end_distance = 0

        # position
        self.start_pos = None
        self.start_angle = None
        self.current_pos = None
        self.current_angle = None

        # mode state
        self._block_ = False
        self._finish_ = True
        self._state_change_ = False

        # vel publish
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odomcallback, queue_size=1)

        self.f = open('./path.csv', 'w')

    def set_state(self, state_):
        """
        external call fuction, set state
        :param state_:
        :return:
        """
        if self.state != state_ and self._block_ is False:
            self.state = state_
            self._state_change_ = True

    def set_angle(self, angle=50):
        """
        external call fuction, set angle
        :param angle:
        :return:
        """
        self.z_angle = int(max(0, min(angle, 100)))

    def set_speed(self, speed=0):
        """
        external call fuction, set speed
        :param speed:
        :return:
        """
        self.x_speed = int(max(0, min(speed, 100)))

    def check_finish(self):
        """
        external call fuction, check current state has finished or not
        :return:
        """
        if self._finish_:
            self._finish_ = False
            return True
        else:
            return False

    def odomcallback(self, msg):
        """
        http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html
        :param msg:
        :return:
        """
        pose_with_covariance = msg.pose
        pose = pose_with_covariance.pose
        position = pose.position
        x = position.x
        y = position.y
        z = position.z
        self.current_pos = (x, y, 0)
        self.current_angle = z

    def pub_twist(self):
        """
        publish speed info
        :return:
        """
        twist_aux = Twist()
        twist_aux.linear.x = self.x_speed
        twist_aux.linear.y = self.y_speed
        twist_aux.angular.z = self.z_angle
        self.vel_pub.publish(twist_aux)

    # def get_current_angle(self):
    #     if self.dir == dir.E:
    #         return 0
    #     elif self.dir == dir.W:
    #         return -math.pi
    #     elif self.dir == dir.N:
    #         return math.pi / 2
    #     elif self.dir == dir.S:
    #         return -math.pi / 2

    def E_distance(self):
        if self._state_change_:
            print('state', 'E')
            self._state_change_ = False
            self.start_pos = self.current_pos  # set start position
            self._block_ = True
            self.x_speed = self.forward_speed  # run forward speed
            self.y_speed = 0
            self.z_angle = 0
        else:
            delt_x = abs(self.current_pos[0] - self.start_pos[0])
            delt_y = abs(self.current_pos[1] - self.start_pos[1])
            current_distance = math.sqrt(math.pow(delt_x, 2) + math.pow(delt_y, 2))  # calculate EUCLIDEAN distance
            if current_distance < self.Node_X_distance:  # judge distance
                pass  # continue run forward
            else:
                self.start_pos = None
                self.state = state.stop
                self._state_change_ = True
                self._block_ = False
                self._finish_ = True

    def W_distance(self):
        if self._state_change_:
            print('state', 'W')
            self._state_change_ = False
            self.start_pos = self.current_pos  # set start position
            self._block_ = True
            self.x_speed = -self.forward_speed  # run forward speed
            self.y_speed = 0
            self.z_angle = 0
        else:
            delt_x = abs(self.current_pos[0] - self.start_pos[0])
            delt_y = abs(self.current_pos[1] - self.start_pos[1])
            current_distance = math.sqrt(math.pow(delt_x, 2) + math.pow(delt_y, 2))  # calculate EUCLIDEAN distance
            if current_distance < self.Node_X_distance:  # judge distance
                pass  # continue run forward
            else:
                self.start_pos = None
                self.state = state.stop
                self._state_change_ = True
                self._block_ = False
                self._finish_ = True

    def S_distance(self):
        if self._state_change_:
            print('state', 'S')
            self._state_change_ = False
            self.start_pos = self.current_pos  # set start position
            self._block_ = True
            self.x_speed = 0  # run forward speed
            self.y_speed = -self.forward_speed
            self.z_angle = 0
        else:
            delt_x = abs(self.current_pos[0] - self.start_pos[0])
            delt_y = abs(self.current_pos[1] - self.start_pos[1])
            current_distance = math.sqrt(math.pow(delt_x, 2) + math.pow(delt_y, 2))  # calculate EUCLIDEAN distance
            if current_distance < self.Node_Y_distance:  # judge distance
                pass  # continue run forward
            else:
                self.start_pos = None
                self.state = state.stop
                self._state_change_ = True
                self._block_ = False
                self._finish_ = True

    def N_distance(self):
        if self._state_change_:
            print('state', 'N')
            self._state_change_ = False
            self.start_pos = self.current_pos  # set start position
            self._block_ = True
            self.x_speed = 0  # run forward speed
            self.y_speed = self.forward_speed
            self.z_angle = 0
        else:
            delt_x = abs(self.current_pos[0] - self.start_pos[0])
            delt_y = abs(self.current_pos[1] - self.start_pos[1])
            current_distance = math.sqrt(math.pow(delt_x, 2) + math.pow(delt_y, 2))  # calculate EUCLIDEAN distance
            if current_distance < self.Node_Y_distance:  # judge distance
                pass  # continue run forward
            else:
                self.start_pos = None
                self.state = state.stop
                self._state_change_ = True
                self._block_ = False
                self._finish_ = True

    # def forward_distance(self):
    #     if self._state_change_:
    #         print('state', 'forward')
    #         self._state_change_ = False
    #         self.start_pos = self.current_pos  # set start position
    #         self._block_ = True
    #         self.x_speed = self.forward_speed  # run forward speed
    #         self.y_speed = 0
    #         self.z_angle = 0
    #     else:
    #         delt_x = abs(self.current_pos[0] - self.start_pos[0])
    #         delt_y = abs(self.current_pos[1] - self.start_pos[1])
    #         current_distance = math.sqrt(math.pow(delt_x, 2) + math.pow(delt_y, 2))  # calculate EUCLIDEAN distance
    #         if current_distance < self.Node_distance:  # judge distance
    #             pass  # continue run forward
    #         else:
    #             # print('angle', self.current_angle)
    #             # print('pos', self.current_pos)
    #             self.start_pos = None
    #             self.state = state.stop
    #             self._state_change_ = True
    #             self._block_ = False
    #             self._finish_ = True
    #
    # def backward_distance(self):
    #     if self._state_change_:
    #         print('state', 'backward')
    #         self._state_change_ = False
    #         self.start_pos = self.current_pos  # set start position
    #         self._block_ = True
    #         self.x_speed = self.back_speed  # run backward speed
    #         self.y_speed = 0
    #         self.z_angle = 0
    #     else:
    #         delt_x = abs(self.current_pos[0] - self.start_pos[0])
    #         delt_y = abs(self.current_pos[1] - self.start_pos[1])
    #         current_distance = math.sqrt(math.pow(delt_x, 2) + math.pow(delt_y, 2))  # calculate EUCLIDEAN distance
    #         if current_distance < self.Node_back_distance:  # judge distance
    #             pass  # continue run backward
    #         else:
    #             self.start_pos = None
    #             self.state = state.stop
    #             self._state_change_ = True
    #             self._block_ = False
    #             self._finish_ = True
    #
    # def turn_left_distance(self):
    #     if self._state_change_:
    #         print('state', 'left')
    #         self._state_change_ = False
    #         self.start_angle = self.get_current_angle()  # set start angle 0,pi/2,-pi,-pi/2
    #         self._block_ = True
    #         self.x_speed = 0
    #         self.y_speed = 0
    #         self.z_angle = self.angle_speed  # w > 0  Rotate left
    #     else:
    #         if abs(self.current_angle - self.start_angle) < self.Rotate_angle:  # judge delt_angle
    #             pass  # continue Rotate in place
    #         else:
    #             self.start_angle = None
    #             self.state = state.forward_distance
    #             self._state_change_ = True
    #             self._block_ = False
    #
    # def turn_right_distance(self):
    #     if self._state_change_:
    #         print('state', 'right')
    #         self._state_change_ = False
    #         self.start_angle = self.current_angle  # set start angle
    #         self._block_ = True
    #         self.x_speed = 0
    #         self.y_speed = 0
    #         self.z_angle = -self.angle_speed  # w < 0  Rotate right
    #     else:
    #         if abs(self.current_angle - self.start_angle) < self.Rotate_angle:  # judge delt_angle
    #             pass  # continue Rotate in place
    #         else:
    #             self.start_angle = None
    #             self.state = state.forward_distance
    #             self._state_change_ = True
    #             self._block_ = False

    def stop(self):
        if self._state_change_:
            print('state', 'stop')
            self._state_change_ = False
            self.x_speed = 0
            self.y_speed = 0
            self.z_angle = 0
        else:
            self.x_speed = 0
            self.y_speed = 0
            self.z_angle = 0

    def spin(self):
        if self.state == state.stop:
            self.stop()
        # elif self.state == state.forward_distance:
        #     self.forward_distance()
        # elif self.state == state.backward_distance:
        #     self.backward_distance()
        # elif self.state == state.turn_right_distance:
        #     self.turn_right_distance()
        # elif self.state == state.turn_left_distance:
        #     self.turn_left_distance()
        elif self.state == state.E_distance:
            self.E_distance()
        elif self.state == state.W_distance:
            self.W_distance()
        elif self.state == state.S_distance:
            self.S_distance()
        elif self.state == state.N_distance:
            self.N_distance()
        self.pub_twist()
        print('ang', self.current_angle)
        print('pos', self.current_pos)
        self.f.write(str(self.current_pos[0]) + ',' + str(self.current_pos[1]) + '\n')

    def __del__(self):
        self.f.close()


def show(ki):
    print('state', ki.state)


# for testd
if __name__ == '__main__':
    # rospy.init_node('servo_node', anonymous=True)
    # rate = rospy.Rate(20)
    # try:
    #     ki = KinematicControl()
    #     time.sleep(1)
    #     while not rospy.is_shutdown():
    #         try:
    #             ord = input()
    #             if ord == 'E':
    #                 ki.set_state(state.E_distance)
    #             elif ord == 'W':
    #                 ki.set_state(state.W_distance)
    #             elif ord == 'S':
    #                 ki.set_state(state.S_distance)
    #             elif ord == 'N':
    #                 ki.set_state(state.N_distance)
    #             ki._finish_ = False
    #             while not ki.check_finish():
    #                 ki.spin()
    #         except Exception as e:
    #             print(e)
    #         rate.sleep()
    # except rospy.ROSInterruptException:
    #     print(rospy.ROSInterruptException)
    pass

