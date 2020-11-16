#!/usr/bin/env python

import time
import math
from constant import StateEnum

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# state constant
state = StateEnum()


class KinematicControl:
    def __init__(self):
        # speed config
        self.forward_speed = 0.5
        self.back_speed = -0.5
        self.angle_speed = 1

        # distance config
        self.Node_distance = 200  # 一个方格的距离(前进或者后退的距离)
        self.Rotate_angle = 90  # 旋转角度 = 90

        # velocity template variable
        self.x_speed = 100
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
        twist_with_covariance = msg.twist
        pose = pose_with_covariance.pos
        twist = twist_with_covariance.twist
        angular = twist.angular
        position = pose.position
        x = position.x.data
        y = position.y.data
        z = angular.z.data
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

    def forward_distance(self):
        if self._state_change_:
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
            if current_distance < self.Node_distance:  # judge distance
                pass  # continue run forward
            else:
                self.start_pos = None
                self.state = state.stop
                self._state_change_ = True
                self._block_ = False
                self._finish_ = True

    def backward_distance(self):
        if self._state_change_:
            self._state_change_ = False
            self.start_pos = self.current_pos  # set start position
            self._block_ = True
            self.x_speed = self.back_speed  # run backward speed
            self.y_speed = 0
            self.z_angle = 0
        else:
            delt_x = abs(self.current_pos[0] - self.start_pos[0])
            delt_y = abs(self.current_pos[1] - self.start_pos[1])
            current_distance = math.sqrt(math.pow(delt_x, 2) + math.pow(delt_y, 2))  # calculate EUCLIDEAN distance
            if current_distance < self.Node_distance:  # judge distance
                pass  # continue run backward
            else:
                self.start_pos = None
                self.state = state.stop
                self._state_change_ = True
                self._block_ = False
                self._finish_ = True

    def turn_left_distance(self):
        if self._state_change_:
            self._state_change_ = False
            self.start_angle = self.current_angle  # set start angle
            self._block_ = True
            self.x_speed = 0
            self.y_speed = 0
            self.z_angle = self.angle_speed  # w > 0  Rotate left
        else:
            if abs(self.current_angle - self.start_angle) < self.Rotate_angle:  # judge delt_angle
                pass  # continue Rotate in place
            else:
                self.start_angle = None
                self.state = state.forward_distance
                self._state_change_ = True
                self._block_ = False

    def turn_right_distance(self):
        if self._state_change_:
            self._state_change_ = False
            self.start_angle = self.current_angle  # set start angle
            self._block_ = True
            self.x_speed = 0
            self.y_speed = 0
            self.z_angle = -self.angle_speed  # w < 0  Rotate right
        else:
            if abs(self.current_angle - self.start_angle) < self.Rotate_angle:  # judge delt_angle
                pass  # continue Rotate in place
            else:
                self.start_angle = None
                self.state = state.forward_distance
                self._state_change_ = True
                self._block_ = False

    def stop_(self):
        if self._state_change_:
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
            self.stop_()
        elif self.state == state.forward_distance:
            self.forward_distance()
        elif self.state == state.backward_distance:
            self.backward_distance()
        elif self.state == state.turn_right_distance:
            self.turn_right_distance()
        elif self.state == state.turn_left_distance:
            self.turn_left_distance()
        self.pub_twist()


# def show(ki):
#     print('speed', ki.speed)
#     print('angle', ki.angle)
#     print('state', ki.state)
#     print('chang', ki.state_change)
#     print('time1', ki.end_time, 'time2', time.time())


# for test
if __name__ == '__main__':
    pass
