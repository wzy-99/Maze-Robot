#!/usr/bin/env python

import traceback
from peripheral import Encoder, Motor
from kinematic import KinematicControl
from constant import DirctionEnum

import rospy
from car.msg import Speed
from std_msgs.msg import Int32


class ServoSystem:
    def __init__(self):
        # position direction infomation
        dir_enum = DirctionEnum()
        self.forward = dir_enum.dir_forward
        self.left = dir_enum.dir_left
        self.right = dir_enum.dir_right
        self.back = dir_enum.dir_back
        del dir_enum

        # peripheral instance
        self.left_encode = Encoder()
        self.right_encode = Encoder()
        self.left_motor = Motor()
        self.right_motor = Motor()

        # loacl variable
        self.left_speed = 0
        self.right_speed = 0
        self.left_dir = self.forward
        self.right_dir = self.forward
        self.dir_change = False
        self.speed_change = False

        self.pub_ngrid = rospy.Publish("/new_grid", Int32, queue_size=1)
        self.sub_speed = rospy.Subscriber("/auto_speed", Speed, self.speedcallback, queue_size=10)

    def speedcallback(self, msg):
        if self.left_dir != msg.left_dir or self.right_dir != msg.right_dir:
            self.dir_change = True
            self.left_dir = msg.left_dir
            self.right_dir = msg.right_dir
        if self.left_speed != msg.left_speed or self.right_speed != msg.right_speed:
            self.speed_change = True
            self.left_speed = msg.left_speed
            self.right_speed = msg.right_speed

    def spin(self):
        if self.dir_change:
            self.left_motor.set_direction(self.left_dir)
            self.right_motor.set_direction(self.right_motor)
            self.dir_change = False
        if self.speed_change:
            self.left_motor.set_speed(self.left_speed)
            self.right_motor.set_speed(self.right_speed)
            self.speed_change = False
        if self.left_encode.new_grid():
            self.pub_ngrid.publish(1)


if __name__ == '__main__':
    rospy.init_node('servo_node', anonymous=True)
    rate = rospy.Rate(20)
    try:
        servo_system = ServoSystem()
        while not rospy.is_shutdown():
            try:
                servo_system.spin()
            except Exception as e:
                print(e)
                traceback.print_exc()
            rate.sleep()
    except rospy.ROSInterruptException:
        print(rospy.ROSInterruptException)
