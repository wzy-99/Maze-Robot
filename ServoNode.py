#!/usr/bin/env python

import time
import traceback

from peripheral import Encoder, MotorOpen, InfraRed
from kinematic import KinematicControl
from constant import DirctionEnum

import rospy
from car.msg import Speed
from car.msg import Detection
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
        self.left_motor = MotorOpen()
        self.right_motor = MotorOpen()
        self.left_infrared = InfraRed()
        self.right_infrared = InfraRed()

        # control instance
        self.kinematic = KinematicControl((self.left_motor, self.right_motor))

        # config data
        self.turn_dtime = 10
        self.turn_speed = 100

        # loacl variable
        self.turn = self.forward
        self.time = time.time()

        self.pub_grid = rospy.Publish("/grid", Int32, queue_size=1)
        self.pub_detect = rospy.Publish("/detect", Int32, queue_size=1)
        rospy.Subscriber("/turn", Int32, self.turncallback, queue_size=1)
        rospy.Subscriber("/speed", Int32, self.speedcallback, queue_size=10)
        rospy.Subscriber("/angle", Int32, self.anglecallback, queue_size=10)

    def turncallback(self, msg):
        self.kinematic.set_state(msg.data)

    def speedcallback(self, msg):
        self.kinematic.set_speed(msg.data)

    def anglecallback(self, msg):
        self.kinematic.set_angle(msg.data)

    def infrared_spin(self):
        left_detect = self.left_infrared.check_obstacle()
        right_detect = self.right_infrared.check_obstacle()
        result = left_detect * 10 + right_detect
        self.pub_detect.publish(result)

    def spin(self):
        self.kinematic.spin()
        self.infrared_spin()


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
