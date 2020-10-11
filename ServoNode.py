#!/usr/bin/env python

import time
import traceback

from peripheral import Encoder, MotorOpen, InfraRed, Radar
from kinematic import KinematicControl
from constant import Pin
import RPi.GPIO as GPIO

import rospy
from std_msgs.msg import Int32


class ServoSystem:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # pins config
        pins = Pin()
        # peripheral instance
        # self.left_encode = Encoder()
        # self.right_encode = Encoder()
        self.left_motor = MotorOpen(pins.left_pin1, pins.left_pin2, pins.left_enale)
        self.right_motor = MotorOpen(pins.right_pin1, pins.right_pin2, pins.right_enale)
        self.font_infrared = InfraRed(pins.font_infra_pin)
        self.left_infrared = InfraRed(pins.left_infra_pin)
        self.right_infrared = InfraRed(pins.right_infra_pin)
        # self.font_rader = Radar()
        # self.left_rader = Radar(pins.left_radar_trig, pins.left_radar_echo)
        # self.right_rader = Radar(pins.right_radar_trig, pins.right_radar_echo)
        del pins

        # control instance
        self.kinematic = KinematicControl((self.left_motor, self.right_motor))

        # self.pub_grid = rospy.Publisher("/grid", Int32, queue_size=1)
        self.pub_detect = rospy.Publisher("/detect", Int32, queue_size=1)
        rospy.Subscriber("/state", Int32, self.statecallback, queue_size=1)
        rospy.Subscriber("/speed", Int32, self.speedcallback, queue_size=10)
        rospy.Subscriber("/angle", Int32, self.anglecallback, queue_size=10)

    def statecallback(self, msg):
        self.kinematic.set_state(msg.data)

    def speedcallback(self, msg):
        self.kinematic.set_speed(msg.data)

    def anglecallback(self, msg):
        self.kinematic.set_angle(msg.data)

    # def encode_spin(self):
    #     left_grid = self.left_encode.get_grid()
    #     right_grid = self.right_encode.get_grid()
    #     grid = (left_grid + right_grid) // 2
    #     self.pub_grid.publish(grid)

    # def encode_single_spin(self):
    #     left_grid = self.left_encode.get_grid()
    #     grid = left_grid // 2
    #     self.pub_grid.publish(grid)

    def infrared_spin(self):
        if self.kinematic.check_finish():
            # print(input('next_step'))
            font_detect = self.font_infrared.check_obstacle()
            back_detect = 1
            left_detect = self.right_infrared.check_obstacle()
            right_detect = self.left_infrared.check_obstacle()
            result = font_detect * 1000 + back_detect * 100 + left_detect * 10 + right_detect
            self.pub_detect.publish(Int32(result))
            print('result', result)

    # def radar_spin(self):
    #     left_distance = self.left_rader.get_distance()
    #     right_distance = self.right_rader.get_distance()
    #     self.kinematic.adjust_angle(left_distance, right_distance)

    def spin(self):
        # self.radar_spin()
        self.infrared_spin()
        self.kinematic.spin()
        # self.encode_spin()


if __name__ == '__main__':
    rospy.init_node('servo_node', anonymous=True)
    rate = rospy.Rate(20)
    try:
        servo_system = ServoSystem()
        time.sleep(1)
        while not rospy.is_shutdown():
            try:
                servo_system.spin()
            except Exception as e:
                print(e)
                traceback.print_exc()
            rate.sleep()
    except rospy.ROSInterruptException:
        print(rospy.ROSInterruptException)
