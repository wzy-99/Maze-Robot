#!/usr/bin/env python

import struct
import socket
import threading
import traceback
from _XiaoRGEEK_SERVO_ import XR_Servo
from server import Server

import rospy

HOST = socket.gethostbyname(socket.gethostname())

Servo = XR_Servo()

Servo.XiaoRGEEK_SetServoAngle(1, 30)


def init():
    Servo.XiaoRGEEK_SetServoAngle(1, 90)
    Servo.XiaoRGEEK_SetServoAngle(2, 90)
    Servo.XiaoRGEEK_SetServoAngle(3, 90)
    Servo.XiaoRGEEK_SetServoAngle(4, 90)


def run(client):
    while True:
        try:
            _recv_ = client.recv(5)
            print('recv', _recv_)
            # 'c' is char and 'H' is unsigned int
            head, number, angle = struct.unpack('cHH', _recv_)
            angle = max(0, min(180, angle))
            number = max(1, min(4, number))
            Servo.XiaoRGEEK_SetServoAngle(number, angle)
        except socket.error as e:
            print(e)
            client.close()
            del client
            return
        except Exception as e:
            print(e)


def add_thread(client):
    thread_control = threading.Thread(None, target=run, args=(client,))
    thread_control.start()


if __name__ == '__main__':
    rospy.init_node('arm_node', anonymous=True)
    rate = rospy.Rate(20)
    try:
        control = Server(HOST, 6666)
        while not rospy.is_shutdown():
            try:
                cli = control.accept()
                add_thread(cli)
            except Exception as e:
                print(e)
                traceback.print_exc()
            rate.sleep()
    except rospy.ROSInterruptException:
        print(rospy.ROSInterruptException)
