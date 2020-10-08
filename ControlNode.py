#!/usr/bin/env python

import socket
import struct
import traceback
import threading
from constant import Command
from server import Server, get_host_ip
from multiprocessing.connection import Client

import rospy
from std_msgs.msg import Int32

Servo = XR_Servo()

HOST = get_host_ip()
print(HOST)

command = Command()

pub_speed = rospy.Publisher("speed", Int32, queue_size=1)
pub_angle = rospy.Publisher("angle", Int32, queue_size=1)
pub_state = rospy.Publisher("state", Int32, queue_size=1)

try:
    arm_cli = Client(('localhost', 8888), authkey=b'arm')
except Exception as e:
    print(e)
    traceback.print_exc()


def run(client):
    while True:
        try:
            _recv_ = client.recv(8)
            print('recv', _recv_)
            # 'c' is char and 'H' is unsigned int
            head, code, data1, data2, data3 = struct.unpack('ccHHH', _recv_)
            if head == command.head:
                if code == command.speed:
                    pub_speed.publish(data1)
                elif code == command.angle:
                    pub_angle.publish(data1)
                elif code == command.state:
                    pub_state.publish(data1)
                elif code == command.arm:
                    arm_cli.send([data1, data2])
                elif code == command.kill:
                    client.close()
            else:
                continue
        except socket.error as e:
            print(e)
            traceback.print_exc()
            client.close()
            del client
            return
        except Exception as e:
            print(e)
            traceback.print_exc()


def add_thread(client):
    thread_control = threading.Thread(None, target=run, args=(client,))
    thread_control.start()


if __name__ == '__main__':
    rospy.init_node('control_node', anonymous=True)
    rate = rospy.Rate(20)
    try:
        control = Server(HOST, 7777)
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
