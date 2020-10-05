# !/usr/bin/env python

import traceback
from control import TcpControl, add_thread

import rospy


if __name__ == '__main__':
    rospy.init_node('control_node', anonymous=True)
    rate = rospy.Rate(20)
    try:
        control = TcpControl()
        while not rospy.is_shutdown():
            try:
                control.accept()
                add_thread(control.client)
            except Exception as e:
                print(e)
                traceback.print_exc()
            rate.sleep()
    except rospy.ROSInterruptException:
        print(rospy.ROSInterruptException)
