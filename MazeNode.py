#!/usr/bin/env python

import rospy
import traceback
from mazesolutionV2 import MazeSolution


if __name__ == '__main__':
    rospy.init_node('maze_solution', anonymous=True)
    rate = rospy.Rate(10)
    try:
        maze_solution = MazeSolution()
        while not rospy.is_shutdown():
            try:
                maze_solution.solve()
                if maze_solution.over:
                    print('maze out')
                    break
            except Exception as e:
                print(e)
                traceback.print_exc()
            rate.sleep()
    except rospy.ROSInterruptException:
        print(rospy.ROSInterruptException)
