
import rospy
import traceback
from mazesolution import MazeSolution


if __name__ == '__main__':
    rospy.init_node('maze_solution', anonymous=True)
    rate = rospy.Rate(10)
    try:
        maze_solution = MazeSolution()
        while (not rospy.is_shutdown()) or maze_solution.over is False:
            try:
                maze_solution.solve()
            except Exception as e:
                print(e)
                traceback.print_exc()
            rate.sleep()
    except rospy.ROSInterruptException:
        print(rospy.ROSInterruptException)
