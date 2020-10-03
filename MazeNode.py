
import rospy
import traceback
import numpy as np
from constant import MazeInfo, DirctionEnum

from std_msgs.msg import Int32


class MazeSolution:
    def __init__(self):
        info = MazeInfo()
        self.maze_wall = info.maze_wall
        self.maze_road = info.maze_road
        self.maze_pass = info.maze_pass
        self.maze_unknown = info.maze_unknown
        del info

        enum = DirctionEnum()
        self.up = enum.dir_up
        self.down = enum.dir_down
        self.left = enum.dir_left
        self.right = enum.dir_right
        del enum

        # loacl variale
        self.maze_map = None
        self.cur_loction = None
        self.cur_direction = None
        self.cur_grid = 0
        self.is_new_grid = False

        self.create_map()

        rospy.Subscriber("/grid", Int32, self.gridcallback, queue_size=3)
        rospy.Subscriber("/detect", Int32, self.detectcallback, queue_size=3)

    def gridcallback(self, msg):
        """
        Current grid information will be published constantly.
        When car enter new grid, car should publish new dirction.
        """
        if self.cur_grid != msg.data:
            self.cur_grid = msg.data
            self.is_new_grid = True

    def detectcallback(self, msg):
        """
        Current detection result will be published cosntantly.
        :param msg: Int32, [0:3] is left, right, front and back detion result.
        """
        # TODO: make use of mean filtering to advoid accident err.


    def create_map(self, size):
        """
        create a new maze map
        :param size: the size of map
        """
        # TODO: Create a maze whose inner grids are unknown and around grids are wall.
        self.maze_map = np.ones(shape=) * self.maze_unknown


    def spin(self):
        pass


if __name__ == '__main__':
    rospy.init_node('maze_solution', anonymous=True)
    rate = rospy.Rate(10)
    try:
        maze_solution = MazeSolution()
        while not rospy.is_shutdown():
            try:
                maze_solution.spin()
            except Exception as e:
                print(e)
                traceback.print_exc()
            rate.sleep()
    except rospy.ROSInterruptException:
        print(rospy.ROSInterruptException)
