
import numpy as np

from constant import MazeInfo


class BackTrack:
    def __init__(self):
        info = MazeInfo()

        self.map_size = info.map_size
        self.map = None
        self.init()

        del info

    def init(self):
        """
        init map and location
        :return:
        """
        self.map = np.zeros(self.map_size)

    def get(self):
        """
        get a target diretion
        :return:
        """
        pass

    def update(self, surround):
        """
        update map
        :param surround:
        :return:
        """
        pass

    def location(self, vel_dir):
        """
        location
        :return:
        """
        pass

