from obstacleMap import ObstacleMap
from math import *


class PointRobot:
    def __init__(self, goal):
        self.actions = [0, 1, 1,
                        -1, 1, sqrt(2),
                        -1, 0, 1,
                        -1, -1, sqrt(2),
                        0, -1, 1,
                        1, -1, sqrt(2),
                        1, 0, 1,
                        1, 1, sqrt(2)]
        self.openList = []
        self.closeList = []
        self.goal = goal
        self.map = ObstacleMap()

    def move(self, action):
        pass


if __name__ == '__main__':
    PointRobot((100, 100))
