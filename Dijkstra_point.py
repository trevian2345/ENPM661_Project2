from obstacleMap import ObstacleMap
from math import *
import numpy as np
import cv2
import sys
import argparse

class PointRobot:
    def __init__(self, start, goal):
        self.actions = [[0, 1, 1],
                        [-1, 1, sqrt(2)],
                        [-1, 0, 1],
                        [-1, -1, sqrt(2)],
                        [0, -1, 1],
                        [1, -1, sqrt(2)],
                        [1, 0, 1],
                        [1, 1, sqrt(2)]]
        self.map = ObstacleMap()
        self.start = self.parse_point(start)
        self.goal = self.parse_point(goal)
        if self.map.image[self.start]:
            print("Start lies within obstacle space!")
            exit(0)
        elif self.map.image[self.goal]:
            print("Goal lies within obstacle space!")
            exit(0)
        elif not (0 <= self.start[0] < self.map.height) or not (0 <= self.start[1] < self.map.width):
            print("Start lies outside of map boundaries!")
            exit(0)
        elif not (0 <= self.goal[0] < self.map.height) or not (0 <= self.goal[1] < self.map.width):
            print("Goal lies outside of map boundaries!")
            exit(0)
        self.openList = []
        self.openGrid = np.zeros_like(self.map.image, dtype=np.uint8)
        self.closeGrid = np.zeros_like(self.map.image, dtype=np.uint8)
        self.actionGrid = np.zeros_like(self.map.image, dtype=np.uint8)
        self.solve()

    def parse_point(self, point_string):
        self.goal = point_string
        return point_string

    def move(self, action):
        pass

    def solve(self):
        self.openList = [[self.start, 0, -1]]  # [point, cost, action]
        self.openGrid[self.start] = 1
        sys.stdout.write("\nStarting path finder...\n")
        while len(self.openList) > 0:
            # Find index of minimum cost cell
            # print(self.openList)
            cost_list = [self.openList[i][1] for i in range(len(self.openList))]
            index = int(np.argmin(cost_list, axis=0))
            cell = self.openList[index][0]
            cost = self.openList[index][1]
            action = self.openList[index][2]

            # See if goal cell has been reached
            if cell == self.goal:
                sys.stdout.write("Goal reached!")
                self.openList = []
                continue

            # Expand cell
            for a in range(len(self.actions)):
                next_cell = (cell[0] + self.actions[a][0], cell[1]+self.actions[a][1])
                # Check for map boundaries
                if 0 <= next_cell[0] < self.map.height and 0 <= next_cell[1] < self.map.width:
                    # Check for obstacles
                    if not self.map.image[next_cell[0], next_cell[1]]:
                        # Check whether cell has been explored
                        if not self.closeGrid[next_cell[0], next_cell[1]]:
                            # Check if cell is already pending exploration
                            if not self.openGrid[next_cell[0], next_cell[1]]:
                                self.openList.append([next_cell, cost + self.actions[a][2], a])
                                self.openGrid[next_cell] = 1
                            else:
                                # TODO:  Handle open cell being approached from two cells with same cumulative cost
                                pass

            # Close cell
            self.openGrid[cell] = 0
            self.closeGrid[cell] = 1
            self.actionGrid[cell] = action
            self.openList.pop(index)
            cv2.imshow("Maze", self.closeGrid * 255)
            cv2.waitKey(1)


if __name__ == '__main__':
    #PointRobot(start=(190, 10), goal=(10, 290))
    parser = argparse.ArgumentParser()
    parser.add_argument('initialx',type=int, help='X-coordinate of Initial node of the robot')
    parser.add_argument('initialy', type=int, help='Y-coordinate of Initial node of the robot')
    parser.add_argument('goalx',type=int, help='X-coordinate of Goal node of the robot')
    parser.add_argument('goaly', type=int, help='Y-coordinate of Goal node of the robot')
    args = parser.parse_args()

    sx = args.initialx
    sy = args.initialy
    gx = args.goalx
    gy= args.goaly
    start_pos = (sx,sy)
    goal_pos = (gx, gy)
    pointrob = PointRobot(start_pos, goal_pos)
    
