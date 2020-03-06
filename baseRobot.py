from obstacleMap import ObstacleMap
from math import *
import numpy as np
import cv2
import sys


class Robot:
    def __init__(self, start, goal, radius, clearance, rigid=False):
        """
        Initialization of the robot.
        :param start: starting coordinates for the robot, in tuple form (y, x)
        :param goal: goal coordinates for the robot, in tuple form (y, x)

        Attributes:
            map: Instance of the obstacle map to be navigated
            start: Same as init argument start
            goal: Same as init argument start
            openList: List of coordinates pending exploration, in the form: [(y, x), cost, action]
            openGrid: Matrix storing "1" for cells pending exploration, and "0" otherwise
            closeGrid: Matrix storing "1" for cells that have been explored, and "0" otherwise
            actionGrid: Matrix storing the optimal movement policy for cells that have been explored, and 255 otherwise
            backTrack: User-friendly visualization of the optimal path
        """
        self.actions = [[0, 1, 1],
                        [-1, 1, sqrt(2)],
                        [-1, 0, 1],
                        [-1, -1, sqrt(2)],
                        [0, -1, 1],
                        [1, -1, sqrt(2)],
                        [1, 0, 1],
                        [1, 1, sqrt(2)]]
        self.start = start  # Starting coordinates in tuple form
        self.goal = goal  # Goal coordinates in tuple form

        # Handle radius and clearance arguments
        self.rigid = rigid
        self.radius = radius
        self.clearance = clearance
        if self.rigid:
            if self.radius < 0:
                sys.stdout.write("\nRadius is negative.  Exiting...\n")
                exit(0)
            elif self.clearance < 0:
                sys.stdout.write("\nClearance is negative.  Exiting...\n")
                exit(0)
            if self.radius == 0:
                sys.stdout.write("\nRadius is zero.  This is a point robot with clearance %d." % self.clearance)

        # Generate an instance of the map object
        self.map = ObstacleMap(radius + clearance)

        # Check to see if start and goal cells lie within map boundaries
        if not (0 <= self.start[0] < self.map.height) or not (0 <= self.start[1] < self.map.width):
            sys.stdout.write("\nStart lies outside of map boundaries!\n")
            exit(0)
        elif not (0 <= self.goal[0] < self.map.height) or not (0 <= self.goal[1] < self.map.width):
            sys.stdout.write("\nGoal lies outside of map boundaries!\n")
            exit(0)

        # Check to see if start and goal cells are in free spaces
        elif self.map.obstacleSpace[self.start]:
            sys.stdout.write("\nStart lies within obstacle space!\n")
            exit(0)
        elif self.map.obstacleSpace[self.goal]:
            sys.stdout.write("\nGoal lies within obstacle space!\n")
            exit(0)

        # Define cell maps to track exploration
        self.openList = []  # List of coordinates to be explored, in the form: [(y, x), cost, action]
        self.openGrid = np.zeros_like(self.map.baseImage, dtype=np.uint8)  # Grid of cells pending exploration
        self.closeGrid = np.zeros_like(self.map.baseImage, dtype=np.uint8)  # Grid of explored cells
        self.actionGrid = np.zeros_like(self.map.baseImage, dtype=np.uint8) + 255  # Grid containing movement policy
        self.backTrack = cv2.cvtColor(np.copy(self.map.baseImage)*255, cv2.COLOR_GRAY2RGB)  # Image of the optimal path
        self.solve()

    def solve(self):
        """
        Solves the puzzle
        """
        # Initialize the open list/grid with the start cell
        self.openList = [[self.start, 0, 255]]  # [point, cost, action]
        self.openGrid[self.start] = 1
        sys.stdout.write("\nStarting path finder...")
        while len(self.openList) > 0:
            # Find index of minimum cost cell
            cost_list = [self.openList[i][1] for i in range(len(self.openList))]
            index = int(np.argmin(cost_list, axis=0))
            cell = self.openList[index][0]
            cost = self.openList[index][1]
            action = self.openList[index][2]

            # See if goal cell has been reached
            if cell == self.goal:
                sys.stdout.write("\nGoal reached!\n")
                self.openList = []

            # Expand cell
            else:
                for a in range(len(self.actions)):
                    next_cell = (cell[0] + self.actions[a][0], cell[1]+self.actions[a][1])
                    # Check for map boundaries
                    if 0 <= next_cell[0] < self.map.height and 0 <= next_cell[1] < self.map.width:
                        # Check for obstacles
                        if not self.map.obstacleSpace[next_cell[0], next_cell[1]]:
                            # Check whether cell has been explored
                            if not self.closeGrid[next_cell[0], next_cell[1]]:
                                # Check if cell is already pending exploration
                                if not self.openGrid[next_cell[0], next_cell[1]]:
                                    self.openList.append([next_cell, cost + self.actions[a][2], a])
                                    self.openGrid[next_cell] = 1
                                else:
                                    # TODO:  Handle open cell being approached from two cells with same cumulative cost
                                    pass
                self.openList.pop(index)

            # Mark the cell as having been explored
            self.openGrid[cell] = 0
            self.closeGrid[cell] = 1
            self.actionGrid[cell] = action
            cv2.imshow("Maze", self.closeGrid * 255)
            cv2.waitKey(1)
        cv2.destroyWindow("Maze")
        current_cell = self.goal
        next_action_index = self.actionGrid[current_cell]

        # Check for failure to reach the goal cell
        if next_action_index == 255:
            sys.stdout.write("\nFailed to find a path to the goal!\n")

        # Backtracking from the goal cell to extract an optimal path
        else:
            while next_action_index != 255:
                current_cell = (current_cell[0] - self.actions[next_action_index][0],
                                current_cell[1] - self.actions[next_action_index][1])
                self.backTrack[current_cell] = (255, 0, 255)
                next_action_index = self.actionGrid[current_cell]
            cv2.imshow("Goal", self.backTrack)
            cv2.waitKey(0)
            cv2.destroyWindow("Goal")
