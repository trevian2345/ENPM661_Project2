import numpy as np
import cv2
from math import *


class ObstacleMap:
    def __init__(self, distance):
        """
        Initialization of the obstacle map
        :param distance: Minimum distance between the center of the robot from any point in the obstacle space.
                         Accounts for both minimum clearance and the radius of the robot.
        """
        self.height = 200
        self.width = 300
        self.thickness = distance
        self.baseImage = np.zeros((self.height, self.width), dtype=np.uint8)
        self.obstacleSpace = np.copy(self.baseImage)
        self.clearanceSpace = np.copy(self.baseImage)
        self.window_name = "Image"
        self.generate_map()

    def show(self, image):
        cv2.imshow(self.window_name, image * 255)
        cv2.waitKey(0)
        cv2.destroyWindow(self.window_name)

    def generate_map(self):
        shapes_to_draw = []
        points = np.array([[25, 185], [75, 185], [100, 150], [75, 120], [50, 150], [20, 120]],
                          dtype=np.int32).reshape((-1, 1, 2))
        for i in range(len(points)):
            points[i][0] = [points[i][0][0], self.height - points[i][0][1]]
        shapes_to_draw.append(points)
        points = np.array([[95, 170], [95 - 75*sqrt(3)/2, 170 - 75/2],
                           [95 - 75*sqrt(3)/2 + 10/2, 170 - 75/2 - 10*sin(60*pi/180)],
                           [95 + 10/2, 170 - 10*sin(60*pi/180)]], dtype=np.int32).reshape((-1, 1, 2))
        shapes_to_draw.append(points)
        points = np.array([[225, 190], [250, 175], [225, 160], [200, 175]], dtype=np.int32).reshape((-1, 1, 2))
        shapes_to_draw.append(points)
        points = np.array([[150 + 40*cos(a*pi/180), 100 + 20*sin(a*pi/180)]
                           for a in range(360)], dtype=np.int32).reshape((-1, 1, 2))
        shapes_to_draw.append(points)
        points = np.array([[225 + 25*cos(a*pi/180), 50 + 25*sin(a*pi/180)]
                           for a in range(360)], dtype=np.int32).reshape((-1, 1, 2))
        shapes_to_draw.append(points)
        cv2.fillPoly(self.baseImage, shapes_to_draw, 1)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.thickness*2 + 1, self.thickness*2 + 1))
        self.obstacleSpace = cv2.dilate(np.copy(self.baseImage), kernel)
        self.clearanceSpace = self.obstacleSpace - self.baseImage

        # TODO: Map boundaries should be treated as walls
