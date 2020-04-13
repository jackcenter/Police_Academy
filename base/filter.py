from math import pi
import time

import numpy as np

from encoders import DriveTrain
from ultrasonics import Ultrasonic


class Filter:
    def __init__(self, drive_train: DriveTrain, ult_left: Ultrasonic, ult_front: Ultrasonic,
                 ult_right: Ultrasonic):

        self.drive_train = drive_train
        self.ult_left = ult_left
        self.ult_front = ult_front
        self.ult_right = ult_right
        self.sensor_list = [self.drive_train, self.ult_left, self.ult_front, self.ult_right]

        self.initial_state = np.array([0, 0, 0, 0, 0])
        self.current_state = np.array([0, 0, 0, 0, 0])
        self.distance_conversion = 2*pi*1.9/self.drive_train.cpr
        self.rotation_conversion = 8.5/(2*1.9)

    def get_state_array(self):
        self.update()
        x = self.get_translation()
        y = self.get_alignment()
        theta = self.get_rotation()
        w_l, w_r = self.get_velocities()

        self.current_state = np.array([x, y, theta, w_l, w_r])

        return self.current_state

    def update(self):
        for sensor in self.sensor_list:
            sensor.update()

    def get_translation(self):
        """
        determines how far the robot has drive in a straight line
        :return:
        """
        x = self.drive_train.get_distance()*self.distance_conversion
        return x

    def get_alignment(self, threshold=20):
        """
        determines how far off center line the robot is base on ultrasonic readings. Defaults to 4 inches if the
        readings are beyond the threshold.
        :return: y axis error
        """

        ult_right = self.ult_right.get_value()
        if ult_right > threshold:
            ult_right = 4

        ult_left = self.ult_left.get_value()
        if ult_left > threshold:
            ult_left = 4

        return ult_right - ult_left

    def get_rotation(self):
        """
        determines how much the robot has turned
        :return:
        """
        theta = self.drive_train.get_rotation()*self.rotation_conversion
        return theta

    def get_velocities(self):
        """
        pulls velocities from the drive train estimate
        :return:
        """
        w_l, w_r = self.drive_train.get_velocities()
        return w_l, w_r

    def set_state(self, state):
        self.initial_state = state

    def print_state(self):
        vals = self.current_state
        print("Current State:\n  x =     {}\n y =     {}\n theta = {}\n w_l   = {}\n w_r   = {}".format(
              vals[0], vals[1], vals[2], vals[3], vals[4]))
        print()
