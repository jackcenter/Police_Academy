#!/usr/bin/python3.8

import csv
import os

import numpy as np
import smbus

from chassis_controller import run_motion_plan
from encoders import DriveTrain, Encoder
from filter import Filter
from ultrasonics import Ultrasonic


def main():
    simple_filter = setup()
    run("commands_3.txt", simple_filter)


def setup():
    bus = smbus.SMBus(1)
    slave_address = 0x07

    enc_left = Encoder('left', 100, bus, slave_address)
    enc_right = Encoder('right', 100, bus, slave_address)
    drive_train = DriveTrain(enc_left, enc_right, bus, slave_address)

    ult_left = Ultrasonic('left', 22, 23)
    ult_front = Ultrasonic('front', 20, 21)
    ult_right = Ultrasonic('right', 18, 19)

    simple_filter = Filter(drive_train, ult_left, ult_front, ult_right, bus, slave_address)

    return simple_filter


def run(commands_filename, simple_filter):
    command_list = read_commands(commands_filename)
    print([x.__dict__ for x in command_list])

    # set initial pose from first command
    initial_pose_cmd = command_list.pop(0)
    simple_filter.set_state(initial_pose_cmd.get_reference_array())

    # LOOP cycle through all commands
    for cmd in command_list:
        if cmd.get_mode() == 'MotionPlanning':
            run_motion_plan(cmd, simple_filter)

        elif cmd.get_mode() == 'Search&Destroy':
            # TODO: fit the shooting stuff in here
            pass

    return 0


def read_commands(filename):
    """
    reads a text file of state commands for the robot to execute in order.
    :param filename: string of the full path to the commands text file
    :return: list of command objects
    """
    file = get_command_file(filename)

    with open(file, 'r', encoding='utf8') as fin:
        reader = csv.DictReader(fin, skipinitialspace=True, delimiter=',')

        command_list = []
        for row in reader:
            u = Command.create_from_dict(row)
            command_list.append(u)

    return command_list


def get_command_file(filename):
    """
    Gets the full path and name for the text file of commands in the commands folder
    :param filename: string of the name of the text file to find
    :return: string of the path to the text file
    """
    base_folder = os.path.dirname(__file__)
    return os.path.join(base_folder, 'cmds', filename)


class Command:
    def __init__(self, step, mode, action, x, y, theta, w_right, w_left):
        self.step = step
        self.mode = mode
        self.action = action
        self.x = x
        self.y = y
        self.theta = theta
        self.w_right = w_right
        self.w_left = w_left

    def get_mode(self):
        return self.mode

    def get_action(self):
        return self.action

    def get_reference_list(self):
        return [self.x, self.y, self.theta, self.w_right, self.w_left]

    def get_reference_array(self):
        return np.array([self.x, self.y, self.theta, self.w_right, self.w_left])

    def print_ref(self):
        vals = self.get_reference_array()
        print("Reference State:\n  x     = {}\n  y     = {}\n  theta = {}\n  w_l   = {}\n  w_r   = {}".format(
              vals[0], vals[1], vals[2], vals[3], vals[4]))
        print()

    @staticmethod
    def create_from_dict(lookup: dict):
        """
        creates a Command object from a dictionary of command values for where the x, y, and theta positions should be
        driven to.
        :param lookup: dictionary of command values
        :return:
        """

        return Command(
            int(lookup['step']),
            str(lookup['mode']),
            str(lookup['action']),
            float(lookup['x']),
            float(lookup['y']),
            float(lookup['theta']),
            float(lookup['w_right']),
            float(lookup['w_left']),
        )


if __name__ == '__main__':
    main()
