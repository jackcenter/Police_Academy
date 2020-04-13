#!/usr/bin/python3.8

import csv
import os

from chassis_controller import run_motion_plan


def main():
    run("command_1.txt", None)


def run(commands_filename, robot):
    command_list = read_commands(commands_filename)
    print([x.__dict__ for x in command_list])

    # TODO: set initial pose from first command

    # TODO: LOOP cycle through all commands
    for cmd in command_list:
        if cmd.get_mode() == 'MotionPlanning':
            run_motion_plan(cmd, robot)

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
    def __init__(self, step, mode, x, y, theta, w_right, w_left):
        self.step = step
        self.mode = mode
        self.x = x
        self.y = y
        self.theta = theta
        self.w_right = w_right
        self.w_left = w_left

    def get_mode(self):
        return self.mode

    def get_reference_list(self):
        return [self.x, self.y, self.theta, self.w_right, self.w_left]

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
            float(lookup['x']),
            float(lookup['y']),
            float(lookup['w_right']),
            float(lookup['w_left']),
            float(lookup['theta']),
        )


if __name__ == '__main__':
    main()
