#!/usr/bin/python3.8

import os
import csv
import simple_pid


def main():
    pass


def run_motion_plan(cmd):
    # TODO: have this change depending on the ref
    ref = [10, 0, 0, 2, 2]

    coefficients_filename = 'coefficients.txt'
    controller = create_controller(coefficients_filename, ref)
    print(controller.__dict__)


def create_controller(coefficients_filename, ref):
    coefficients_list = read_coefficients(coefficients_filename)
    controller = ChassisPID.create_from_PID_list(coefficients_list)
    controller.update_reference(ref)

    return controller


def get_coefficients_file(filename):
    """
    Gets the full path and name for the text file of commands in the commands folder
    :param filename: string of the name of the text file to find
    :return: string of the path to the text file
    """
    base_folder = os.path.dirname(__file__)
    return os.path.join(base_folder, 'cmds', filename)


def read_coefficients(filename):
    """
    reads a text file of state commands for the robot to execute in order.
    :param filename: string of the full path to the commands text file
    :return: list of command objects
    """
    file = get_coefficients_file(filename)

    with open(file, 'r', encoding='utf8') as fin:
        reader = csv.DictReader(fin, skipinitialspace=True, delimiter=',')

        coefficients_list = []
        for row in reader:
            k = PIDCoefficients.create_from_dict(row)
            coefficients_list.append(k)

    return coefficients_list


class ChassisPID:
    def __init__(self, k_x1: dict, k_x2: dict, k_x3: dict, k_x4: dict, k_x5: dict, state_names: list):
        self.k_x1 = k_x1
        self.k_x2 = k_x2
        self.k_x3 = k_x3
        self.k_x4 = k_x4
        self.k_x5 = k_x5
        self.state_names = state_names

        self.ref = []
        self.coefficients_list = [self.k_x1, self.k_x2, self.k_x3, self.k_x4, self.k_x5]

        # TODO: create a number of simple_pid objects based on the k values
        self.controller_dict = {}
        for k in self.coefficients_list:
            controller = simple_pid.PID(k['kp'], k['ki'], k['kd'])
            self.controller_dict.update({k['state']: controller})

    def update_reference(self, ref):
        self.ref = ref

        for state, r in zip(self.state_names, self.ref):
            self.controller_dict[state].setpoint = r

    def run(self):
        pass

    @staticmethod
    def create_from_PID_list(PID_list: list):
        """
        creates a PID controller for the chassis from a list of PIDCoefficients objects.
        :param PID_list: list of PIDCoefficients objects
        :return: ChassisPID object
        """
        state_names = []
        for item in PID_list:
            state_names.append(item.get_state_name())

        return ChassisPID(
            PID_list[0].get_coefficients_dict(),
            PID_list[1].get_coefficients_dict(),
            PID_list[2].get_coefficients_dict(),
            PID_list[3].get_coefficients_dict(),
            PID_list[4].get_coefficients_dict(),
            state_names
        )


class PIDCoefficients:
    def __init__(self, state_name: str, kp: float, ki: float, kd: float):
        self.state_name = state_name
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def get_coefficients_dict(self):
        return {'state': self.state_name, 'kp': self.kp, 'ki': self.ki, 'kd': self.kd}

    def get_state_name(self):
        return self.state_name

    @staticmethod
    def create_from_dict(lookup: dict):
        """
        creates a PIDCoefficients object from a dictionary of PID coefficients
        :param lookup: dictionary of command values
        :return:
        """

        return PIDCoefficients(
            str(lookup['state']),
            float(lookup['kp']),
            float(lookup['ki']),
            float(lookup['kd']),
        )


if __name__ == '__main__':
    main()
