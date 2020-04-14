#!/usr/bin/python3.8

import csv
import os
import time

import numpy as np
import simple_pid


def main():
    pass


def run_motion_plan(cmd, simple_filter):
    """
    TODO: need to pass robot to this function to get current state
    :param cmd: Command object containing the reference value for each state
    :param simple_filter: object that contains the sensors
    :return:
    """
    simple_filter.set_state = [0, 0, 0, 0, 0]
    ref = cmd.get_reference_array()
    # TODO: have this change depending on the ref
    coefficients_filename = 'coefficients.txt'
    controller = create_controller(coefficients_filename, ref)

    start = time.time()
    t = start
    while t - start < 20:
        # TODO: add an actual stop process, not time
        time.sleep(.1)
        current_state = simple_filter.get_state_array()
        cmd.print_ref()
        simple_filter.print_state()
        u = controller.run(current_state)

        u_omega = u['x'] + u['w_left'] + u['w_right']
        u_psi = u['y'] + u['theta']

        command = [u_omega, u_psi]
        converted_command = send_command(command, simple_filter.bus, simple_filter.slave_address)

        print_dict_pretty("Input Components:", u)
        print_dict_pretty("Inputs:", {"U_omega": u_omega, "U_psi": u_psi})
        print_dict_pretty("Inputs Sent:", converted_command)
        t = time.time()

    print("================== Next Command ======================")

    return 0


def send_command(command, bus, slave_address):
    pass
    # TODO: set command
    command = set_range(command, -3, 3)
    converted = [int(command[0]), (int(command[1]))]
    bus.write_i2c_block_data(slave_address, 0, converted)
    return {'U_omega': converted[0], 'U_psi': converted[1]}


def set_range(array, lower, upper):
    new_array = np.copy(array)
    new_array[new_array < lower] = lower
    new_array[new_array > upper] = upper

    return new_array


def print_dict_pretty(title, item):
    print(title)
    for key, val in item.items():
        print("  {} = {}".format(key, val))
    print()


def create_controller(coefficients_filename, ref):
    """

    :param coefficients_filename: string with full path to PID coefficients
    :param ref: values the controller is commanded to
    :return: controller object containing five PID controllers, one for each state
    """
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
        coefficients_list = [self.k_x1, self.k_x2, self.k_x3, self.k_x4, self.k_x5]

        # TODO: create a number of simple_pid objects based on the k values
        self.controller_dict = {}
        for k in coefficients_list:
            controller = simple_pid.PID(k['kp'], k['ki'], k['kd'])
            self.controller_dict.update({k['state']: controller})

    def update_reference(self, ref: list):
        """
        changes the reference values in the PID controllers
        :param ref: list of reference values
        :return: none
        """
        self.ref = ref

        for state, r in zip(self.state_names, self.ref):
            self.controller_dict[state].setpoint = r

    def run(self, current_state: np.ndarray):
        """
        returns the cumulative PID input for the current state
        :param current_state: TODO: figure out data type coming in
        :return: list of commands for u_omega and u_psi
        """
        u_dict = {}
        for x, state_name in zip(current_state, self.state_names):
            controller = self.controller_dict[state_name]
            u = controller(x)
            u_dict.update({state_name: u})

        u_dict = self.apply_exceptions(u_dict)
        # TODO: convert to translation and turn commands.
        return u_dict

    @staticmethod
    def apply_exceptions(u_dict):
        """
        adjusts inputs base on heuristics
        :param u_dict: dictionary of the inputs from the controllers
        :return: updated input dictionary
        """
        if abs(u_dict['w_right']) > 0:
            u_dict.update({'w_right': 0})

        if abs(u_dict['w_left']) > 0:
            u_dict.update({'w_left': 0})

        return u_dict

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