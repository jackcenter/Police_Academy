import smbus
import time
import os
import numpy as np
from simple_filter import Filter


def main():
    bus = smbus.SMBus(1)
    slave_address = 0x07        # Chassis Arduino

    u1_ref = 2     # velocity
    u2_ref = 0      # heading
    u_ref = np.array([u1_ref, u2_ref])

    kp = 3
    ki = 0.03
    kd = 0.3

    state_estimate = Filter(bus, slave_address)
    time.sleep(1)
    controller = PID(kp, ki, kd, 2)

    time_start = time.time()
    time_elapsed = 0

    while time_elapsed < 10:
        # print("State Estimate:")
        state = state_estimate.get_state()
        print(state)

        u = controller.run_pid(u_ref, state)
        u_int = u.astype(int)

        u_int = set_range(u_int, -3, 3)

        print("Command: ")
        # print(u)
        print(u_int)
        print()

        bytesToSend = ConvertInputToBytes(u_int)
        time.sleep(0.1) # delay for wire to settle
        bus.write_i2c_block_data(slave_address, 0, bytesToSend)
        time_elapsed = time.time()-time_start

    u1_ref = 0      # velocity
    u2_ref = 0      # heading
    u_ref = np.array([u1_ref, u2_ref])

    while time_elapsed < 20:
        # print("State Estimate:")
        state = state_estimate.get_state()
        print("State Values")
        print(state)

        u = controller.run_pid(u_ref, state)
        u_int = u.astype(int)

        u_int = set_range(u_int, -3, 3)

        print("Command: ")
        # print(u)
        print(u_int)
        print()

        bytesToSend = ConvertInputToBytes(u_int)
        time.sleep(0.1) # delay for wire to settle
        bus.write_i2c_block_data(slave_address, 0, bytesToSend)
        time_elapsed = time.time()-time_start


class PID:
    def __init__(self, kp, ki, kd, dim):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.e = np.zeros(dim)
        self.e_sum = np.zeros(dim)
        self.e_k0 = np.array(dim)

        self.k0 = time.time()
        self.k1 = time.time()

    def run_pid(self, ref, state):
        """
        current state is just motor speeds: w_r, w_l
        :param ref:
        :param state:
        :return:
        """
        self.k1 = time.time()

        u1_ref = ref[0]
        u2_ref = ref[1]

        enc_r = state[0]
        w_r = state[1]
        enc_l = state[2]
        w_l = state[3]

        u1 = (w_r + w_l)/2
        # u2 = w_r - w_l

        e1 = u1_ref - u1
        e2 = u2_ref - (enc_r - enc_l)
        self.e = np.array([e1, e2])
        print("Error Values:")
        print(self.e)

        # Proportional ====================================
        u_p = self.kp * self.e

        # Integral ========================================
        for e in self.e:
            if e > 1:
                self.e_sum += e
        u_i = self.ki * self.e_sum

        # Derivative ======================================
        e_dot = (self.e - self.e_k0)/(self.k1 - self.k0)
        self.e_k0 = self.e
        u_d = self.kd*e_dot

        u = u_p + u_i + u_d

        return u


def get_state_estimate():
    return 50, 50


def ConvertInputToBytes(src):
    converted = [(src[0] + 3).item(), (src[1] + 3).item()]
    return converted


def set_range(array, lower, upper):
    new_array = np.copy(array)
    new_array[new_array < lower] = lower
    new_array[new_array > upper] = upper
        
    return new_array


if __name__ == '__main__':
    main()
