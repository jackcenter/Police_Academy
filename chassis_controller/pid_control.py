# import smbus
import time
import os

from simple_filter import Filter


def main():
    # bus = smbus.SMBus(1)
    slave_address = 0x07        # Chassis Arduino

    u1_ref = 1      # velocity
    u2_ref = 0      # heading
    u_ref = [u1_ref, u2_ref]

    kp = 1
    ki = 0.01
    kd = 0.1

    state_estimate = Filter(1, slave_address)
    time.sleep(1)
    print(state_estimate.get_state_test())

    u = run_pid(u_ref, kp, ki, kd, state_estimate)
    bytesToSend = ConvertInputToBytes(u)
    # bus.write_i2c_block_data(slave_address, 0, bytesToSend)


def run_pid(ref, kp, ki, kd, state):
    """
    current state is just motor speeds: w_r, w_l
    :param ref:
    :return:
    """
    u1_ref = ref[0]
    u2_ref = ref[1]

    state = get_state_estimate()
    w_r = state[0]
    w_l = state[1]

    u1 = (w_r + w_l)/2
    u2 = w_r - w_l

    e1 = u1_ref - u1
    e2 = u2_ref - u2

    return 0, 0


def get_state_estimate():
    return 50, 50


def ConvertInputToBytes(src):
    converted = [src[0] + 3, src[1] + 3]
    return converted


if __name__ == '__main__':
    main()
