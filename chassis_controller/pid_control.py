import smbus
import time
import os
import numpy as np
import simple_pid
from simple_filter import Filter


def main():
    bus = smbus.SMBus(1)
    slave_address = 0x07        # Chassis Arduino

    u1_ref = 2      # velocity
    u2_ref = 0      # heading
    u3_ref = 0      # ultrasonics
    u_ref = np.array([u1_ref, u2_ref, u3_ref])

    kp = np.diag([3, 0.4, 2])
    ki = np.diag([0, 0.0, 0.0])
    kd = np.diag([0.3, 40.0, .1])
    
#     u1_ref = 0      # velocity
#     u2_ref = -60    # heading
#     u3_ref = 0      # ultrasonics
#     u_ref = np.array([u1_ref, u2_ref, u3_ref])    
#     kp = np.diag([0, 1, 0])
#     ki = np.diag([0, 0.0, 0])
#     kd = np.diag([0, 50, 0])

    state_estimate = Filter(bus, slave_address)
    time.sleep(1)
    controller = PID(kp, ki, kd, 3)
    test_controller1 = simple_pid.PID(3, 0, .3, setpoint=2)
    test_controller2 = simple_pid.PID(0.4, 0, 40, setpoint=0)
    test_controller3 = simple_pid.PID(2, 0, .1, setpoint=0)


    time_start = time.time()
    time_elapsed = 0

    while time_elapsed < 15:
        state = state_estimate.get_state()
        print("State Values")
        print(" Encoders: {}, {}\n Velocity: {}, {}\n Ultrason: {}, {}, {}".format(state[0], state[1], state[2], state[3], state[4], state[5], state[6]))
        print()

        u = controller.run_pid(u_ref, state)
        u_int = u.astype(int)
        u_omega = u_int[0]
        u_phi = u_int[1] + u_int[2]

        u_omega = set_range(u_omega, -6, 6)
        u_phi = set_range(u_phi, -6, 6)

        cmd = [u_omega, u_phi]

        print("Command: ")
        print(cmd)
        print()

        bytesToSend = ConvertInputToBytes(cmd)
        time.sleep(0.2) # delay for wire to settle
        bus.write_i2c_block_data(slave_address, 0, bytesToSend)
        time_elapsed = time.time()-time_start

        # TEST ===========================================
#         e_r = state[0]
#         e_l = state[1]
#         v_r = state[2]
#         v_l = state[3]
#         ul_r = state[4]
#         ul_l = state[6]
#         u_test1 = test_controller1(v_r-v_l)
#         u_test2 = test_controller2(e_r-e_l)
#         u_test3 = test_controller3(ul_r-ul_l)
#         print("Test = ")
#         print(u_test1)
#         print(u_test2)
#         print(u_test3)
#         print()

    u1_ref = 0      # velocity
    u2_ref = 0      # heading
    u3_ref = 0
    u_ref = np.array([u1_ref, u2_ref, u3_ref])
    print("SLOW DOWN===============================")

    while time_elapsed < 30:
        # print("State Estimate:")
        state = state_estimate.get_state()
        print("State Values")
        print(" Encoders: {}, {}\n Velocity: {}, {}\n Ultrason: {}, {}, {}".format(state[0], state[1], state[2], state[3], state[4], state[5], state[6]))
        print()

        u = controller.run_pid(u_ref, state)
        u_int = u.astype(int)
        u_omega = u_int[0]
        u_phi = u_int[1] + u_int[2]

        u_omega = set_range(u_omega, -6, 6)
        u_phi = set_range(u_phi, -6, 6)

        cmd = [u_omega, u_phi]

        print("Command: ")
        print(cmd)
        print()

        bytesToSend = ConvertInputToBytes(cmd)
        time.sleep(0.04) # delay for wire to settle
        bus.write_i2c_block_data(slave_address, 0, bytesToSend)
        time_elapsed = time.time()-time_start


class PID:
    def __init__(self, kp, ki, kd, dim):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.e = np.zeros(dim)
        self.e_sum = np.zeros((dim, 1))
        self.e_k0 = np.array(dim)

        self.k0 = time.time()
        self.k1 = time.time()

    def run_pid(self, ref, measurement):
        """
        current state  motor speeds: w_r, w_l
        :param ref:
        :param measurement:
        :return:
        """
        self.k1 = time.time()

        u1_ref = ref[0]
        u2_ref = ref[1]
        u3_ref = ref[2]

        encod_r = measurement[0]
        encod_l = measurement[1]
        w_r = measurement[2]
        w_l = measurement[3]
        ultra_r = measurement[4]
        ultra_l = measurement[6]

        u1 = (w_r + w_l)/2
        u2 = encod_r - encod_l
        u3 = ultra_l - ultra_r

        e1 = u1_ref - u1
        e2 = u2_ref - u2    # swapped to see
        e3 = u3_ref - u3

        self.e = np.array([[e1, e2, e3]]).T
        print("Error Values:\n  Velocity: {0}\n  Encoders: {1}\n  Ultrason: {2}".format(e1, e2, e3))
        print()

        # Proportional ====================================
        u_p = np.squeeze(self.kp @ self.e)

        # Integral ========================================
        for e in self.e:
            if 1 < abs(e) < 1000:
                self.e_sum += e*(self.k1-self.k0)
        u_i = np.squeeze(self.ki @ self.e_sum)

        # Derivative ======================================
        e_dot = (self.e - self.e_k0)/(self.k1 - self.k0)
        self.e_k0 = self.e
        u_d = np.squeeze(self.kd @ e_dot)
        
        print("PID:\n {0}\n {1}\n {2}".format(u_p.T, u_i.T, u_d.T))
        print()
#         u1 = u_p[0] + u_i[0] + u_d[0]
#         u2 = u_p[1] + u_i[1] + u_d[1]
#         u3 = u_p[2] + u_i[2] + u_d[2]
        
#         u = [u1, u2 + u3]
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
