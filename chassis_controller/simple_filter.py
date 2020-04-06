import smbus
import time
import os
import RPi.GPIO as GPIO
import sys
from math import pi

sys.path.append('/home/pi/Documents/Police_Academy/sensors')
print(sys.path)
import sonar_measurements
# TODO: need to reorganize this so that the i2c line is not clogged up. Need to see if I can pass both encoder values at
# TODO: once so that there doesn't need to be such a long delay.

def main():
    # Tested to make sure values make sense
    # bus = smbus.SMBus(1)
    slave_address = 0x07        # Chassis Arduino

    test = Filter(1, slave_address)

    i = 0
    sonar_measurements.main()
    while i < 5:
        GPIO.setmode(GPIO.BCM)
        for trig, echo in zip(test.trig_pins, test.echo_pins):
            GPIO.setup(trig, GPIO.OUT)
            GPIO.setup(echo, GPIO.IN)
            GPIO.output(trig, False)
        time.sleep(1)
        omega = test.get_state_test()
        test.update_ultrasonic_values()
        print(omega)
        i += 1
    GPIO.cleanup()


class Filter:
    def __init__(self, bus, slave_address):
        self.bus = bus
        self.slave_address = slave_address
        self.encod_k0 = [0, 0]
        self.encod_k1 = [0, 0]
        self.ultra_k0 = [0, 0, 0]
        self.ultra_k1 = [0, 0, 0]
        self.k0 = 0
        self.k1 = self.k0
        
        self.trig_pins = [18, 20, 22]
        self.echo_pins = [19, 21, 23]
        self.units = 'in'
        self.cpr = 64

    def get_state(self):
        GPIO.setmode(GPIO.BCM)
        for trig, echo in zip(self.trig_pins, self.echo_pins):
            GPIO.setup(trig, GPIO.OUT)
            GPIO.setup(echo, GPIO.IN)
            GPIO.output(trig, False)
            
        self.update_encoder_values()
        omega = self.get_motor_velocities()
        self.update_ultrasonic_values()
        state = self.encod_k1 + omega + self.ultra_k1

        GPIO.cleanup()
        return state

    def get_state_test(self):
        self.update_encoder_values_test()
        omega = self.get_motor_velocities()
        return omega

    def get_motor_velocities(self):
        """
        State estimate here is simply the motor velocities
        """
        encod_dot = [(w_k1 - w_k0) / (self.k1 - self.k0) for w_k1, w_k0 in zip(self.encod_k1, self.encod_k0)]
        omega = [2*pi*e/self.cpr for e in encod_dot]
        return omega

    def update_encoder_values(self):
        """
        Requests two long integer values from the slave. i2c command 0 gets the right encoder, 1 gets the left encoder
        value.
        """

        # delay for line to settle
        time.sleep(0.05)
        self.k0 = self.k1
        self.k1 = time.time()

        # Read and convert encoder values
        data_bytes = self.bus.read_i2c_block_data(self.slave_address, 0, 8)
        data_int_r = self.bytes_to_int(data_bytes[0:3])
        data_int_l = self.bytes_to_int(data_bytes[4:7])

        # time.sleep(0.1)
        # self.k1[1] = time.time()
        # data_bytes_l = self.bus.read_i2c_block_data(self.slave_address, 1, 4)
        # print("encoder values: ")
        # print(data_bytes_r)
        # print(data_bytes_l)
        # data_int_r = self.bytes_to_int(data_bytes_r)
        # data_int_l = self.bytes_to_int(data_bytes_l)
        # print("encoder values: ")
        # print(data_int_r)
        # print(data_int_l)

        self.encod_k0 = self.encod_k1
        self.encod_k1 = [data_int_r, data_int_l]

    def update_encoder_values_test(self):
        self.k0 = self.k1
        self.k1 = time.time()

        self.encod_k0 = self.encod_k1
        self.encod_k1 = [x + 64 for x in self.encod_k1]

    def update_ultrasonic_values(self):
        self.ultra_k0 = self.ultra_k1
        array = []
        for trigger, echo in zip(self.trig_pins, self.echo_pins):
            meas = sonar_measurements.get_sonar_reading(trigger, echo, self.units)
            if meas > 20:
                meas = 4
            array.append(meas)
        
        self.ultra_k1 = array

    @staticmethod
    def bytes_to_int(bytes):
        result = 0
        bytes.reverse()
        for b in bytes:
            result = result * 256 + int(b)
        return result


if __name__ == '__main__':
    main()
