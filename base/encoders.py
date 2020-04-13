#!/usr/bin/env python3
from math import pi
import smbus
import time


def main():
    print_header()
    run_test()
    return 0


def print_header():
    print("Encoder sensor module")


def run_test():
    bus = smbus.SMBus(1)
    slave_address = 0x07

    left_value, right_value = get_encoder_values(bus, slave_address)
    print("Left Encoder Reading:  {}".format(left_value))
    print("Right Encoder Reading: {}".format(right_value))

    leftEncoder = Encoder('left', 100, bus, slave_address)
    rightEncoder = Encoder('right', 100, bus, slave_address)
    driveTrain = DriveTrain(leftEncoder, rightEncoder, bus, slave_address)

    leftEncoder.update()
    leftEncoder.print_value()

    rightEncoder.update()
    rightEncoder.print_value()

    driveTrain.update()
    leftEncoder.print_value()
    rightEncoder.print_value()


class Encoder:
    def __init__(self, location, cpr, bus, slave_address):
        self.location = location
        self.cpr = cpr
        self.bus = bus
        self.slave_address = slave_address

        self.position = 0.0
        self.previous_position = 0.0
        self.velocity = 0.0
        self.k0 = 0.0
        self.k1 = 0.0

        self.parent = None

    def update(self, value=None):
        self.k0 = self.k1
        self.k1 = time.time()

        self.previous_position = self.position

        if value:
            self.position = value

        else:
            self.position = get_encoder_values(self.bus, self.slave_address, self.location)

        self.compute_velocity()

    def get_value(self):
        return self.position

    def get_position(self):
        return self.position

    def get_previous_position(self):
        return self.previous_position

    def get_velocity(self):
        return self.velocity

    def set_parent(self, parent):
        self.parent = parent

    def print_value(self):
        print("Position: ".format(self.position))
        print("Velocity: ".format(self.veloicty))

    def compute_velocity(self):
        e_dot = (self.position - self.previous_position)/(self.k1 - self.k0)
        self.velocity = 2*pi*e_dot/self.cpr


class DriveTrain:
    def __init__(self, enc_left, enc_right, bus, slave_address):
        self.enc_left = enc_left
        self.enc_right = enc_right
        self.bus = bus
        self.slave_address = slave_address

        enc_left.set_parent(self)
        enc_right.set_parent(self)

    def update(self):
        left_value, right_value = get_encoder_values(self.bus, self.slave_address)
        self.enc_left.update(left_value)
        self.enc_right.update(right_value)


def get_encoder_values(bus, slave_address, location=None):
    data_bytes = bus.read_i2c_block_data(slave_address, 0, 8)
    data_int_r = bytes_to_int(data_bytes[0:3])
    data_int_l = bytes_to_int(data_bytes[4:7])

    if location == 'left':
        return data_int_l

    elif location == 'right':
        return data_int_r

    else:
        return data_int_l, data_int_r


def bytes_to_int(data):
    return int.from_bytes(data, byteorder='little', signed=True)


if __name__ == "__main__":
    main()
