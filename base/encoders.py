#!/usr/bin/env python3
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


class Encoder:
    def __init__(self, location, serial_address):
        self.location = location
        self.serial_address = serial_address

        self.position = 0.0
        self.previous_position = 0.0
        self.velocity = 0.0
        self.k0 = 0.0
        self.k1 = 0.0

    def update(self):
        self.k0 = self.k1
        self.k1 = time.time()

        self.previous_position = self.position
        self.position = get_encoder_values(self.location, self.serial_address)

    def get_value(self):
        return self.position

    def get_position(self):
        return self.position

    def get_previous_position(self):
        return self.previous_position

    def get_velocity(self):
        return self.velocity


def get_encoder_values(bus, slave_address):
    data_bytes = bus.read_i2c_block_data(slave_address, 0, 8)
    data_int_r = bytes_to_int(data_bytes[0:3])
    data_int_l = bytes_to_int(data_bytes[4:7])

    return data_int_l, data_int_r


def bytes_to_int(data):
    result = 0
    # bytes.reverse()
    return int.from_bytes(data, byteorder='little', signed=True)
    # for b in bytes:
    #     result = result * 256 + int(b)
    # return result


if __name__ == "__main__":
    main()
