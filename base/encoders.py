#!/usr/bin/env python3
import serial
import time


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


def get_encoder_value(location: str, serial_address: str):
    cmd = location + '\n'
    serial_bus = serial.Serial(serial_address, 9600, timeout=1)
    serial_bus.flush()

    serial_bus.write(cmd.encode('utf-8'))

    if serial_bus.in_waiting > 0:
        line = serial_bus.readline().decode('utf-8').rstrip()
        return line

    return 1
