import os
import RPi.GPIO as GPIO
import smbus
import time

from sensors import sonar_measurements as sonar


def main():
    bus = smbus.SMBus(1)
    slave_address = 0x07
    i2c_cmd = 0x01
    exit_program = False
    trig_pins, echo_pins, units = sonar.setup()

    while not exit_program:
        r = input('Enter something, "q" to quit"')
        print(r)

        bytesToSend = ConvertStringToBytes(r)
        bus.write_i2c_block_data(slave_address, i2c_cmd, bytesToSend)

    distances = sonar.get_sonar_readings(trig_pins, echo_pins, units)
    print("Measurements: {0}, {1}, {2}".format(distances[0], distances[1], distances[2]))
    GPIO.cleanup()


def ConvertStringToBytes(src):
    converted = []
    for b in src:
        converted.append(ord(b))
    return converted


if __name__ == '__main__':
    main()
