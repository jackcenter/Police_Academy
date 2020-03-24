import smbus
import time
import os

from sensors import sonar_measurements

def main():
    # display system info
    print(os.uname())
    bus = smbus.SMBus(1)

    # I2C address of Arduino Slave
    slave_address = 0x07
    i2c_cmd = 0x01

    os.system("python sensors/sonar_measurements.py")
    # loop to send message
    exit = False

    while not exit:
        r = input('Enter something, "q" to quit"')
        print(r)
        sonar_measurements

        bytesToSend = ConvertStringToBytes(r)
        bus.write_i2c_block_data(slave_address, i2c_cmd, bytesToSend)

        if r == 'q':
            exit = True


def ConvertStringToBytes(src):
    converted = []
    for b in src:
        converted.append(ord(b))
    return converted


if __name__ == '__main__':
    main()

