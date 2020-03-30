import smbus
import time
import os


def main():
    bus = smbus.SMBus(1)
    slave_address = 0x07        # Chassis Arduino
    i2c_cmd = 0x01

    working = True
    while working:
        r = input('Enter something, "q" to quit"')
        print(r)

        data_bytes = bus.read_i2c_block_data(slave_address, 0, 4)
        data_int = bytes_to_int(data_bytes)
        print(data_int)

    if r == 'q':
        working = False

def get_encoder_values():
    data_bytes = bus.read_i2c_block_data(slave_address, 0, 4)
    data_int = bytes_to_int(data_bytes)
    return data_int

def bytes_to_int(bytes):
    result = 0
    bytes.reverse()
    for b in bytes:
        result = result * 256 + int(b)
    return result


if __name__ == '__main__':
