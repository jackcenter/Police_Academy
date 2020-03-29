import smbus
import time
import os

# display system info
print(os.uname())

bus = smbus.SMBus(1)

# I2C address of Arduino Slave
slave_address = 0x07
i2c_cmd = 0x01

def ConvertStringToBytes(src):
    converted = []
    for b in src:
        converted.append(ord(b))
    return converted


# loop to send message
exit = False
while not exit:
    r = input('Enter something, "q" to quit"')
    print(r)

    bytesToSend = ConvertStringToBytes(r)
    bus.write_i2c_block_data(slave_address, i2c_cmd, bytesToSend)
    bus.read_i2c_block_data(slave_address, i2c_cmd, bytesToSend)

    if r == 'q':
        exit = True