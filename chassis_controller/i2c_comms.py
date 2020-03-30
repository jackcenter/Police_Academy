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

def read_from_slave(address, command, i2cbus):
    data = ""
    result = ""
    try:
        # Read an entire 32-byte data block from the slave.
        data = i2cbus.read_i2c_block_data(address, command)

        # Since the data block can contain any character but a 255 means no
        # string character, we'll convert the series of bytes into a Python-
        # compatible string, char-by-char.
        index = 0
        while (data[index] != 255):
            result += chr(data[index])
            index += 1

        print(result)
        return result

    except:
        pass
def bytes_to_int(bytes):
    result = 0
    bytes.reverse()
    for b in bytes:
        result = result * 256 + int(b)
    return result

# loop to send message
exit = False
while not exit:
    r = input('Enter something, "q" to quit"')
    print(r)

    bytesToSend = ConvertStringToBytes(r)
    bus.write_i2c_block_data(slave_address, i2c_cmd, bytesToSend)
    data_bytes = bus.read_i2c_block_data(slave_address, 0, 4)
    data_int = bytes_to_int(data_bytes) 
    print(data_int)

    if r == 'q':
        exit = True