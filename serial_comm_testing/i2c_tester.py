# -*- coding: utf-8 -*-
"""
Created on Sat Apr 18 15:01:42 2020

@author: chadd
"""
import smbus

# i2c tester 

def send_command(bus, slave_address, command):
    bus.write_i2c_block_data(slave_address, 0, command)
    return 
#removed try except catcher here



def read_data(bus, slave_address, data_size):
    try:
        data_bytes = bus.read_i2c_block_data(slave_address, 0, data_size)
        return data_bytes

    except OSError:
        print("ERROR: bus didn't respond")
        return None



slave_address = 0x08
arduino_data_size = 16
bus = smbus.SMBus(1)



tot_cmd = [0, 1, 1, 0, 20000, 1, 1, 0, 20000]
print(tot_cmd)
#total_cmd_bytes = [a.to_bytes(2, 'big') for a in tot_cmd]  # the size 2 in to_bytes is the size of integers up to 30000, so this should use 16 bytes
#print(total_cmd_bytes)
send_command(bus, slave_address, tot_cmd)
received_data = read_data(bus, slave_address, arduino_data_size)
if received_data is not None:
    str_received_data = [a.decode("utf-8") for a in received_data]
    print(str_received_data)
    

    