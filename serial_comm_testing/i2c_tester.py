# -*- coding: utf-8 -*-
"""
Created on Sat Apr 18 15:01:42 2020

@author: chadd
"""
import smbus
import time
# i2c tester 

def send_command(bus, slave_address, command):
    bus.write_i2c_block_data(slave_address, 0, command)
    return 
#removed try except catcher here

def bytes_to_int(data):
    return int.from_bytes(data, byteorder='little', signed=True)

def get_turret_status(bus, slave_address, num_bytes):
    try:
        data_bytes = bus.read_i2c_block_data(slave_address, 0, num_bytes)
        print("data_bytes = ")
        print(data_bytes)
        data_int_rot = bytes_to_int(data_bytes[0:3])
        data_int_pit = bytes_to_int(data_bytes[4:7])
        data_int_servo = bytes_to_int(data_bytes[8:11])

    except OSError:
        print("ERROR: bus didn't respond")
        data_int_rot = 0
        data_int_pit = 0
        data_int_servo = 0;

    return data_int_rot, data_int_pit, data_int_servo




slave_address = 0x08
arduino_data_size = 12
bus = smbus.SMBus(1)



tot_cmd = [0, 1, 1, 0, 200, 1, 1, 0, 200]
print(tot_cmd)
#total_cmd_bytes = [a.to_bytes(1, 'big') for a in tot_cmd]  # the size 2 in to_bytes is the size of integers up to 30000, so this should use 16 bytes
#print(total_cmd_bytes)
send_command(bus, slave_address, tot_cmd)

time.sleep(1)

rot_steps_from_home, pit_steps_from_home, servo_pulls = get_turret_status(bus, slave_address, arduino_data_size)

print(rot_steps_from_home)
print(pit_steps_from_home)
print(servo_pulls)



    

    