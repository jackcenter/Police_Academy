# -*- coding: utf-8 -*-
"""
Created on Sat Apr 18 15:01:42 2020

@author: chadd

File Status:
    
    This file is for rotating and pitching the turret some specified number of degrees
    
    
    
    
    The command list syntax is as follows:
    tot_cmd = [fire, rot_on, rot_dir, rot_steps, rot_delay, pit_on, pit_dir, pit_steps, pit_delay]
    
    ALL ENTRIES MUST BE INTEGERS (of int type) BETWEEN 0 and 255
    
    fire = 0 or 1
        0 means no fire
        1 means FIRE NOW
        
    rot_on = 0 or 1 or 2
        0 means turn off rotation
        1 means turn on rotation
        2 means start homing process for rotation 
        
    rot_dir = 0 or 1
        Direction is as yet untested, so this assumption may need to be switched
        Current assumption is that if the viewer is standing on top of the turret,
        a positive rotation will be a right turn, while a negative rotation will 
        be a left turn:
             0 means negative rotation direction (negative velocity given by camera)
             1 means positive rotation direction (positive velocity given by camera)
             
    rot_steps = 0 or #steps/10 < 255
        0 means do not control according to steps, i.e. normal ops = velocity commands
        #steps/10 means control as if push button was performed.  Turret actuates 
            10 times the number of steps entered.  Approx 800 steps = 360 degree rotation
            
    rot_delay = 0 to 255 = microseconds of delay/100
        The targeting file will keep this value between 50 and 250, i.e. the delay
        will be between 5000 and 25000.  The smaller the delay, the faster the turret moves.
            THIS MAY NEED TO BE SLOWED DOWN TO DELAY/1000 AND RANGE ADJUSTED TO 10-40 or so 
                (or the rotation PID could be more accurately tuned in the targeting file)
    
    pit_on = 0 or 1 or 2
        0 means turn off pitch
        1 means turn on pitch
        2 means start homing process for pitch
        
    pit_dir = 0 or 1
        Direction is as yet untested, so this assumption may need to be switched
        Current assumption is that if the viewer is standing on top of the turret,
        a positive pitch will be up, while a negative pitch will be down:
             0 means negative pitch direction (negative velocity given by camera)
             1 means positive pitch direction (positive velocity given by camera)
    
    pit_steps = 0 or #steps/10 < 255
        0 means do not control according to steps, i.e. normal ops = velocity commands
        #steps/10 means control as if push button was performed.  Turret actuates 
            10 times the number of steps entered.  Approx ??? steps pitch
            
    pit_delay = 0 to 255 = microseconds of delay/100
        The targeting file will keep this value between 50 and 250, i.e. the delay
        will be between 5000 and 25000 us.  The smaller the delay, the faster the turret moves.
            THIS MAY NEED TO BE SLOWED DOWN TO DELAY/1000 AND RANGE ADJUSTED TO 10-40 or so 
                (or the pitch PID could be more accurately tuned in the targeting file)
                
                
    
    When pitching with number of steps, end stop switches simply stop the actuation on the hardware level.
    When rotating with steps, after +600 or -600 steps from home, turret stops at hardware level.
    When in velocity command mode, turret will have to stop in upper software.
    
    
    The get_turret_status gets 5 signed long variables:
        
        rot and pitch steps from home = the positive or negative number of steps 
            that the steppers have incremented since the home function was last called
        
        servo pulls = number of times the servo has been completely actuated (# fires)
        
        pit and rot break = 0 or 1 
            0 means has not hit end stop switches or rotated too far
            1 means that it has ^
                right now this just stops things dead, but maybe we should just try to fire ... ?
        


"""
import smbus
import time
from math import floor
# i2c tester 
# TODO turn pitch steps into pitch degrees

def send_command(bus, slave_address, command):
    bus.write_i2c_block_data(slave_address, 0, command)
    return 
#removed try except catcher here

def bytes_to_int(data):
    return int.from_bytes(data, byteorder='little', signed=True)

def get_turret_status(bus, slave_address, num_bytes):
    #TODO recently added rot and pit break, untested addition
    try:
        data_bytes = bus.read_i2c_block_data(slave_address, 0, num_bytes)
        print("data_bytes = ")
        print(data_bytes)
        data_int_rot    = bytes_to_int(data_bytes[0:3])
        data_int_pit    = bytes_to_int(data_bytes[4:7])
        data_int_servo  = bytes_to_int(data_bytes[8:11])
        rot_break       = bytes_to_int(data_bytes[12:15])
        pit_break       = bytes_to_int(data_bytes[16:19])

    except OSError:
        print("ERROR: bus didn't respond")
        data_int_rot = 0
        data_int_pit = 0
        data_int_servo = 0
        rot_break = None
        pit_break = None
    return data_int_rot, data_int_pit, data_int_servo, pit_break, rot_break


def send_home_cmd():
    slave_address     = 0x08
    arduino_data_size = 20
    bus = smbus.SMBus(1)
    
    # 800 steps (80 steps_div10) ~ 1 full rotation 360 deg

    
    
    fire      = 0
    rot_on    = 2
    rot_steps = 0
    rot_delay = 250
    rot_dir   = 0
        
    pit_on    = 2
    pit_dir   = 0
    pit_steps = 0
    pit_delay = 250
    
    
    tot_cmd = [fire, rot_on, rot_dir, rot_steps, rot_delay, pit_on, pit_dir, pit_steps, pit_delay]
    print("Sent command list: ")
    print(tot_cmd)
    send_command(bus, slave_address, tot_cmd)
    rot_steps_from_home, pit_steps_from_home, servo_pulls, rot_break, pit_break = get_turret_status(bus, slave_address, arduino_data_size)
    print("Received information:")
    print("rot_steps = "   + str(rot_steps_from_home))
    print("pit_steps = "   + str(pit_steps_from_home))
    print("servo_pulls = " + str(servo_pulls))
    print("rot_break = "   + str(rot_break))
    print("pit_break = "   + str(pit_break))
    print("... ... ... sleeping ... ... ...")
    time.sleep(15)
    rot_steps_from_home, pit_steps_from_home, servo_pulls, rot_break, pit_break = get_turret_status(bus, slave_address, arduino_data_size)
    print("Received information:")
    print("rot_steps = "   + str(rot_steps_from_home))
    print("pit_steps = "   + str(pit_steps_from_home))
    print("servo_pulls = " + str(servo_pulls))
    print("rot_break = "   + str(rot_break))
    print("pit_break = "   + str(pit_break))




def send_fire_cmd():
    slave_address     = 0x08
    arduino_data_size = 20
    bus = smbus.SMBus(1)

    
    fire      = 1
    rot_on    = 0
    rot_steps = 0
    rot_delay = 250
    rot_dir   = 0
        
    pit_on    = 0
    pit_dir   = 0
    pit_steps = 0
    pit_delay = 250
    
    
    tot_cmd = [fire, rot_on, rot_dir, rot_steps, rot_delay, pit_on, pit_dir, pit_steps, pit_delay]
    print("Sent command list: ")
    print(tot_cmd)
    send_command(bus, slave_address, tot_cmd)
    rot_steps_from_home, pit_steps_from_home, servo_pulls, rot_break, pit_break = get_turret_status(bus, slave_address, arduino_data_size)
    print("Received information:")
    print("rot_steps = "   + str(rot_steps_from_home))
    print("pit_steps = "   + str(pit_steps_from_home))
    print("servo_pulls = " + str(servo_pulls))
    print("rot_break = "   + str(rot_break))
    print("pit_break = "   + str(pit_break))
    print("... ... ... sleeping ... ... ...")
    time.sleep(6)
    rot_steps_from_home, pit_steps_from_home, servo_pulls, rot_break, pit_break = get_turret_status(bus, slave_address, arduino_data_size)
    print("Received information:")
    print("rot_steps = "   + str(rot_steps_from_home))
    print("pit_steps = "   + str(pit_steps_from_home))
    print("servo_pulls = " + str(servo_pulls))
    print("rot_break = "   + str(rot_break))
    print("pit_break = "   + str(pit_break))






def send_rot_turn_cmd(degrees, delay_div100):
    # break end stop switches should happen in the arduino code
    # note that the steps input are the actual steps that the arduino will perform divided by 10
    # note that the delay input is the actual microsecond delay divided by 100
    slave_address     = 0x08
    arduino_data_size = 20
    bus = smbus.SMBus(1)
    
    degrees = int(degrees)
    delay_div100 = int(delay_div100)
    
    # 800 steps (80 steps_div10) ~ 1 full rotation 360 deg
    steps_div10 = round((80.0/360.0)*degrees)
    
    
    fire      = 0
    rot_on    = 1 
    rot_steps = abs(steps_div10)
    rot_delay = delay_div100
    
    if steps_div10 < 0:
        rot_dir = 0
    else:
        rot_dir = 1
        
    pit_on    = 0
    pit_dir   = 0
    pit_steps = 0
    pit_delay = delay_div100
    
    
    tot_cmd = [fire, rot_on, rot_dir, rot_steps, rot_delay, pit_on, pit_dir, pit_steps, pit_delay]
    print("Sent command list: ")
    print(tot_cmd)
    send_command(bus, slave_address, tot_cmd)
    rot_steps_from_home, pit_steps_from_home, servo_pulls, rot_break, pit_break = get_turret_status(bus, slave_address, arduino_data_size)
    print("Received information:")
    print("rot_steps = "   + str(rot_steps_from_home))
    print("pit_steps = "   + str(pit_steps_from_home))
    print("servo_pulls = " + str(servo_pulls))
    print("rot_break = "   + str(rot_break))
    print("pit_break = "   + str(pit_break))
    print("... ... ... sleeping ... ... ...")
    time.sleep(6)
    rot_steps_from_home, pit_steps_from_home, servo_pulls, rot_break, pit_break = get_turret_status(bus, slave_address, arduino_data_size)
    print("Received information:")
    print("rot_steps = "   + str(rot_steps_from_home))
    print("pit_steps = "   + str(pit_steps_from_home))
    print("servo_pulls = " + str(servo_pulls))
    print("rot_break = "   + str(rot_break))
    print("pit_break = "   + str(pit_break))




def send_pit_turn_cmd(steps_div10, delay_div100):
    # break end stop switches should happen in the arduino code
    # note that the steps input are the actual steps that the arduino will perform divided by 10
    # note that the delay input is the actual microsecond delay divided by 100
    slave_address     = 0x08
    arduino_data_size = 20
    bus = smbus.SMBus(1)
    
    steps_div10  = int(steps_div10)
    delay_div100 = int(delay_div100)
    
    fire      = 0
    pit_on    = 1 
    pit_steps = abs(steps_div10)
    pit_delay = delay_div100
    
    if steps_div10 < 0:
        pit_dir = 0
    else:
        pit_dir = 1
        
    rot_on    = 0
    rot_dir   = 0
    rot_steps = 0
    rot_delay = delay_div100
    
    
    tot_cmd = [fire, rot_on, rot_dir, rot_steps, rot_delay, pit_on, pit_dir, pit_steps, pit_delay]
    print("Sent command list: ")
    print(tot_cmd)
    send_command(bus, slave_address, tot_cmd)
    rot_steps_from_home, pit_steps_from_home, servo_pulls, rot_break, pit_break = get_turret_status(bus, slave_address, arduino_data_size)
    print("Received information:")
    print("rot_steps = "   + str(rot_steps_from_home))
    print("pit_steps = "   + str(pit_steps_from_home))
    print("servo_pulls = " + str(servo_pulls))
    print("rot_break = "   + str(rot_break))
    print("pit_break = "   + str(pit_break))
    print("... ... ... sleeping ... ... ...")
    time.sleep(6)
    rot_steps_from_home, pit_steps_from_home, servo_pulls, rot_break, pit_break = get_turret_status(bus, slave_address, arduino_data_size)
    print("Received information:")
    print("rot_steps = "   + str(rot_steps_from_home))
    print("pit_steps = "   + str(pit_steps_from_home))
    print("servo_pulls = " + str(servo_pulls))
    print("rot_break = "   + str(rot_break))
    print("pit_break = "   + str(pit_break))

    
    
