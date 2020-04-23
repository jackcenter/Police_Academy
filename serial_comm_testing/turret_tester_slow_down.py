# -*- coding: utf-8 -*-
"""
Created on Sat Apr 18 15:01:42 2020

@author: chadd

File Status:
    
    The get_turret status worked well in the previous version(s) of this code.
    However, I've not tested it since adding the pit_break and rot_break variables,
    although this functionality should already be incorporated into the Arduino 
    turret code.
    
    The command list syntax is as follows:
    tot_cmd = [fire, rot_on, rot_dir, rot_steps, rot_delay, pit_on, pit_dir, pit_steps, pit_delay]
    ALL ENTRIES MUST BE BETWEEN 0 and 255
    
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
             
    rot_steps = 0 or #steps/10
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
    
    pit_steps = 0 or #steps/10
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
        rot_break = 0
        pit_break = 0
    return data_int_rot, data_int_pit, data_int_servo, pit_break, rot_break


rot_steps_from_home = 0
pit_steps_from_home = 0
servo_pulls = 0
rot_break = 0
pit_break = 0



slave_address = 0x08
arduino_data_size = 20
bus = smbus.SMBus(1)

fire = 0
rot_on = 1
rot_dir = 1
rot_steps = 0
rot_delay = 210
pit_on = 1
pit_dir = 1
pit_steps = 0
pit_delay = 210


# slowdown and fire
for i in range(7):
    if i == 6:
        fire = 1
        rot_on = 0
        pit_on = 0
        
    tot_cmd = [fire, rot_on, rot_dir, rot_steps, rot_delay, pit_on, pit_dir, pit_steps, pit_delay]
    print("sending: ")
    print(tot_cmd)
    send_command(bus, slave_address, tot_cmd)
    print("\n")

    rot_steps_from_home, pit_steps_from_home, servo_pulls, rot_break, pit_break = get_turret_status(bus, slave_address, arduino_data_size)

    
    print("rot_steps = ")
    print(rot_steps_from_home)
    print("    pit_steps = ")
    print(pit_steps_from_home)
    print("    servo_pulls = ")
    print(servo_pulls)
    print("    rot_break = ")
    print(rot_break)
    print("    pit_break = ")
    print(pit_break)
    
    if rot_break == 1:
        break
    if pit_break == 1:
        break
    
    

    
    rot_delay = rot_delay + 5
    pit_delay = pit_delay + 5
    time.sleep(0.5)
    
    
fire = 0
rot_on = 1
rot_dir = 0
rot_steps = 0
rot_delay = 250
pit_on = 1
pit_dir = 0
pit_steps = 0
pit_delay = 250


time.sleep(10)




# speed up and fire
for i in range(7):
    if i == 6:
        fire = 1
        rot_on = 0
        pit_on = 0
    
    tot_cmd = [fire, rot_on, rot_dir, rot_steps, rot_delay, pit_on, pit_dir, pit_steps, pit_delay]
    print("sending: ")
    print(tot_cmd)
    send_command(bus, slave_address, tot_cmd)
    print("\n")

    rot_steps_from_home, pit_steps_from_home, servo_pulls, rot_break, pit_break = get_turret_status(bus, slave_address, arduino_data_size)

    
    print("rot_steps = ")
    print(rot_steps_from_home)
    print("    pit_steps = ")
    print(pit_steps_from_home)
    print("    servo_pulls = ")
    print(servo_pulls)
    print("    rot_break = ")
    print(rot_break)
    print("    pit_break = ")
    print(pit_break)
    
    if rot_break == 1:
        break
    if pit_break == 1:
        break
    
    
    rot_delay = rot_delay - 5
    pit_delay = pit_delay - 5
    time.sleep(0.5)
    





    

    