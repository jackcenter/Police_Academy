# -*- coding: utf-8 -*-
"""
Created on Thu Apr 23 21:45:27 2020

@author: chadd

Test file for turret

Sending individual rotation and pitch commands

"""
from turret_commands import send_rot_turn_cmd
from turret_commands import send_pit_turn_cmd
from turret_commands import send_home_cmd
from turret_commands import send_fire_cmd

# TODO turn pitch steps into pitch degrees
trash = input("Homing on your command!")
send_home_cmd()
# trash = input("Firing on your command!")
# send_fire_cmd()


# print("Sending rotation command: ")
# rdeg = input("Please enter the rotation amount in degrees (-270 < angle < 270): ")
# rdelay = input("Please enter the speed/delay you would like to rotate at (50 < delay < 250).  Smaller number = faster speed:  ")
# 
# send_rot_turn_cmd(rdeg, rdelay)


print("Sending pitch command: ")
psteps = input("Please enter the pitch amount in steps (TODO): ")
pdelay = input("Please enter the speed/delay you would like to pitch at (50 < delay < 250).  Smaller number = faster speed:  ")

send_pit_turn_cmd(psteps, pdelay)
