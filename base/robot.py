#!/usr/bin/python

import sys
import numpy as np 
import datetime
import time 
import math
import os

import numpy as np 
import matplotlib.pyplot as plt
import random



class Ultrasonic:
    def __init__(self,parent = None):
        
        distance = None
        trig_pin = None
        echo_pin = None
        units = None

    def update(self):
        if(type(distance) == 'float'):
            self.distance = 

    def get_val(self):
        if(type(distance) == 'float'):
            print( max(self.distance,0.0) )
            return self.distance

## End of Ultrasonic Class

class Encoder:
    def __init__(self,parent = None):

        val = None
        status = None

    def update(self):
        
        # Todo add the file location and update
        self.val = max ( int(######Add the fileval ##) , 0)

    def get_val(self):

        # Add the file 
        print("Encoder Value Returned")
        print(self.val)
        return self.val

# End of Encoder Class

# class Camera:
class Turret:
    def __init__(self,parent = None):

        camera_status = None

    def cam_status(self):
        camera_status = #############################
        print(camera_status)

## End of Camera Class


def odom(robot):
    def __init__(self, world_size = ):

        frame_id = "odom"
        child_frame_id = "base_link"
        translation_x = 0.0
        translation_y = 0.0
        translation_z = 0.0
        yaw = 0.0 #Using Degrees 

        #Robot Specs
        Dn = 101.6 #Distance in mm between wheels
        Ce = 64 #Encoder Resolution
        n =131.25 # Gear Ratio of the motor
        cm = Dn/(n*Ce) # Conversion factor of Encoder Pulses to Linear Wheel Displacement
        b = 254 # Wheel Base
        dU_left=0.0
        dU_right =0.0
        dU =0.0 
        d_angle=0.0
        encoder_left= 0.0
        encoder_right = 0.0
    
    def set_val(self):
        
        encoder_left = robot.motor_left.val
        encoder_right = robot.motor_right.val

        return encoder_left,encoder_right

    def update_odom(self):

        encoder_left,encoder_right = set_val()
        dU_left = cm * encoder_left
        dU_right = cm * encoder_right
        dU = (dU_left + dU_right)/2.0
        d_angle = (dU_right - dU_left)/b
        #Imp step update
        angle += d_angle

        translation_x = translation_x + ( dU * cos(angle + d_angle/2)*2 )
        translation_y = translation_y + ( dU * sin(angle + d_angle/2)*2 )
        yaw = angle

    def print_odom(self):

        status = [translation_x,translation_y,yaw]
        return status

## End of Odom Class 

class Twist_vel:
    def __init__(self,parent = None):

        vel_left = None
        vel_right = None

## End of Twist_vel  class

class robot(Encoder,Ultrasonic):

    def __init__(self):
        
        # Pose Status of the robot

        #Ultrasonics
        US_left = Ultrasonic()
        US_left.distance = None

        US_right = Ultrasonic()
        US_right.distance = None

        US_front = Ultrasonic()
        US_front.distance = None

        #Motor Encoders
        motor_left = Encoder()
        motor_left.val = None

        motor_right = Encoder()
        motor_right.val = None

        #Setup Odom 
        odom = Odom()

    def pose(self):
        currentDT = datetime.datetime.now()
        print (str(currentDT))
        pose_status = odom.print_odom()
        print(" Translation X:")
        print(pose_status[0])
        print(" Translation Y:")
        print(pose_status[1])
        print(" Yaw:")
        print(pose_status[2])

    


## End of Robot class 
        






        




