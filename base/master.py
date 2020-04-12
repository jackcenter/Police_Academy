#!/usr/bin/python3.3

#RULES FOR Using this code
# 1) Dont mess with QT class
# 2) Comment the code if you are using the any Global variables and if they can have the same name as local variables 
# 3) 



import sys
import numpy as np 

import time 
import math
import os

import threading 

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *


#UNCOMMENT THE FOLLOWING ON RASPBERRY PI 
# import smbus 
# import RPi.GPIO as GPIO  
# import simple_pid 
# from pid_control import * 

# Jack's Stuff
base_folder = os.path.dirname(__file__)



# JACK's STUFF 

# Scott Stuff

# Chadd Stuff

#Global Variable
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

# PyQT class minimized for better code readability
class RobotMonitorWidget(QWidget): 
    
    def __init__(self,parent = None):
        super(RobotMonitorWidget,self).__init__()
        layout = QVBoxLayout()



        #####################################
        #Ultrasonic Group 
        USGroupLayout = QHBoxLayout()
        USGroup = QGroupBox('Ultrasonic')
        
        #left
        boxLayout = QHBoxLayout()
        groupBox = QGroupBox('Left Ultrasonic')
        self.US_Left = QPushButton('Calculate')
        self.US_Left.clicked.connect(self.US_Left_onclick) 
        boxLayout.addWidget(self.US_Left)
        self.US_Left_status = QPushButton('Status')
        self.US_Left_status.clicked.connect(self.US_Left_onclick_status) 
        boxLayout.addWidget(self.US_Left_status)    
        
        groupBox.setLayout(boxLayout) #Setting Horizontal Layout for Left Ultrasonic
        USGroupLayout.addWidget(groupBox) #Addind the Widget to Ultrsonic Group

        #right
        boxLayout = QHBoxLayout()
        groupBox = QGroupBox('Right Ultrasonic')
        self.US_Right = QPushButton('Calculate')
        self.US_Right.clicked.connect(self.US_Right_onclick) 
        boxLayout.addWidget(self.US_Right)
        self.US_Right_status = QPushButton('Status')
        self.US_Right_status.clicked.connect(self.US_Right_onclick_status) 
        boxLayout.addWidget(self.US_Right_status)
        
        groupBox.setLayout(boxLayout) #Setting Horizontal Layout for Left Ultrasonic
        USGroupLayout.addWidget(groupBox) #Addind the Widget to Ultrsonic Group
        
        #Front
        boxLayout = QHBoxLayout()
        groupBox = QGroupBox('Front Ultrasonic')
        self.US_Front = QPushButton('Calculate')
        self.US_Front.clicked.connect(self.US_Front_onclick) 
        boxLayout.addWidget(self.US_Front)
        self.US_Front_status = QPushButton('Status')
        self.US_Front_status.clicked.connect(self.US_Right_onclick_status) 
        boxLayout.addWidget(self.US_Front_status)
        
        groupBox.setLayout(boxLayout) #Setting Horizontal Layout for Left Ultrasonic
        USGroupLayout.addWidget(groupBox) #Addind the Widget to Ultrsonic Group
        
        #End of Ultrasonic Widget
        USGroup.setLayout(USGroupLayout)
        layout.addWidget(USGroup)
        ##########################################

        #Motor Group 
        MotorGroupLayout = QHBoxLayout()
        MotorGroup = QGroupBox('Motor')
        
        #left
        boxLayout = QHBoxLayout()
        groupBox = QGroupBox('Left Motor')
        self.enc_Left = QPushButton('Encoder')
        self.enc_Left.clicked.connect(self.enc_Left_onclick) 
        boxLayout.addWidget(self.enc_Left)
        self.enc_Left_status = QPushButton('Status')
        self.enc_Left_status.clicked.connect(self.enc_Left_onclick_status) 
        boxLayout.addWidget(self.enc_Left_status)    
        groupBox.setLayout(boxLayout) #Setting Horizontal Layout for Left Ultrasonic
        MotorGroupLayout.addWidget(groupBox) #Addind the Widget to Ultrsonic Group

        #right
        boxLayout = QHBoxLayout()
        groupBox = QGroupBox('Right Motor')
        self.enc_Right = QPushButton('Enocoder')
        self.enc_Right.clicked.connect(self.enc_Right_onclick) 
        boxLayout.addWidget(self.enc_Right)
        self.enc_Right_status = QPushButton('Status')
        self.enc_Right_status.clicked.connect(self.enc_Right_onclick_status) 
        boxLayout.addWidget(self.enc_Right_status)
        
        groupBox.setLayout(boxLayout) #Setting Horizontal Layout for Left Ultrasonic
        MotorGroupLayout.addWidget(groupBox) #Addind the Widget to Ultrsonic Group
        
        #End of Motor Widget
        MotorGroup.setLayout(MotorGroupLayout)
        layout.addWidget(MotorGroup)
       

        #Turret Group 
        TurretGroupLayout = QHBoxLayout()
        TurretGroup = QGroupBox('Turret')
        
        #Stepper Motor
        boxLayout = QVBoxLayout()
        groupBox = QGroupBox('Stepper Motor')
        self.enc_stepper = QPushButton('Encoder Count')
        self.enc_stepper.clicked.connect(self.enc_stepper_onclick) 
        boxLayout.addWidget(self.enc_stepper)
        self.enc_stepper_status = QPushButton('Status')
        self.enc_stepper_status.clicked.connect(self.enc_stepper_onclick_status) 
        boxLayout.addWidget(self.enc_stepper_status)    
        groupBox.setLayout(boxLayout) #Setting Horizontal Layout for Left Ultrasonic
        TurretGroupLayout.addWidget(groupBox) #Addind the Widget to Ultrsonic Group

        #Servo Motor
        boxLayout = QVBoxLayout()
        groupBox = QGroupBox('Servo')
        self.enc_servo = QPushButton('Servo Count')
        self.enc_servo.clicked.connect(self.enc_servo_onclick) 
        boxLayout.addWidget(self.enc_servo)
        self.enc_servo_status = QPushButton('Status')
        self.enc_servo_status.clicked.connect(self.enc_servo_onclick_status) 
        boxLayout.addWidget(self.enc_servo_status)
        
        groupBox.setLayout(boxLayout) #Setting Horizontal Layout for Left Ultrasonic
        TurretGroupLayout.addWidget(groupBox) #Addind the Widget to Ultrsonic Group
        
        #End of Turret Widget
        TurretGroup.setLayout(TurretGroupLayout)
        layout.addWidget(TurretGroup)
       
        
        #####################################
        #LAUNCH Group 
        LaunchGroupLayout = QHBoxLayout()
        LaunchGroup = QGroupBox('LAUNCH')
        
        #Odom
        boxLayout = QHBoxLayout()
        groupBox = QGroupBox('Odometry')
        self.odome = QPushButton('ODOM')
        self.odome.clicked.connect(self.odome_onclick) 
        boxLayout.addWidget(self.odome)
        groupBox.setLayout(boxLayout) 
        LaunchGroupLayout.addWidget(groupBox) 

        #Begin moving and PID
        boxLayout = QHBoxLayout()
        groupBox = QGroupBox('GO GO GO')
        self.go = QPushButton('Begin Movement')
        self.go.clicked.connect(self.go_onclick) 
        boxLayout.addWidget(self.go)
        
        groupBox.setLayout(boxLayout) #Setting Horizontal Layout for Left Ultrasonic
        LaunchGroupLayout.addWidget(groupBox) #Addind the Widget to Ultrsonic Group
        
        #Pose
        boxLayout = QHBoxLayout()
        groupBox = QGroupBox('Pose')
        self.pose = QPushButton('Pose Status')
        self.pose.clicked.connect(self.pose_onclick) 
        boxLayout.addWidget(self.pose)
        
        groupBox.setLayout(boxLayout) #Setting Horizontal Layout for Left Ultrasonic
        LaunchGroupLayout.addWidget(groupBox) #Addind the Widget to Ultrsonic Group
        
        #End of Ultrasonic Widget
        LaunchGroup.setLayout(LaunchGroupLayout)
        layout.addWidget(LaunchGroup)
        ##########################################


        #Exit Button layout
        self.btnQuit = QPushButton('Exit')
        self.btnQuit.clicked.connect(self.btnQuit_onclick)
        layout.addWidget(self.btnQuit)

        self.setLayout(layout)


    def US_Left_onclick(self):
        print("CLICKED")
        # trig_pin, echo_pin, units = setup_left()
        # distance = get_sonar_readings(trig_pin, echo_pin, units)
        # print(distance)

    def US_Left_onclick_status(self):
        trig_pin, echo_pin, units = setup_left()
        distance = get_sonar_readings(trig_pin, echo_pin, units)
        if(distance > 0.0):
            print(True)
        else:
            print("Invalid Ultrasonic Reading")

    def US_Right_onclick(self):
        trig_pin, echo_pin, units = setup_right()
        distance = get_sonar_readings(trig_pin, echo_pin, units)
        print(distance)

    def US_Right_onclick_status(self):
        trig_pin, echo_pin, units = setup_right()
        distance = get_sonar_readings(trig_pin, echo_pin, units)
        if(distance > 0.0):
            print(True)
        else:
            print("Invalid Ultrasonic Reading")

    def US_Front_onclick(self):
        trig_pin, echo_pin, units = setup_front()
        distance = get_sonar_readings(trig_pin, echo_pin, units)
        print(distance)

    def US_Front_onclick_status(self):
        trig_pin, echo_pin, units = setup_front()
        distance = get_sonar_readings(trig_pin, echo_pin, units)
        if(distance > 0.0):
            print(True)
        else:
            print("Invalid Ultrasonic Reading")

    def enc_Left_onclick(self):
        print("On Click")

    def enc_Left_onclick_status(self):
        print("On Click Status")

    def enc_Right_onclick(self):
        print("On Click")

    def enc_Right_onclick_status(self):
        print("On Click Status")

    def enc_stepper_onclick(self):
        print("On Click")

    def enc_stepper_onclick_status(self):
        print("On Click Status")

    def enc_servo_onclick(self):
        print("On Click")

    def enc_servo_onclick_status(self):
        print("On Click Status")

    def odome_onclick(self):
        print("gefefe")

    def go_onclick(self):
        print("gefefe")

    def pose_onclick(self):
        print("gefefe")

    def btnQuit_onclick(self):
        self.parent().close()


#ULTRASONIC SETUP
def get_sonar_readings(trig_pins, echo_pins, units):

    send_pulse(trig_pins)
    pulse_duration = get_pulse_duration(echo_pins)
    distances = convert_duration_to_distance(pulse_duration, units)
    time.sleep(0.03)
    
    return distances
    
def send_pulse(trig,duration=0.00001):
    GPIO.output(trig, True)
    time.sleep(duration)
    GPIO.output(trig, False)
    
def get_pulse_duration(echo):
    while GPIO.input(echo) == 0:
        pulse_start = time.time()

    while GPIO.input(echo) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    return pulse_duration

def convert_duration_to_distance(duration,units='in'):
    unit_dict = {'cm': 17150, 'in': 6750}
    unit_conversion = unit_dict.get(units)
    distance = duration * unit_conversion
    return distance

def setup_left():
    GPIO.setmode(GPIO.BCM)
    units = 'in'  # 'cm'
    trig_pin = 22
    echo_pin = 23

    GPIO.setup(trig_pin, GPIO.OUT)
    GPIO.setup(echo_pin, GPIO.IN)
    GPIO.output(trig_pin, False)

    return trig_pin, echo_pin, units

def setup_front():
    GPIO.setmode(GPIO.BCM)
    units = 'in'  # 'cm'
    trig_pin = 20
    echo_pin = 21

    GPIO.setup(trig_pin, GPIO.OUT)
    GPIO.setup(echo_pin, GPIO.IN)
    GPIO.output(trig_pin, False)

    return trig_pin, echo_pin, units

def setup_right():
    GPIO.setmode(GPIO.BCM)
    units = 'in'  # 'cm'
    trig_pin = 18
    echo_pin = 19

    GPIO.setup(trig_pin, GPIO.OUT)
    GPIO.setup(echo_pin, GPIO.IN)
    GPIO.output(trig_pin, False)

    return trig_pin, echo_pin, units





def main():

    app = QApplication(sys.argv)
    mainWidget = RobotMonitorWidget(app)
    mainWindow = QMainWindow()
    mainWindow.setWindowTitle('Police Academy Robot')
    mainWindow.setCentralWidget(mainWidget)
    mainWindow.setStatusBar(QStatusBar())
    mainWindow.show()
    app.exec_()

if __name__ == '__main__':
    main()