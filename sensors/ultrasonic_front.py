#!/usr/bin/python

import sys
import rospy 
import numpy as np 
import time
from sensor_msgs.msg import Range

import RPi.GPIO as GPIO

pub = rospy.Publisher('ultrasonic_front',Range,queue_size=10)

rospy.init_node('ultrasonic_front',anonymous = True)

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
    

def main():
    trig_pin, echo_pin, units = setup()
    range_msg = Range()
    range_msg.header.frame_id = "ultrasonic_front"
    range_msg.radiation_type = 0
    range_msg.field_of_view = 0.1 #Fake value
    range_msg.min_range = 0.78
    range_msg.max_range = 157.48
    
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        distance = get_sonar_readings(trig_pin, echo_pin, units)
        # print(distance)
        range_msg.header.stamp = rospy.Time.now()
        range_msg.range = distance

        pub.publish(range_msg)
        rate.sleep()

def setup():
    GPIO.setmode(GPIO.BCM)
    units = 'in'  # 'cm'
    trig_pin = 20
    echo_pin = 21

    GPIO.setup(trig_pin, GPIO.OUT)
    GPIO.setup(echo_pin, GPIO.IN)
    GPIO.output(trig_pin, False)

    return trig_pin, echo_pin, units


if __name__ == '__main__':
    main()
