#!/usr/bin/python

import sys
import rospy 
import numpy as np 
import time
from sensor_msgs.msg import Range
import tf 




# Role of this code is to read the Ultrasonics from the  GPIO and publish to topic ultrasonic_right 
# and also to publish a transform from US_front to base_link


import RPi.GPIO as GPIO
# from sensors import sonar_measurements as sonar

pub = rospy.Publisher('ultrasonic_right',Range,queue_size=10)

rospy.init_node('ultrasonic_right',anonymous = True)

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
    # trig_pin, echo_pin, units = setup()
    # distance = sonar.get_sonar_readings(trig_pin, echo_pin, units)
    # return distance
    trig_pin, echo_pin, units = setup()
    range_msg = Range()
    range_msg.header.frame_id = "ultrasonic_right"
    range_msg.radiation_type = 0
    range_msg.field_of_view = 0.1 #Fake value
    range_msg.min_range = 0.78
    range_msg.max_range = 157.48
    
    # br = tf.TransformBroadcaster()
    transform_broadcaster_right_ultrasonic = tf.TransformBroadcaster()


    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        distance = get_sonar_readings(trig_pin, echo_pin, units)
	# print(distance) 
        # Publishing to topic 
        roll = 0
        pitch = 0
        yaw = math.radians(0)
        rotation_quaternion = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
        translation_vector = (distance,0.0,0.0)
        current_time = rospy.Time.now()
        transform_broadcaster_front_ultrasonic.sendTransform(translation_vector,rotation_quaternion,current_time,"US_right_view","US_right")

        range_msg.header.stamp = rospy.Time.now()
        range_msg.range = distance
        # br.sendTransform (distance*25.4,0,0,tf.transformations.quaternion_from_euler(0,0,0),rospy.Time.now(),"US_right_view", "US_right")
        # br.sendTransform ((0,0,0),tf.transformations.quaternion_from_euler(0,0,0),rospy.Time.now(),"US_right_view", "US_right")
        pub.publish(range_msg)
        rate.sleep()



def setup():
    GPIO.setmode(GPIO.BCM)
    units = 'in'  # 'cm'
    trig_pin = 18
    echo_pin = 19

    GPIO.setup(trig_pin, GPIO.OUT)
    GPIO.setup(echo_pin, GPIO.IN)
    GPIO.output(trig_pin, False)

    return trig_pin, echo_pin, units


if __name__ == '__main__':
    main()
