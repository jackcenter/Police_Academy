#!/usr/bin/env python
import rospy 
import roslib
import math 
import tf 
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
import numpy as np 
# from scipy.spatial.transform import Rotation 
import message_filters
import time 
import tf2_ros
from sensor_msgs.msg import Range

def normalize_quaternion(rot):
    ratio = math.sqrt(rot[0]**2 + rot[1]**2 + rot[2]**2 + rot[3]**2)
    return (rot[0]/ratio, rot[1]/ratio, rot[2]/ratio, rot[3]/ratio)

def callback_front(data, args):
    roll = 0
    pitch = 0
    yaw = math.radians(0)
    rotation_quaternion = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
    rotation_quaternion = normalize_quaternion(rotation_quaternion)
    translation_vector = (data.range,0.0,0.0)
    current_time = rospy.Time.now()
    print("Front")
    args.sendTransform(translation_vector,rotation_quaternion,current_time,"US_front_view","US_front")


def callback_left(data, args):
    roll = 0
    pitch = 0
    yaw = math.radians(0)
    rotation_quaternion = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
    rotation_quaternion = normalize_quaternion(rotation_quaternion)
    translation_vector = (data.range,0.0,0.0)
    current_time = rospy.Time.now()
    print("Left")
    args.sendTransform(translation_vector,rotation_quaternion,current_time,"US_left_view","US_left")

def callback_right(data, args):
    roll = 0
    pitch = 0
    yaw = math.radians(0)
    rotation_quaternion = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
    rotation_quaternion = normalize_quaternion(rotation_quaternion)
    translation_vector = (data.range,0.0,0.0)
    current_time = rospy.Time.now()
    print("Right")
    args.sendTransform(translation_vector,rotation_quaternion,current_time,"US_right_view","US_right")


def main():
    rospy.init_node('ultrasonic_view_publisher')
    transform_broadcaster_front_ultrasonic = tf.TransformBroadcaster()
    transform_broadcaster_left_ultrasonic = tf.TransformBroadcaster()
    transform_broadcaster_right_ultrasonic = tf.TransformBroadcaster()
    rate = rospy.Rate(0.5)
    while(not rospy.is_shutdown()):
        rospy.Subscriber("ultrasonic_front",Range,callback_front, transform_broadcaster_front_ultrasonic)
        rospy.Subscriber("ultrasonic_left",Range,callback_left, transform_broadcaster_left_ultrasonic)
        rospy.Subscriber("ultrasonic_right",Range,callback_right, transform_broadcaster_right_ultrasonic)
        time.sleep(0.5)
    rospy.spin()

if __name__ == '__main__':
    main()
