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


def normalize_quaternion(rot):
    ratio = math.sqrt(rot[0]**2 + rot[1]**2 + rot[2]**2 + rot[3]**2)
    return (rot[0]/ratio, rot[1]/ratio, rot[2]/ratio, rot[3]/ratio)

def main():
    #Code to generate tf between rigid links to publish on tf tre
    
    #Initialized Node
    rospy.init_node('robot_tf_broadcaster', anonymous=True)
    rate = rospy.Rate(0.5)

    while(not rospy.is_shutdown()):
        #base_link is the Origin of the robot 

        #left_wheel
        transform_broadcaster_left_wheel = tf.TransformBroadcaster()
        roll = 0
        pitch = 0
        yaw = math.radians(-90.0)
        rotation_quaternion = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
        rotation_quaternion = normalize_quaternion(rotation_quaternion)
        translation_vector = (0.34,92.62,18.47)
        current_time = rospy.Time.now()
        transform_broadcaster_left_wheel.sendTransform(translation_vector,rotation_quaternion,current_time,"left_wheel","base_link")

        #right_wheel
        transform_broadcaster_right_wheel = tf.TransformBroadcaster()
        roll = 0.0
        pitch = 0.0
        yaw = math.radians(90.0)
        rotation_quaternion = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
        rotation_quaternion = normalize_quaternion(rotation_quaternion)
        translation_vector = (0.34,-92.62,18.47)
        current_time = rospy.Time.now()
        transform_broadcaster_right_wheel.sendTransform(translation_vector,rotation_quaternion,current_time,"right_wheel","base_link")

        #Ultrasonic Left
        transform_broadcaster_US1 = tf.TransformBroadcaster()
        roll = 0.0
        pitch = 0.0
        yaw = math.radians(90.0)
        rotation_quaternion = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
        rotation_quaternion = normalize_quaternion(rotation_quaternion)
        translation_vector = (86.19,93.66,6.56)
        current_time = rospy.Time.now()
        transform_broadcaster_US1.sendTransform(translation_vector,rotation_quaternion,current_time,"US_left","base_link")

        #Ultrasonic Front
        transform_broadcaster_US2 = tf.TransformBroadcaster()
        roll = 0.0
        pitch = 0.0
        yaw = math.radians(0)
        rotation_quaternion = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
        rotation_quaternion = normalize_quaternion(rotation_quaternion)
        translation_vector = (130.73,-0.33,6.53)
        current_time = rospy.Time.now()
        transform_broadcaster_US2.sendTransform(translation_vector,rotation_quaternion,current_time,"US_front","base_link")

        #Ultrasonic Right
        transform_broadcaster_US3 = tf.TransformBroadcaster()
        roll = 0.0
        pitch = 0.0
        yaw = math.radians(-90)
        rotation_quaternion = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
        rotation_quaternion = normalize_quaternion(rotation_quaternion)
        translation_vector = (86.19,-93.33,6.53)
        current_time = rospy.Time.now()
        transform_broadcaster_US3.sendTransform(translation_vector,rotation_quaternion,current_time,"US_right","base_link")

        #turret
        transform_broadcaster_turret = tf.TransformBroadcaster()
        roll = 0.0
        pitch = math.radians(90)
        yaw = 0.0
        rotation_quaternion = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
        rotation_quaternion = normalize_quaternion(rotation_quaternion)
        translation_vector = (0,0,122.62)
        current_time = rospy.Time.now()
        transform_broadcaster_turret.sendTransform(translation_vector,rotation_quaternion,current_time,"turret","base_link")

        #Pitch Gun Exit 
        transform_broadcaster_gun = tf.TransformBroadcaster()
        roll = 0.0
        pitch = math.radians(-82.24)
        yaw = 0.0
        rotation_quaternion = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
        rotation_quaternion = normalize_quaternion(rotation_quaternion)
        translation_vector = (135.93,-3.13,133.33)
        current_time = rospy.Time.now()
        transform_broadcaster_turret.sendTransform(translation_vector,rotation_quaternion,current_time,"gun","turret")

        time.sleep(0.5)
    rospy.spin()    

if __name__ == '__main__':
    main()