#!/usr/bin/env python
import rospy 
import roslib
import math 
import tf 
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
import numpy as np 
import message_filters
import time 
import tf2_ros
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range

args_left = Range()
args_right = Range()
args_front = Range()

def normalize_quaternion(rot):
    ratio = math.sqrt(rot[0]**2 + rot[1]**2 + rot[2]**2 + rot[3]**2)
    return (rot[0]/ratio, rot[1]/ratio, rot[2]/ratio, rot[3]/ratio)

# def multiply_tfs (trans1, rot1, trans2, rot2):
def multiply_tfs_left (trans2, rot2):
    global args_left
    
    # trans1_mat = tf.transformations.translation_matrix(trans1)
    # rot1_mat = tf.transformations.quaternion_matrix(rot1)
    trans1_mat = tf.transformations.translation_matrix(np.array([args_left.range*25.4,0,0]))
    rot1_mat = tf.transformations.quaternion_matrix(np.array([0,0,0,1]))
    mat1 = np.dot(trans1_mat,rot1_mat)

    trans2_mat = tf.transformations.translation_matrix(trans2)
    rot2_mat = tf.transformations.quaternion_matrix(rot2)
    mat2 = np.dot(trans2_mat,rot2_mat)

    mat3 = np.dot(mat1, mat2)
    trans3 = tf.transformations.translation_from_matrix(mat3)
    rot3 = tf.transformations.quaternion_from_matrix(mat3)

    return trans3, rot3

def multiply_tfs_right (trans2, rot2):
    global args_right
    
    # trans1_mat = tf.transformations.translation_matrix(trans1)
    # rot1_mat = tf.transformations.quaternion_matrix(rot1)
    trans1_mat = tf.transformations.translation_matrix(np.array([args_right.range*-25.4,0,0]))
    rot1_mat = tf.transformations.quaternion_matrix(np.array([0,0,0,1]))
    mat1 = np.dot(trans1_mat,rot1_mat)

    trans2_mat = tf.transformations.translation_matrix(trans2)
    rot2_mat = tf.transformations.quaternion_matrix(rot2)
    mat2 = np.dot(trans2_mat,rot2_mat)

    mat3 = np.dot(mat1, mat2)
    trans3 = tf.transformations.translation_from_matrix(mat3)
    rot3 = tf.transformations.quaternion_from_matrix(mat3)

    return trans3, rot3

def multiply_tfs_front (trans2, rot2):
    global args_front
    
    # trans1_mat = tf.transformations.translation_matrix(trans1)
    # rot1_mat = tf.transformations.quaternion_matrix(rot1)
    trans1_mat = tf.transformations.translation_matrix(np.array([args_front.range*25.4,0,0]))
    rot1_mat = tf.transformations.quaternion_matrix(np.array([0,0,0,1]))
    mat1 = np.dot(trans1_mat,rot1_mat)

    trans2_mat = tf.transformations.translation_matrix(trans2)
    rot2_mat = tf.transformations.quaternion_matrix(rot2)
    mat2 = np.dot(trans2_mat,rot2_mat)

    mat3 = np.dot(mat1, mat2)
    trans3 = tf.transformations.translation_from_matrix(mat3)
    rot3 = tf.transformations.quaternion_from_matrix(mat3)

    return trans3, rot3

def callback_front(msg , args_front):
    args_front.range = msg.range

def callback_left(msg , args_left):
    args_left.range = msg.range

def callback_right(msg , args_right):
    args_right.range = msg.range


def main():
    rospy.init_node('UStoLaserScan')
    global args_left
    global args_right
    global args_front
    rospy.Subscriber("ultrasonic_front",Range,callback_front,args_front)
    rospy.Subscriber("ultrasonic_left",Range,callback_left,args_left)
    rospy.Subscriber("ultrasonic_right",Range,callback_right,args_right)
    pub = rospy.Publisher('scan',LaserScan,queue_size=10)
    laserscan = LaserScan()
    laserscan.header.stamp = rospy.Time()
    laserscan.header.frame_id = "base_laser"
    laserscan.angle_min = math.pi/4 -0.05
    laserscan.angle_max = math.pi/4 +0.05
    laserscan.angle_increment = 0.1
    laserscan.time_increment = 0.0
    laserscan.range_max = 4000
    laserscan.range_min = 20

    tf_listener = tf.TransformListener()

    while not rospy.is_shutdown():

        # Child , Parent   , Lookuop child parent 
        tf_listener.waitForTransform("US_front" , "base_link" , rospy.Time() , rospy.Duration(2) )
        (posT1, posR1) = tf_listener.lookupTransform("base_link","US_front" ,  rospy.Time(0))
        # print(posT1)

        # tf_listener.waitForTransform("US_front_view" , "US_front" , rospy.Time() , rospy.Duration(2) )
        # (posT2, posR2) = tf_listener.lookupTransform("US_front_view" , "US_front", rospy.Time(0))
        # print(posT2)

        # trans_mul, rot_mul = multiply_tfs(posT1, posR1 , posT2, posR2)
        trans_mul, rot_mul = multiply_tfs_front(posT1, posR1)

        front = trans_mul[0]

        tf_listener.waitForTransform("US_right" , "base_link" , rospy.Time() , rospy.Duration(2) )
        # (posT1, posR1) = tf_listener.lookupTransform("US_right" , "base_link", rospy.Time(0))
        (posT1, posR1) = tf_listener.lookupTransform("US_right" , "base_link", rospy.Time(0))
        # print(posT1)

        # tf_listener.waitForTransform("US_right_view" , "US_right" , rospy.Time() , rospy.Duration(2) )
        # (posT2, posR2) = tf_listener.lookupTransform("US_right_view" , "US_right", rospy.Time(0))
        # # print(posT2)

        # trans_mul, rot_mul = multiply_tfs(posT1, posR1 , posT2, posR2)
        trans_mul, rot_mul = multiply_tfs_right(posT1, posR1)

        right = trans_mul[0]


        tf_listener.waitForTransform("US_left" , "base_link" , rospy.Time() , rospy.Duration(2) )
        (posT1, posR1) = tf_listener.lookupTransform("US_left" , "base_link", rospy.Time(0))
        
        trans_mul, rot_mul = multiply_tfs_left(posT1, posR1)

        left = trans_mul[0]

        laserscan.ranges = np.array([front,right,left])
        print(laserscan.ranges)

        pub.publish(laserscan)

if __name__ == "__main__":
    main()


