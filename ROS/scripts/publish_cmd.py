#!/usr/bin/env python

import rospy 
import roslib
import math 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


twist = Twist()
twist.angular.x = 0.0
twist.angular.y = 0.0
twist.angular.z = 0.0
twist.linear.x = 0.0
twist.linear.y = 0.0
twist.linear.z = 0.0



def map_range(left,right,leftMin=150,leftMax=450,rightMin=150,rightMax=450):
    leftSpan = (leftMax) - (leftMin)
    rightSpan = (rightMax - rightMin)
    if( ( ((-1.0*left) < leftMin) and ((-1.0*left) <= leftMax) ) and  ( (right > rightMin) and ( right <= rightMax) ) ):
        valueScaledLeft = -1.0*(float( left + leftMin) / float(leftSpan))   
        valueScaledRight = float( right + rightMin) / float(rightSpan)    
        return valueScaledLeft,valueScaledRight



def callback(msg,args):
    global twist
    flag = 0
    front = msg.ranges[0]
    right = -1.0*msg.ranges[1]
    left =  msg.ranges[2]

    # left_cmd,right_cmd = map_range(left,right)
   
    if( ( left - right) >0 ):
        if ( abs((left - right)) > 50 ):
            twist.linear.x = 0.5
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.5
            flag = 1

    elif( ( right - left) >0 ):
        if ( abs((right - left)) >50):
            twist.linear.x = 0.5
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = -0.5
            flag = 1

    elif ( abs(right - left) >= 0.0  and  abs(right - left) <= 50 ):
        twist.linear.x = 0.5
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twsit.angular.y = 0.0
        twist.angular.z = 0.0
        flag = 1


    if(flag == 1):
        args.publish(twist)
        flag =0
    else:
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        args.publish(twist)
        flag = 0


def main():
    rospy.init_node('Publish_cmd_vel_fromUS')
    rate = rospy.Rate(10)
    pub = rospy.Publisher('cmd_vel',Twist,queue_size=10)
    rospy.Subscriber("scan",LaserScan,callback,pub)
    rospy.spin()

if __name__ == '__main__':
    main()