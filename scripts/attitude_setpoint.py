#!/usr/bin/env python

from __future__ import print_function
import rospy
from std_msgs.msg import String, Int32 , Float32
import random

rospy.init_node("attitude_setpoint_taker")
roll = rospy.Publisher("/teju/roll_pub" , Float32 , queue_size = 1)
pitch = rospy.Publisher("/teju/pitch_pub" , Float32 , queue_size =1)
yaw = rospy.Publisher("/teju/yaw_pub", Float32 , queue_size = 1)

pitch_set = 0.0
yaw_set = 0.0
roll_set = 0.0
rate = rospy.Rate(20)

while not rospy.is_shutdown():
        print ("put")
        yaw_set = float(input())
        roll.publish(roll_set)

        #pitch_set = int(input_raw())
        pitch.publish(pitch_set)

        #yaw_set = int(input())
        yaw.publish(yaw_set)

        rate.sleep()