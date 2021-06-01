#!/usr/bin/env python

from __future__ import print_function
import rospy
from std_msgs.msg import String

rospy.init_node("setpoint_taker")
teju_setpoint = rospy.Publisher("/teju/give_velocity" , String , queue_size = 1)
#teju_attitude_raw_pub = rospy.Publisher("/teju/attitude_raw" , String , queue_size =1)

input_setpoint = String()

while not rospy.is_shutdown():
	print("press key")
	input_setpoint.data = raw_input()
	teju_setpoint.publish(input_setpoint)
