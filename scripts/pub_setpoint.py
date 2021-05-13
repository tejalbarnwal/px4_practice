#!/usr/bin/env python

from __future__ import print_function
import rospy
from std_msgs.msg import String

rospy.init_node("setpoint_taker")
teju_setpoint = rospy.Publisher("/teju_give_setpoints" , String , queue_size = 1)
input_setpoint = String()

while not rospy.is_shutdown():
	print("setpoint:")
	input_setpoint.data = raw_input()
	teju_setpoint.publish(input_setpoint)
