#!/usr/bin/env python

from __future__ import print_function
import rospy
from std_msgs.msg import String

rospy.init_node("setpoint_taker")
teju_setpoint = rospy.Publisher("/teju/give_setpoints" , String , queue_size = 1)
input_setpoint = String()

while not rospy.is_shutdown():
	print("setpoint:")
	input_setpoint.data = raw_input()
	teju_setpoint.publish(input_setpoint)

'''
listrtz = listrtz.data.split(" ")

        # radial = float(listrtz[0])
        # x = radial 
        theta = float(listrtz[1]) * math.pi / 180
        # relative_height = float(listrtz[2])

    
        self.setpoint.pose.position.x = self.previous_setpoint.pose.position.x + float(listrtz[0]) * math.cos(theta)
        self.setpoint.pose.position.y = self.previous_setpoint.pose.position.y + float(listrtz[0])* math.sin(theta)
        self.setpoint.pose.position.z = self.previous_setpoint.pose.position.z + float(listrtz[2])

        self.rpy = [0.0 , 0.0 , theta]

        quaternion = quaternion_from_euler(self.rpy[0] , self.rpy[1] , self.rpy[2])

        self.setpoint.pose.orientation.x = quaternion[0]
        self.setpoint.pose.orientation.y = quaternion[1]
        self.setpoint.pose.orientation.z = quaternion[2]
        self.setpoint.pose.orientation.w = quaternion[3] 

        self.previous_setpoint = self.setpoint    

'''        