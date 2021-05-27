#!/usr/bin/env python

from __future__ import print_function
import rospy
from std_msgs.msg import String, Int32 , Float32
import random

rospy.init_node("setpoint_taker")
teju_setpoint1 = rospy.Publisher("/teju/give_setpoints1" , Float32 , queue_size = 1)
teju_setpoint2 = rospy.Publisher("/teju/give_setpoints2" , Float32 , queue_size = 1)
teju_setpoint3 = rospy.Publisher("/teju/give_setpoints3" , Float32 , queue_size = 1)
teju_setpoint4 = rospy.Publisher("/teju/give_setpoints4" , Float32 , queue_size = 1)
teju_setpoint5 = rospy.Publisher("/teju/give_setpoints5" , Float32 , queue_size = 1)
teju_setpoint6 = rospy.Publisher("/teju/give_setpoints6" , Float32 , queue_size = 1)
teju_setpoint = rospy.Publisher("/teju/give_setpoints" , String , queue_size = 1)



input_setpoint = 0.001
rate = rospy.Rate(20)
i = 0
while not rospy.is_shutdown():

        #print("put")
        #input_setpoint = raw_input()
        #teju_setpoint.publish(input_setpoint)

        teju_setpoint1.publish(0.001)
        teju_setpoint2.publish(-0.002)
        teju_setpoint3.publish(0.004)
        teju_setpoint4.publish(-0.008)
        teju_setpoint5.publish(0.032)
        teju_setpoint6.publish(-0.015)


        rate.sleep()
	#print("setpoint:")
        # if i % 2 == 0:
	#         input_setpoint.data = "0 0 1000"
        #         teju_setpoint.publish(input_setpoint)
        #         print("setpoint -", input_setpoint)
        #         rate.sleep()
        # else:
        #         input_setpoint.data = "0 0 5" 
        #         teju_setpoint.publish(input_setpoint)
        #         print("setpoint -", input_setpoint) 
        #         rate.sleep()    
	# 
        # i = i + 1
        
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