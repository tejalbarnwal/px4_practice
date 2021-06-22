#!/usr/bin/env python

from __future__ import print_function
import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest,CommandBoolResponse, SetMode , SetModeRequest, SetModeResponse
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler , euler_from_quaternion
import time
from std_msgs.msg import String
import math

class raster_setpoints_control():
    def __init__(self):
        rospy.init_node("raster_scan_node")

        # subscriber to /mavros/state
        rospy.Subscriber("/mavros/state" ,State, self.state_callback , queue_size = 1)

        self.teju_setpoint = rospy.Publisher("/teju/give_setpoints" , String , queue_size = 1)

        self.raster_points = [ [0,10], [2,0], [0,-10], [2,0] ]
        self.estimate_pose = [0,0]

    def odom_callback(self , msg):
        self.pose_x = msg.pose.position.x
        self.pose_y = msg.pose.position.y


    def action(self , rounds):
        
        for i in rounds :
            for j in self.raster_points:
                    if abs(self.pose_x - self.estimate_pose) <= 0.01 and abs(self.pose_y-self.estimate_pose[1]) <= 0.01:
                        
                        self.teju_setpoint.publish(f"{j[0]} {j[1]} 0")
                        self.estimate_pose[0] = self.estimate_pose[0] + j[0]
                        self.estimate_pose[1] = self.estimate_pose[1] + j[1]
                    else:
                        while abs(self.pose_x - self.estimate_pose) >= 0.01 and abs(self.pose_y-self.estimate_pose[1]) >= 0.01:
                            pass


if __name__=="__main__":
    yo = raster_setpoints_control()
    print("put no.of rounds for raster")
    num_rounds = raw_input()
    yo.action(int(num_rounds))
    #rospy.spin()
    






        