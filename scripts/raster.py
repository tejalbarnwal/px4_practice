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
        rospy.Subscriber("/mavros/local_position/pose" , PoseStamped, self.pose_callback , queue_size = 1)

        self.teju_setpoint = rospy.Publisher("/teju/give_setpoints" , String , queue_size = 1)

        self.raster_points = [ [0,10], [2,0], [0,-10], [2,0] ]
        self.estimate_pose = [0,0]

    def pose_callback(self , msg):
        self.pose_x = msg.pose.position.x
        self.pose_y = msg.pose.position.y


    def action(self , rounds):
        
        for i in range(rounds) :
            for j in self.raster_points:
                    print("x" , abs(self.pose_x - self.estimate_pose[0]))
                    print("y" , abs(self.pose_y-self.estimate_pose[1]))
                    print("esimate_y" , self.estimate_pose[1])
                    print("estimate_x" , self.estimate_pose[0])

                    if abs(self.pose_x - self.estimate_pose[0]) <= 0.1 and abs(self.pose_y-self.estimate_pose[1]) <= 0.1:
                        yo = str(j[0]) + " " + str(j[1]) + " " + "0"
                        print("yo" , yo)
                        self.teju_setpoint.publish(yo)
                        self.estimate_pose[0] = self.estimate_pose[0] + j[0]
                        self.estimate_pose[1] = self.estimate_pose[1] + j[1]
                        time.sleep(10)
                        while abs(self.pose_x - self.estimate_pose[0]) > 0.1 and abs(self.pose_y-self.estimate_pose[1]) > 0.1:
                            print("x1" , abs(self.pose_x - self.estimate_pose[0]))
                            print("y1" , abs(self.pose_y-self.estimate_pose[1]))
                            pass
                    else:
                        print("ohh noooo")


if __name__=="__main__":
    yo = raster_setpoints_control()
    print("put no.of rounds for raster")
    num_rounds = raw_input()
    yo.action(int(num_rounds))
    #rospy.spin()
    






        