#!/usr/bin/env python

from __future__ import print_function
import rospy
from mavros_msgs.msg import State



class switch_to_offboard():
    def __init__(self):
        rospy.init_node("offboard_node")

        # subscriber to /mavros/state
        rospy.Subscriber("/mavros/state" ,State, self.state_callback)
        
        # service to arm the drone
        

    def state_callback(self,msg):
        print("Current mode of the drone" , msg.mode)
        print("Is MAVROS connected to SITL", msg.connected)

if __name__=="__main__":
    yo = switch_to_offboard()
    rospy.spin()       
