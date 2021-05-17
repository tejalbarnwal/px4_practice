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

class local_setpoints_control():
    def __init__(self):
        rospy.init_node("offboard_node")

        # subscriber to /mavros/state
        rospy.Subscriber("/mavros/state" ,State, self.state_callback , queue_size = 1)
        
        rospy.Subscriber("/teju_give_setpoints" , String , self.setpoint_position_local_callback)

        self.local_setpoint_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped , queue_size=1)

        self.setpoint = PoseStamped()
        self.previous_setpoint = PoseStamped()
        self.previous_rpy = None

        self.state_indicator = State()
        self.rate = rospy.Rate(10)
        self.set_mode = SetModeRequest()
        self.isarmed = CommandBoolRequest()
        self.arming_response = CommandBoolResponse()
        self.set_mode_response = SetModeResponse()
        self.setpoints_given = False
        self.rpy = None


    def state_callback(self,msg):
        print("Current mode of the drone" , msg.mode)
        print("Is MAVROS connected to SITL", msg.connected)
        self.state_indicator = msg
        print(self.state_indicator.connected)

    def arming_service(self):
        if not self.state_indicator.armed:
            rospy.wait_for_service("/mavros/cmd/arming")
            try:
                arming_service = rospy.ServiceProxy("/mavros/cmd/arming",CommandBool)
                self.isarmed.value = True
                self.arming_response = arming_service(self.isarmed)
                while (not rospy.is_shutdown()) and (self.state_indicator.connected) and (not(self.arming_response.success)):
                    self.arming_response = arming_service(self.isarmed)
                print("-------------ARMING SUCCESSFULL------------")    
                return True    
            except rospy.ServiceException as e:
                print("ERROR IN ARMING DRONE",e)
        else:
            return True        

    def change_mode(self):
        rospy.wait_for_service("/mavros/set_mode")
        try:
            change_mode_service = rospy.ServiceProxy("/mavros/set_mode", SetMode)
            self.set_mode.custom_mode = "OFFBOARD"
            while (not rospy.is_shutdown()) and (self.state_indicator.connected) and (self.state_indicator.mode != "OFFBOARD"):
                self.set_mode_response  = change_mode_service(self.set_mode)
                print("------------MODE CHANGING INITIATED---------------")
            return True    
        except rospy.ServiceException as e:
            print("ERROR IN CHANGING MODE",e)    

    def onstart_setpoint(self):
        self.setpoint.pose.position.x = 0
        self.setpoint.pose.position.y = 0
        self.setpoint.pose.position.z = 3.0
        self.setpoint.pose.orientation.x = 0
        self.setpoint.pose.orientation.y = 0
        self.setpoint.pose.orientation.z = 0
        self.setpoint.pose.orientation.w = 1

        i = 50
        print("condition--",(not rospy.is_shutdown()) ,(self.state_indicator.connected) ,(i > 0))
        while (not rospy.is_shutdown()) and (self.state_indicator.connected) and (i > 0):
            self.local_setpoint_pub.publish(self.setpoint)
            i=i-1
            print("setpoint", i)
            self.rate.sleep()
            if i == 1: 
                self.previous_setpoint = self.setpoint
                self.previous_rpy = list(euler_from_quaternion([self.previous_setpoint.pose.orientation.x,
                                                                self.previous_setpoint.pose.orientation.y,
                                                                self.previous_setpoint.pose.orientation.z,
                                                                self.previous_setpoint.pose.orientation.w]))
                return True
        else: 
            return False        

    def setpoint_position_local_callback(self , listxyz):
        listxyz = listxyz.data.split(" ")

        delta_x = float(listxyz[0]) * math.cos(self.previous_rpy[2]) - float(listxyz[2]) * math.sin(self.previous_rpy[2])
        delta_x = float(listxyz[0]) * math.sin(self.previous_rpy[2]) + float(listxyz[2]) * math.cos(self.previous_rpy[2])

        self.setpoint.pose.position.x = self.previous_setpoint.pose.position.x + float(listxyz[0])
        self.setpoint.pose.position.y = self.previous_setpoint.pose.position.y + float(listxyz[1])
        self.setpoint.pose.position.z = self.previous_setpoint.pose.position.z + float(listxyz[2])

        self.rpy = [0.0 , 0.0 ,  math.atan2(float(listxyz[1]) , float(listxyz[0]))]

        quaternion = quaternion_from_euler(self.rpy[0] , self.rpy[1] , self.rpy[2])

        self.setpoint.pose.orientation.x = quaternion[0]
        self.setpoint.pose.orientation.y = quaternion[1]
        self.setpoint.pose.orientation.z = quaternion[2]
        self.setpoint.pose.orientation.w = quaternion[3] 

        self.previous_setpoint = self.setpoint    
        self.previous_rpy = list(euler_from_quaternion([self.previous_setpoint.pose.orientation.x,
                                                                self.previous_setpoint.pose.orientation.y,
                                                                self.previous_setpoint.pose.orientation.z,
                                                                self.previous_setpoint.pose.orientation.w]))      


if __name__=="__main__":
    yo = local_setpoints_control()
    #rospy.spin()
    while True:
        if yo.onstart_setpoint():
            if yo.change_mode():
                if yo.arming_service():
                    print("COMPLETED")
                    yo.onstart_setpoint()

                    while not rospy.is_shutdown():
                        yo.local_setpoint_pub.publish(yo.setpoint)    

        else:
            yo.onstart_setpoint()                    

"""
feedback to take previous setpoints into consideration
setpoint_position/local gives global position
already armed case remaining to be resolved in arming service
"""                
                        