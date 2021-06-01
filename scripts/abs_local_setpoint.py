#!/usr/bin/env python

from __future__ import print_function
import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest,CommandBoolResponse, SetMode , SetModeRequest, SetModeResponse
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler , euler_from_quaternion
import time
from std_msgs.msg import String , Float64
import math

class local_setpoints_control():
    def __init__(self):
        rospy.init_node("offboard_node")

        # subscriber to /mavros/state
        rospy.Subscriber("/mavros/state" ,State, self.state_callback , queue_size = 1)
        
        rospy.Subscriber("/teju/give_setpoints" , String , self.setpoint_position_local_callback)
        rospy.Subscriber("/mavros/local_position/pose" , PoseStamped , self.pose_callback)

        self.local_setpoint_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped , queue_size=1)

        self.setpoint_x_pub = rospy.Publisher("/position/setpoint_x" , Float64 , queue_size =1)
        self.setpoint_y_pub = rospy.Publisher("/position/setpoint_y" , Float64 , queue_size=1)
        self.setpoint_z_pub = rospy.Publisher("/position/setpoint_z" , Float64 , queue_size=1)

        self.actual_x_pub = rospy.Publisher("/position/actual/x" , Float64 , queue_size=1)
        self.actual_y_pub = rospy.Publisher("/position/actual/y" , Float64 , queue_size=1)
        self.actual_z_pub = rospy.Publisher("/position/actual/z" , Float64 , queue_size=1)


        self.error_x_pub = rospy.Publisher("/position/error/x" , Float64 , queue_size=1)
        self.error_y_pub = rospy.Publisher("/position/error/y" , Float64 , queue_size=1)
        self.error_z_pub = rospy.Publisher("/position/error/z" , Float64 , queue_size=1)

        self.error_percent_x_pub = rospy.Publisher("/position/error_percent/x" , Float64 , queue_size=1)
        self.error_percent_y_pub = rospy.Publisher("/position/error_percent/y" , Float64 , queue_size=1)
        self.error_percent_z_pub = rospy.Publisher("/position/error_percent/z" , Float64 , queue_size=1)


        self.setpoint = PoseStamped()
        self.actual = PoseStamped()

        self.previous_setpoint = PoseStamped()
        self.previous_rpy = None

        self.state_indicator = State()
        self.rate = rospy.Rate(20)
        self.set_mode = SetModeRequest()
        self.isarmed = CommandBoolRequest()
        self.arming_response = CommandBoolResponse()
        self.set_mode_response = SetModeResponse()
        self.setpoints_given = False
        self.rpy = None

    def pose_callback(self , msg):
        self.actual = msg

    def state_callback(self,msg):
        print("Current mode of the drone" , msg.mode)
        # print("Is MAVROS connected to SITL", msg.connected)
        self.state_indicator = msg
        # print(self.state_indicator.connected)

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
        # print("condition--",(not rospy.is_shutdown()) ,(self.state_indicator.connected) ,(i > 0))
        while (not rospy.is_shutdown()) and (self.state_indicator.connected) and (i > 0):
            self.local_setpoint_pub.publish(self.setpoint)
            i=i-1
            print("setpoint", i)
            self.rate.sleep()
            if i == 1: 
                return True
        else: 
            return False        

    def setpoint_position_local_callback(self , listxyz):
        listxyz = listxyz.data.split(" ")
        print(listxyz)

        # delta_x = float(listxyz[0]) * math.cos(self.previous_rpy[2]) - float(listxyz[2]) * math.sin(self.previous_rpy[2])
        # delta_x = float(listxyz[0]) * math.sin(self.previous_rpy[2]) + float(listxyz[2]) * math.cos(self.previous_rpy[2])

        self.setpoint.pose.position.x = float(listxyz[0])
        self.setpoint.pose.position.y = float(listxyz[1])
        self.setpoint.pose.position.z = float(listxyz[2])

        self.rpy = [0.0 , 0.0 , 0.0]

        quaternion = quaternion_from_euler(self.rpy[0] , self.rpy[1] , self.rpy[2])

        self.setpoint.pose.orientation.x = quaternion[0]
        self.setpoint.pose.orientation.y = quaternion[1]
        self.setpoint.pose.orientation.z = quaternion[2]
        self.setpoint.pose.orientation.w = quaternion[3] 

    def publishes(self):

        self.local_setpoint_pub.publish(self.setpoint)

        self.setpoint_x_pub.publish(self.setpoint.pose.position.x)
        self.setpoint_y_pub.publish(self.setpoint.pose.position.y)
        self.setpoint_z_pub.publish(self.setpoint.pose.position.z)  

        self.actual_x_pub.publish(self.actual.pose.position.x)
        self.actual_y_pub.publish(self.actual.pose.position.y)  
        self.actual_z_pub.publish(self.actual.pose.position.z)  

        self.error_x_pub.publish((self.setpoint.pose.position.x - self.actual.pose.position.x))
        self.error_y_pub.publish((self.setpoint.pose.position.y - self.actual.pose.position.y))
        self.error_z_pub.publish((self.setpoint.pose.position.z - self.actual.pose.position.z))

        if self.setpoint.pose.position.x != 0: 
            self.error_percent_x_pub.publish(
                                abs((self.setpoint.pose.position.x - self.actual.pose.position.x) / self.setpoint.pose.position.x) * 100)

        if self.setpoint.pose.position.y != 0:    
            self.error_percent_y_pub.publish(
                                abs((self.setpoint.pose.position.y - self.actual.pose.position.y) / self.setpoint.pose.position.y) * 100)

        if self.setpoint.pose.position.z != 0:    
            self.error_percent_z_pub.publish(
                                abs((self.setpoint.pose.position.z - self.actual.pose.position.z) / self.setpoint.pose.position.z) * 100)







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
                        yo.publishes()



        else:
            yo.onstart_setpoint()                    

"""
feedback to take previous setpoints into consideration
setpoint_position/local gives global position
already armed case remaining to be resolved in arming service
"""                
                        