#!/usr/bin/env python

from __future__ import print_function
import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest,CommandBoolResponse, SetMode , SetModeRequest, SetModeResponse
from geometry_msgs.msg import PoseStamped , TwistStamped
from tf.transformations import quaternion_from_euler , euler_from_quaternion
import time
from std_msgs.msg import String , Float64
import math
import random

class local_setpoints_control():
    def __init__(self):
        rospy.init_node("offboard_node")

        # subscriber to /mavros/state
        rospy.Subscriber("/mavros/state" ,State, self.state_callback , queue_size = 1)
        # rospy.Subscriber("/teju_give_setpoints" , String , self.setpoint_position_local_callback)
        # rospy.Subscriber("/teju/give_velocity" , String , self.setpoint_velocity_callback)

        # self.local_setpoint_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped , queue_size=1)
        self.velocity_setpoint_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel" , TwistStamped , queue_size=1)

        self.vel_setpoint_x_pub = rospy.Publisher("/teju/vel/x" , Float64 , queue_size=1)
        self.vel_setpoint_y_pub = rospy.Publisher("/teju/vel/y" , Float64 , queue_size=1)
        self.vel_setpoint_z_pub = rospy.Publisher("/teju/vel/z" , Float64 , queue_size=1)
        self.vel_setpoint_yaw_pub = rospy.Publisher("/teju/vel/yaw" , Float64 , queue_size=1)


        self.velocity_setpoint = TwistStamped()

        self.state_indicator = State()
        self.rate = rospy.Rate(3)
        self.set_mode = SetModeRequest()
        self.isarmed = CommandBoolRequest()
        self.arming_response = CommandBoolResponse()
        self.set_mode_response = SetModeResponse()
        self.setpoints_given = False
        self.rpy = None

        self.i = 0
        #self.rate = rospy.Rate(0.1)

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
        self.velocity_setpoint.twist.linear.x = 0.0
        self.velocity_setpoint.twist.linear.y = 0.0
        self.velocity_setpoint.twist.linear.z = 0.2

        self.velocity_setpoint.twist.angular.x = 0.0
        self.velocity_setpoint.twist.angular.y = 0.0
        self.velocity_setpoint.twist.angular.z = 0.0

        i = 50
        print("condition--",(not rospy.is_shutdown()) ,(self.state_indicator.connected) ,(i > 0))
        while (not rospy.is_shutdown()) and (self.state_indicator.connected) and (i > 0):
            self.velocity_setpoint_pub.publish(self.velocity_setpoint)
            i=i-1
            print("setpoint", i)
            self.rate.sleep()
            if i == 1: 
                return True
        else: 
            return False  


    def setpoint_velocity_callback(self) :
        
        # vector_dir = vector_direction.data.split(" ")
        # self.velocity_setpoint.twist.angular.x = 0.0
        # self.velocity_setpoint.twist.angular.y = 0.0
        
        self.velocity_setpoint.twist.linear.y = 0.0
        self.velocity_setpoint.twist.linear.z = 0.0

        self.velocity_setpoint.twist.angular.x = 0.0
        self.velocity_setpoint.twist.angular.y = 0.0
        self.velocity_setpoint.twist.angular.z = 0.0

        if self.i % 2 == 0:
            self.velocity_setpoint.twist.linear.x = random.randint(1 , 10)
        else:
            self.velocity_setpoint.twist.linear.x= random.randint(-10 , -1)  


        self.velocity_setpoint_pub.publish(self.velocity_setpoint)
        print("setpoint -", self.velocity_setpoint.twist.linear.x)


        self.i = self.i + 1
        


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
                        yo.setpoint_velocity_callback()
                        yo.rate.sleep()                                    
                    

        else:
            yo.onstart_setpoint()                    

"""
feedback to take previous setpoints into consideration
setpoint_position/local gives global position
already armed case remaining to be resolved in arming service
"""                
                        