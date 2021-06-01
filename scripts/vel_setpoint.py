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

class local_setpoints_control():
    def __init__(self):
        rospy.init_node("offboard_node")

        # subscriber to /mavros/state
        rospy.Subscriber("/mavros/state" ,State, self.state_callback , queue_size = 1)
        # rospy.Subscriber("/teju_give_setpoints" , String , self.setpoint_position_local_callback)
        rospy.Subscriber("/teju/give_velocity" , String , self.setpoint_velocity_callback)
        rospy.Subscriber("/mavros/local_position/velocity_body" , TwistStamped , self.velocity_callback)
        rospy.Subscriber("/mavros/local_position/pose" , PoseStamped , self.pose_callback)


        # self.local_setpoint_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped , queue_size=1)
        self.velocity_setpoint_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel" , TwistStamped , queue_size=1)

        self.vel_setpoint_x_pub = rospy.Publisher("/velocity/setpoint/x" , Float64 , queue_size=1)
        self.vel_setpoint_y_pub = rospy.Publisher("/velocity/setpoint/y" , Float64 , queue_size=1)
        self.vel_setpoint_z_pub = rospy.Publisher("/velocity/setpoint/z" , Float64 , queue_size=1)
        self.vel_setpoint_yaw_pub = rospy.Publisher("/velocity/setpoint/yaw" , Float64 , queue_size=1)

        self.actual_x_pub = rospy.Publisher("/velocity/actual/x" , Float64 , queue_size=1)
        self.actual_y_pub = rospy.Publisher("/velocity/actual/y" , Float64 , queue_size=1)
        self.actual_z_pub = rospy.Publisher("/velocity/actual/z" , Float64 , queue_size=1)
        self.actual_yaw_pub = rospy.Publisher("/velocity/actual/yaw" , Float64 , queue_size=1)

        self.error_x_pub = rospy.Publisher("/velocity/error/x" , Float64 , queue_size=1)
        self.error_y_pub = rospy.Publisher("/velocity/error/y" , Float64 , queue_size=1)
        self.error_z_pub = rospy.Publisher("/velocity/error/z" , Float64 , queue_size=1)
        self.error_yaw_pub = rospy.Publisher("/velocity/error/yaw" , Float64 , queue_size=1)

        self.error_percent_x_pub = rospy.Publisher("/velocity/error_percent/x" , Float64 , queue_size=1)
        self.error_percent_y_pub = rospy.Publisher("/velocity/error_percent/y" , Float64 , queue_size=1)
        self.error_percent_z_pub = rospy.Publisher("/velocity/error_percent/z" , Float64 , queue_size=1)
        self.error_percent_yaw_pub = rospy.Publisher("/velocity/error_percent/yaw" , Float64 , queue_size=1)

        self.actual_px_pub = rospy.Publisher("/position/actual/x" , Float64 , queue_size=1)
        self.actual_py_pub = rospy.Publisher("/position/actual/y" , Float64 , queue_size=1)
        self.actual_pz_pub = rospy.Publisher("/position/actual/z" , Float64 , queue_size=1)


        self.velocity_setpoint = TwistStamped()
        self.actual = TwistStamped()
        self.position = PoseStamped()

        self.state_indicator = State()
        self.rate = rospy.Rate(10)
        self.set_mode = SetModeRequest()
        self.isarmed = CommandBoolRequest()
        self.arming_response = CommandBoolResponse()
        self.set_mode_response = SetModeResponse()
        self.setpoints_given = False
        self.rpy = None

    def pose_callback(self , msg):
        self.position = msg

    def velocity_callback(self , msg):
        self.actual = msg    

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
        self.velocity_setpoint.twist.linear.z = 0.4

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


    def setpoint_velocity_callback(self , key) :
        
        # vector_dir = vector_direction.data.split(" ")
        # self.velocity_setpoint.twist.angular.x = 0.0
        # self.velocity_setpoint.twist.angular.y = 0.0
        key = key.data
        print("key",key)

        if key == "w":
            self.velocity_setpoint.twist.linear.x += 0.2
        if key == "x":
            self.velocity_setpoint.twist.linear.x -= 0.2    
        if key == "a":    
            self.velocity_setpoint.twist.linear.y += 0.2 
        if key == "d":    
            self.velocity_setpoint.twist.linear.y -= 0.2
        if key == "s":
            self.velocity_setpoint.twist.linear.z += 0.2
        if key == "z":

            self.velocity_setpoint.twist.linear.z -= 0.2        
             
        if key == "e":
            self.velocity_setpoint.twist.angular.z += 0.2
        if key == "r":
            self.velocity_setpoint.twist.angular.z -= 0.2 

        if key == "i":
            self.velocity_setpoint.twist.angular.x += 0.2
        if key == "j":
            self.velocity_setpoint.twist.angular.x -= 0.2 
            
        if key == "o":
            self.velocity_setpoint.twist.angular.y += 0.2
        if key == "k":
            self.velocity_setpoint.twist.angular.y -= 0.2             

    def publishing(self):
        self.velocity_setpoint_pub.publish(self.velocity_setpoint)

        self.vel_setpoint_x_pub.publish(self.velocity_setpoint.twist.linear.x)
        self.vel_setpoint_y_pub.publish(self.velocity_setpoint.twist.linear.y)
        self.vel_setpoint_z_pub.publish(self.velocity_setpoint.twist.linear.z)
        self.vel_setpoint_yaw_pub.publish(self.velocity_setpoint.twist.angular.z)    

        self.actual_x_pub.publish(self.actual.twist.linear.x)
        self.actual_y_pub.publish(self.actual.twist.linear.y)
        self.actual_z_pub.publish(self.actual.twist.linear.z)
        self.actual_yaw_pub.publish(self.actual.twist.angular.z)

        self.error_x_pub.publish((self.velocity_setpoint.twist.linear.x - self.actual.twist.linear.x))
        self.error_y_pub.publish((self.velocity_setpoint.twist.linear.y - self.actual.twist.linear.y))
        self.error_z_pub.publish((self.velocity_setpoint.twist.linear.z - self.actual.twist.linear.z))
        self.error_yaw_pub.publish((self.velocity_setpoint.twist.angular.z - self.actual.twist.angular.z))

        self.actual_px_pub.publish(self.position.pose.position.x)
        self.actual_py_pub.publish(self.position.pose.position.y)  
        self.actual_pz_pub.publish(self.position.pose.position.z)  

        if self.velocity_setpoint.twist.linear.x != 0:
            self.error_percent_x_pub.publish(
                        abs((self.velocity_setpoint.twist.linear.x - self.actual.twist.linear.x) / self.velocity_setpoint.twist.linear.x))

        if self.velocity_setpoint.twist.linear.y != 0:
            self.error_percent_y_pub.publish(
                        abs((self.velocity_setpoint.twist.linear.y - self.actual.twist.linear.y) / self.velocity_setpoint.twist.linear.y))
        
        if self.velocity_setpoint.twist.linear.z != 0:
            self.error_percent_z_pub.publish(
                        abs((self.velocity_setpoint.twist.linear.x - self.actual.twist.linear.z) / self.velocity_setpoint.twist.linear.z))

        if self.velocity_setpoint.twist.angular.z != 0:
            self.error_percent_yaw_pub.publish(
                        abs((self.velocity_setpoint.twist.angular.z - self.actual.twist.angular.z) / self.velocity_setpoint.twist.angular.z))                                    





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
                        yo.publishing()

                        print("linear.xyz {} , {} , {}".format(yo.velocity_setpoint.twist.linear.x , 
                                                                yo.velocity_setpoint.twist.linear.y , 
                                                                yo.velocity_setpoint.twist.linear.z)) 

                        print("angular.xyz {} , {} , {}".format( yo.velocity_setpoint.twist.angular.x , 
                                                                 yo.velocity_setpoint.twist.angular.y , 
                                                                 yo.velocity_setpoint.twist.angular.z))                                           
                    

        else:
            yo.onstart_setpoint()                    

"""
feedback to take previous setpoints into consideration
setpoint_position/local gives global position
already armed case remaining to be resolved in arming service
"""                
                        