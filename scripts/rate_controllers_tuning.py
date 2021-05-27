#!/usr/bin/env python

from __future__ import print_function
import rospy
from mavros_msgs.msg import State , AttitudeTarget
from mavros_msgs.srv import CommandBool , CommandBoolRequest , CommandBoolResponse ,SetMode , SetModeRequest , SetModeResponse
from tf.transformations import quaternion_from_euler , euler_from_quaternion
from geometry_msgs.msg import PoseStamped , TwistStamped
import time
from std_msgs.msg import String ,Float64
import math

class rate_controller():
    def __init__(self):
        rospy.init_node("rate_controller_node")

        #sub to mavros state
        rospy.Subscriber("/mavros/state", State , self.state_callback, queue_size = 1)
        rospy.Subscriber("/mavros/local_position/pose" , PoseStamped, self.pose_callback , queue_size =1 )

        #pub to setpoint velocity
        self.velocity_setpoint_publisher = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel" , TwistStamped , queue_size = 1)
        self.attitude_raw_publisher = rospy.Publisher("/mavros/setpoint_raw/attitude" , AttitudeTarget , queue_size = 1)

        self.velocity_setpoint = TwistStamped()
        self.attitude_raw_setpoint = AttitudeTarget()

        self.state_indicator = State()

        self.rate = rospy.Rate(5)

        self.set_mode = SetModeRequest()

        self.isarmed = CommandBoolRequest()
        self.arming_response = CommandBoolResponse()

        self.set_mode_response = SetModeResponse()

        self.i = 0
        self.z = 0.0
        self.x = 0.0

    def state_callback(self , msg):
        self.state_indicator = msg
        # print("is mavros connected to sitl" , msg.connected)

    def pose_callback(self , msg):
        self.z = msg.pose.position.z   
        self.x = msg.pose.position.x 

    def arming_service(self):
        if not self.state_indicator.armed:
            rospy.wait_for_service("/mavros/cmd/arming")

            try:
                arming_service = rospy.ServiceProxy("/mavros/cmd/arming" ,CommandBool)
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
        self.velocity_setpoint.body_rate.y = 0
        self.attitude_raw_setpoint.body_rate.z = 0
        self.attitude_raw_setpoint.orientation.w = 1
        self.attitude_raw_setpoint.thrust = 

        i = 50
        # print("condition--",(point.twist.angular.z = 0.0

        self.attitude_raw_setpoint.type_mask = 128
        self.attitude_raw_setpoint.body_rate.x = 0
        self.attitude_raw_setpoint.body_rate.y = 0
        self.attitude_raw_setpoint.body_rate.z = 0
        self.attitude_raw_setpoint.orientation.w = 1
        self.attitude_raw_setpoint.thrust = 

        i = 50
        # print("condition--",(not rospy.is_shutdown()) ,(self.state_indicator.connected) ,(i > 0))
        while (not rospy.is_shutdown()) and (self.state_indicator.connected) and (i > 0):
            self.velocity_setpoint_publisher.publish(self.velocity_setpoint)
            i=i-1
            print("setpoint", i)
            self.rate.sleep()
            if i == 1: 
                return True
        else: 
            return False  

    def onstart_setpoint1(self):
        self.velocity_setpoint.twist.linear.x = 0.0
        self.velocity_setpoint.twist.linear.y = 0.0
        self.velocity_setpoint.twist.linear.z = 0.4

        self.velocity_setpoint.twist.angular.x = 0.0
        self.velocity_setpoint.twist.angular.y = 0.0
        self.velocity_setpoint.twist.angular.z = 0.0

        
        # print("condition--",(not rospy.is_shutdown()) ,(self.state_indicator.connected) ,(i > 0))
        while (not rospy.is_shutdown()) and (self.state_indicator.connected) and (self.z < 3):
            self.velocity_setpoint_publisher.publish(self.velocity_setpoint)
            print("z pose",self.z)
            self.rate.sleep() 
        
        #self.velocity_setpoint.twist.linear.z = 0.0
        #self.velocity_setpoint_publisher.publish(self.velocity_setpoint)
        


    def setpoint_velocity_callback(self) :
        
        # vector_dir = vector_direction.data.split(" ")
        # self.velocity_setpoint.twist.angular.x = 0.0
        # self.velocity_setpoint.twist.angular.y = 0.0
        print("111111")

        self.velocity_setpoint.twist.linear.y = 0 
        self.velocity_setpoint.twist.linear.z = 0
        self.velocity_setpoint.twist.angular.x = 0
        self.velocity_setpoint.twist.angular.y = 0
        self.velocity_setpoint.twist.angular.z = 0

        if self.i % 2 == 0:
            while self.x < 30:
                print("22222")
                self.velocity_setpoint.twist.linear.x = 1 * (30 - self.x)
                self.velocity_setpoint_publisher.publish(self.velocity_setpoint)
                yo.rate.sleep()

        if self.i % 2 == 1:
            while self.x > 0:
                print("22222jjj")
                self.velocity_setpoint.twist.linear.x = -1 * (self.x) 
                self.velocity_setpoint_publisher.publish(self.velocity_setpoint)
                yo.rate.sleep()
                 
        
        
        
        print("44444")
        self.i = self.i + 1
        print(self.i)


if __name__=="__main__":
    yo = rate_controller()
    #rospy.spin()
    while True:
        if yo.onstart_setpoint():
            if yo.change_mode():
                if yo.arming_service():
                    print("COMPLETED")
                    yo.onstart_setpoint1()
                    print("while loop starting")  

                    while not rospy.is_shutdown():
                        yo.setpoint_velocity_callback()
                        print("done")
                        #time.sleep()
                               
                    

        else:
            yo.onstart_setpoint()                    
