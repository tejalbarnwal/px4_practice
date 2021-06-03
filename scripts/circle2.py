#!/usr/bin/env python

from __future__ import print_function
import rospy
from mavros_msgs.msg import State , AttitudeTarget , PositionTarget
from mavros_msgs.srv import CommandBool , CommandBoolRequest , CommandBoolResponse ,SetMode , SetModeRequest , SetModeResponse
from tf.transformations import quaternion_from_euler , euler_from_quaternion
from geometry_msgs.msg import PoseStamped , TwistStamped , Twist
import time
from std_msgs.msg import String ,Float64 , Float32
import math

class rate_controller():
    def __init__(self):
        rospy.init_node("rate_controller_node")

        # subscribers
        rospy.Subscriber("/mavros/state", State , self.state_callback, queue_size = 1)
        rospy.Subscriber("/mavros/local_position/pose" , PoseStamped, self.pose_callback , queue_size =1 )

        rospy.Subscriber("/mavros/local_position/velocity_body", TwistStamped , self.vel_callback , queue_size=1)


        rospy.Subscriber("/teju/roll_pub" , Float32 , self.roll_rate_call , queue_size = 1)
        rospy.Subscriber("/teju/pitch_pub" , Float32 , self.pitch_rate_call , queue_size = 1)
        rospy.Subscriber("/teju/yaw_pub" , Float32 , self.yaw_rate_call , queue_size = 1)



        # publishers
        self.attitude_raw_setpoint_publisher = rospy.Publisher("/mavros/setpoint_raw/attitude" , AttitudeTarget , queue_size = 1)
        self.local_raw_publisher = rospy.Publisher("/mavros/setpoint_raw/local" , PositionTarget , queue_size =1)
        self.local_setpoint_publisher = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped , queue_size=1)
        self.velocity_setpoint_publisher = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel" , TwistStamped , queue_size=1)
        self.velocity_unstamp_setpoint_publisher = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped" , Twist , queue_size=1)



        self.roll_pub = rospy.Publisher("/setpoint/roll" , Float32 , queue_size = 1)
        self.pitch_pub = rospy.Publisher("/setpoint/pitch" , Float32 , queue_size = 1)
        self.yaw_pub = rospy.Publisher("/setpoint/yaw" , Float32 , queue_size = 1)

        self.actual_roll_pub = rospy.Publisher("/actual/roll" , Float32 , queue_size = 1)
        self.actual_pitch_pub = rospy.Publisher("/actual/pitch" , Float32 , queue_size = 1)
        self.actual_yaw_pub = rospy.Publisher("/actual/yaw" , Float32 , queue_size = 1)

        self.error_roll_pub = rospy.Publisher("/error/roll" , Float32 , queue_size = 1)
        self.error_pitch_pub = rospy.Publisher("/error/pitch" , Float32 , queue_size = 1)
        self.error_yaw_pub = rospy.Publisher("/error/yaw" , Float32 , queue_size = 1)



        # VARIABLE TO CHECKOUT
        self.velocity_setpoint = TwistStamped()
        self.attitude_raw_setpoint = AttitudeTarget()
        self.local_raw_setpoint = PositionTarget()
        self.pose_setpoint = PoseStamped()
        self.velocity_unstamp_setpoint = Twist()
        


        self.state_indicator = State()

        self.rate = rospy.Rate(20)

        self.set_mode = SetModeRequest()

        self.isarmed = CommandBoolRequest()
        self.arming_response = CommandBoolResponse()

        self.set_mode_response = SetModeResponse()

        self.i = 0
        self.z = 0.0
        self.x = 0.0
        self.prev_z = 0.0
        self.integral = 0.0

        self.roll_rate = 0.0
        self.pitch_rate = 0.0
        self.yaw_rate = 0.0

        self.actual_vel = TwistStamped()

    def vel_callback(self, msg):
        self.actual_vel = msg    

    def state_callback(self , msg):
        self.state_indicator = msg
        # print("is mavros connected to sitl" , msg.connected)


    def pose_callback(self , msg):
        
        self.prev_z = self.z
        self.z = msg.pose.position.z   
        self.x = msg.pose.position.x 
        # self.prev_z = prev_z   


    def roll_rate_call(self , msg):
        self.roll_rate = msg.data


    def yaw_rate_call(self , msg):
        self.yaw_rate = msg.data


    def pitch_rate_call(self , msg):
        self.pitch_rate = msg.data


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


    def onstart_setpoint1(self):

        self.pose_setpoint.pose.position.x = 0
        self.pose_setpoint.pose.position.y = 0
        self.pose_setpoint.pose.position.z = 3.0
        self.pose_setpoint.pose.orientation.x = 0
        self.pose_setpoint.pose.orientation.y = 0
        self.pose_setpoint.pose.orientation.z = 0
        self.pose_setpoint.pose.orientation.w = 1

        i = 50
        # print("condition--",(not rospy.is_shutdown()) ,(self.state_indicator.connected) ,(i > 0))
        while (not rospy.is_shutdown()) and (self.state_indicator.connected) and (i > 0):
            self.local_setpoint_publisher.publish(self.pose_setpoint)
            i=i-1
            print("setpoint", i)
            self.rate.sleep()
            if i == 1: 
                return True
        else: 
            return False   

    def onstart_setpoint2(self):

        self.velocity_unstamp_setpoint.linear.x = 0
        self.velocity_unstamp_setpoint.linear.y = 0
        self.velocity_unstamp_setpoint.linear.z = 0.0
        self.velocity_unstamp_setpoint.angular.x = 0
        self.velocity_unstamp_setpoint.angular.y = 0
        self.velocity_unstamp_setpoint.angular.z = 0
     


        i = 50
        # print("condition--",(not rospy.is_shutdown()) ,(self.state_indicator.connected) ,(i > 0))
        while (not rospy.is_shutdown()) and (self.state_indicator.connected) and (i > 0):
            self.velocit_unstamp_setpoint_publisher.publish(self.velocity_unstamp_setpoint)
            i=i-1
            print("setpoint", i)
            self.rate.sleep()
            if i == 1: 
                return True
        else: 
            return False           


    def onstart_setpoint3(self):

        self.attitude_raw_setpoint.header.stamp = rospy.Time.now()
        self.attitude_raw_setpoint.type_mask = 128

        self.attitude_raw_setpoint.body_rate.x = 0.0
        self.attitude_raw_setpoint.body_rate.y = 0.0
        self.attitude_raw_setpoint.body_rate.z = 0.0

    
        self.attitude_raw_setpoint.thrust = 0.7
     


        # i = 50
        # print("condition--",(not rospy.is_shutdown()) ,(self.state_indicator.connected) ,(i > 0))
        # while (not rospy.is_shutdown()) and (self.state_indicator.connected) and (i > 0):
        self.attitude_raw_setpoint_publisher.publish(self.attitude_raw_setpoint)
        #     i=i-1
        #     print("setpoint", i)
        #     self.rate.sleep()
        #     if i == 1: 
        #         return True
        # else: 
        #     return False    


    def circle(self):
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.coordinate_frame = 8
        msg.type_mask = 2019

        #msg.position.x = 0.0
        #msg.position.y = 0.0
        msg.position.z = -3.0

        msg.velocity.x = 1.0
        msg.velocity.y = 0
        # msg.velocity.z = 0.01

        # msg.acceleration_or_force.x = -0.02
        # msg.acceleration_or_force.y = 0
        # msg.acceleration_or_force.z = 0
        msg.yaw_rate = 1.0
        self.local_raw_publisher.publish(msg)


    def circle_velocity_unstamp_setpoint(self):
        self.velocity_unstamp_setpoint.linear.x = 1
        self.velocity_unstamp_setpoint.linear.y = 0
        self.velocity_unstamp_setpoint.linear.z = 0.2
        self.velocity_unstamp_setpoint.angular.x = 0
        self.velocity_unstamp_setpoint.angular.y = 0
        self.velocity_unstamp_setpoint.angular.z = 1

        self.velocity_unstamp_setpoint_publisher.publish(self.velocity_unstamp_setpoint)


    def try_traj(self):
        p = 0.5 # 0.05
        d = 0.4  # 0.5 , 0.9
        i = 0.2
        time = 0.05

        self.attitude_raw_setpoint.header.stamp = rospy.Time.now()
        self.attitude_raw_setpoint.type_mask = 128

        self.attitude_raw_setpoint.body_rate.x = self.roll_rate
        self.attitude_raw_setpoint.body_rate.y = self.pitch_rate
        self.attitude_raw_setpoint.body_rate.z = self.yaw_rate

        self.integral += ( ( 3-self.z ) * time )
        self.attitude_raw_setpoint.thrust = p * (3 - self.z ) - d * ((self.z - self.prev_z) / time) + i * (self.integral)
        print(self.attitude_raw_setpoint.thrust ,"----" , self.z)

        self.attitude_raw_setpoint_publisher.publish(self.attitude_raw_setpoint)
        self.roll_pub.publish(self.roll_rate)
        self.pitch_pub.publish(self.pitch_rate)    
        self.yaw_pub.publish(self.yaw_rate) 

        self.actual_roll_pub.publish(self.actual_vel.twist.angular.x)
        self.actual_pitch_pub.publish(self.actual_vel.twist.angular.y)
        self.actual_yaw_pub.publish(self.actual_vel.twist.angular.z)

        self.error_roll_pub.publish(self.roll_rate - self.actual_vel.twist.angular.x)
        self.error_pitch_pub.publish(self.pitch_rate - self.actual_vel.twist.angular.y)   
        self.error_yaw_pub.publish(self.yaw_rate - self.actual_vel.twist.angular.z)   




if __name__=="__main__":
    yo = rate_controller()
    
    while not rospy.is_shutdown():
        print("send points")
        for i in range(100):
            yo.onstart_setpoint3()
            yo.rate.sleep()
        if yo.change_mode():
            print("mode_changed")
            yo.arming_service()
            while True:
                print("starting with code")
                yo.try_traj()
                yo.rate.sleep()
        


    '''
    #rospy.spin()
    while True:
        if yo.onstart_setpoint():
            if yo.change_mode():
                if yo.arming_service():
                    print("COMPLETED")
                    yo.onstart_setpoint()
                    print("while loop starting")  

                    while not rospy.is_shutdown():
                        yo.local_raw_function()
                        yo.attitude_raw_publisher.publish(yo.attitude_raw_setpoint)

                        print("r-" ,yo.attitude_raw_setpoint.body_rate.x)
                        print("p-" ,yo.attitude_raw_setpoint.body_rate.y)
                        print("y-" ,yo.attitude_raw_setpoint.body_rate.z)
                        # print("th-" ,yo.attitude_raw_setpoint.thrust)
                        yo.rate.sleep()
                               
                    

        else:
            yo.onstart_setpoint() '''             
