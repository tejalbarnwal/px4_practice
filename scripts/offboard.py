#!/usr/bin/env python

from __future__ import print_function
import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest,CommandBoolResponse, SetMode , SetModeRequest, SetModeResponse
from geometry_msgs.msg import PoseStamped


class switch_to_offboard():
    def __init__(self):
        rospy.init_node("offboard_node")

        # subscriber to /mavros/state
        rospy.Subscriber("/mavros/state" ,State, self.state_callback)
        
        self.local_setpoint_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped , queue_size=1)

        self.setpoint = PoseStamped()
        self.state_indicator = State()
        self.rate = rospy.Rate(10)
        self.set_mode = SetModeRequest()
        self.isarmed = CommandBoolRequest()
        self.arming_response = CommandBoolResponse()
        self.set_mode_response = SetModeResponse()
        self.setpoints_given = False


    def arming_service(self):
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

    def initial_setpoints(self):
        self.setpoint.pose.position.x = 0
        self.setpoint.pose.position.y = 0
        self.setpoint.pose.position.z = 3.0
        self.setpoint.pose.orientation.x = 0
        self.setpoint.pose.orientation.y = 0
        self.setpoint.pose.orientation.z = 0
        self.setpoint.pose.orientation.w = 1

        i = 100
        print("condition--",(not rospy.is_shutdown()) ,(self.state_indicator.connected) ,(i > 0))
        while (not rospy.is_shutdown()) and (self.state_indicator.connected) and (i > 0):
            self.local_setpoint_pub.publish(self.setpoint)
            i=i-1
            print("setpoint", i)
            self.rate.sleep()
            if i == 1:    
                return True
        

        
    def state_callback(self,msg):
        print("Current mode of the drone" , msg.mode)
        # print("Is MAVROS connected to SITL", msg.connected)
        self.state_indicator = msg

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




if __name__=="__main__":
    yo = switch_to_offboard()
    a =True
    while a:
        if yo.initial_setpoints():
            a = False
            if yo.change_mode():
                if yo.arming_service():
                    print("COMPLETED")
                    b = True
                    while b:
                        yo.initial_setpoints()
                        b =False
                            #print("done")

                            

          
