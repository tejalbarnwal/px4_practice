#!/usr/bin/env python

from __future__ import print_function

from px4_try.srv import add_two_ints, add_two_intsResponse
import rospy


def handle_add_two_ints(req):
    print(f'retuning {req.a} + {req.b} = {req.a + req.b}')
    return add_two_intsResponse(req.a + req.b)


def add_two_ints_server():
    rospy.init_node("add_two_ints_server")
    s = rospy.Service("add_two_ints", add_two_ints , handle_add_two_ints)

    # this decalares a new service named add_two_ints with the add_two_ints
    # service type. All requests are passes to handle_add_two_ints function
    # handle_add_two_ints is called with instances of add_two_intsRequest
    # and returns instances of add_two_intsResponse.
    
    print("ready to add two ints")
    rospy.spin()

if __name__="__main__":
    add_two_ints_server()    

