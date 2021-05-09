#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from px4_try.srv import add_two_ints,add_two_intsRequest,add_two_intsResponse

def add_two_ints_client(x,y):
    rospy.wait_for_service('add_two_ints')
    # for clients u do not hv to call init_node()
    # method that blocks until service is available
    try:
        add_two_int_yo = rospy.ServiceProxy('add_two_ints',add_two_ints)
        # handle for calling the service
        resp1 = add_two_int_yo(x ,y)
        return resp1.sum
    except rospy.ServiceException as e:
        print("service call failed {}".format(e))

def usage():
    return str(sys.argv[0])


if __name__ == '__main__':
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print(usage)
        sys.exit(1)

    print('requesting {} + {}'.format(x,y))
    print('{} + {} = {}'.format(x,y,add_two_ints_client(x,y)))        