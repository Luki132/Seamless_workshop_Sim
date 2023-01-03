#!/usr/bin/env python

from __future__ import print_function
from group_messages.srv import order_cube1
import rospy

def add_two_ints_client(order):
    rospy.wait_for_service('uarm3_controll/move')
    Uarm2_client = rospy.ServiceProxy('uarm3_controll/move', order_cube1)
    result = Uarm2_client(order)
    print("The result of the Service is: " + str(result))
    pass



if __name__ == "__main__":
    order = order_cube1.request_class(storage1=True, storage2=False, storage3=False)
    add_two_ints_client(order)