#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
    # values at 0 degree
    print("at 0 degree (TB front):", msg.ranges[0])

    # values at 90 degree
    print("at 90 degree (TB left):", msg.ranges[89])

    # values at 180 degree
    print("at 180 degree (TB rear):", msg.ranges[179])

    # values at 270 degree
    print("at 270 degree (TB right):", msg.ranges[269])

    print("------------------------------")

rospy.init_node('scan_values')
sub = rospy.Subscriber('/turtlebot1/scan', LaserScan, scan_callback)
rospy.spin()