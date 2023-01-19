# !/usr/bin/env python


import os
import time
import math
# from math import dist, sqrt, sin, cos, pi
from dataclasses import dataclass
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray


class QOrientation:
    # https://stackoverflow.com/questions/53033620/how-to-convert-euler-angles-to-quaternions-and-get-the-same-euler-angles-back-fr

    class Quaternion:
        def __init__(self):
            self.x = 0.00
            self.y = 0.00
            self.z = 1.00
            self.w = 0.00

        def __repr__(self):
            return f"Quaternion: x={self.x:.03} y={self.y:.03} z={self.z:.03} w={self.w:.03}"

        def set_euler_rad(self, z, x=0.0, y=0.0):
            if x or y:
                self.x = math.sin(x / 2) * math.cos(y / 2) * math.cos(z / 2) - math.cos(x / 2) * math.sin(y / 2) * math.sin(z / 2)
                self.y = math.cos(x / 2) * math.sin(y / 2) * math.cos(z / 2) + math.sin(x / 2) * math.cos(y / 2) * math.sin(z / 2)
                self.z = math.cos(x / 2) * math.cos(y / 2) * math.sin(z / 2) - math.sin(x / 2) * math.sin(y / 2) * math.cos(z / 2)
                self.w = math.cos(x / 2) * math.cos(y / 2) * math.cos(z / 2) + math.sin(x / 2) * math.sin(y / 2) * math.sin(z / 2)
            else:
                self.x = 0.00
                self.y = 0.00
                self.z = math.sin(z / 2)
                self.w = math.cos(z / 2)

    class Euler:
        def __init__(self):
            self.x = 0.00
            self.y = 0.00
            self.z = 0.00

        def __repr__(self):
            return f"Euler: x={self.x:.03} y={self.y:.03} z={self.z:.03} [rad]"

        def set_quaternion(self, x, y, z, w):
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            self.x = math.degrees(math.atan2(t0, t1))

            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            self.y = math.degrees(math.asin(t2))

            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            self.z = math.degrees(math.atan2(t3, t4))

    def __init__(self):
        # Quaternion
        self.q = QOrientation.Quaternion()

        # Euler
        self.e = QOrientation.Euler()

    def __repr__(self):
        return f"{self.q.__repr__()} - {self.e.__repr__()}"

    def set_quaternion(self, x, y, z, w):
        self.q.x = x
        self.q.y = y
        self.q.z = z
        self.q.w = w
        self.e.set_quaternion(
            x=x,
            y=y,
            z=z,
            w=w,
        )

    def set_euler_rad(self, z, x=0.0, y=0.0):
        self.e.x = x
        self.e.y = y
        self.e.z = z
        self.q.set_euler_rad(
            x=x,
            y=y,
            z=z,
        )

    def set_euler_deg(self, z, x=0.0, y=0.0):
        self.set_euler_rad(
            x=math.radians(x),
            y=math.radians(y),
            z=math.radians(z),
        )

    def get_euler_deg(self):
        x = math.radians(self.x)
        y = math.radians(self.y)
        z = math.radians(self.z)
        return x, y, z

    def get_euler_rad(self):
        return self.e.x, self.e.y, self.e.z

    def get_quaternion(self):
        return self.q.x, self.q.y, self.q.z, self.q.w


def publish_reliable_odom(pos, orientation):



def cb_pos_in_odom(p):

    return


def cb_pos_in_amcl(p):

    return


def cb_pos_in_charuco(p):

    return


def cb_pos_in_kinect(p):

    return


def main():
    rospy.init_node('iNav')
    rospy.Subscriber("/turtlebot1/odom", data_class=Odometry, callback=cb_pos_in_odom)
    rospy.Subscriber("/turtlebot1/move_base/amcl_pose", data_class=PoseWithCovarianceStamped, callback=cb_pos_in_amcl)
    rospy.Subscriber("/turtlebot1/camera/image_charuco_pose", PoseStamped, callback=cb_pos_in_charuco)
    rospy.Subscriber("/group6/kinect_pose", PoseStamped, callback=cb_pos_in_kinect)

    while not rospy.is_shutdown():
        rospy.sleep(0.05)
        # do something


if __name__ == '__main__':
    os.system("cd ~/jetson_ws/src/jetson/scripts && python3 switch_workstation.py A")
    try:
        ping_pong()
    except rospy.ROSInterruptException:
        pass
