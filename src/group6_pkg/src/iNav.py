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


# Handles orientations in both Euler and Quaternion form and provides conversion between both.
# Supports Euler in degrees and radians.
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
            x, y, z = self.get_deg()
            return f"Euler: x={x:.03} y={y:.03} z={z:.03} [deg]"

        def set_quaternion(self, x, y, z, w):
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            self.x = math.atan2(t0, t1)

            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            self.y = math.asin(t2)

            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            self.z = math.atan2(t3, t4)

        def get_deg(self):
            x = math.degrees(self.x)
            y = math.degrees(self.y)
            z = math.degrees(self.z)
            return x, y, z

        def get_rad(self):
            return self.x, self.y, self.z

    def __init__(self):
        self.q = QOrientation.Quaternion()
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
        return self.e.get_deg()

    def get_euler_rad(self):
        return self.e.get_rad()

    def get_quaternion(self):
        return self.q.x, self.q.y, self.q.z, self.q.w


class Position:
    def __int__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class Pose:
    def __int__(self):
        self.position = Position()
        self.orientation = QOrientation()
        self.last_update = 0


# Values represent priorities. Since they're also used to distinguish them,
# each source must have a unique value/priority
@dataclass
class POSE_SOURCES:
    NONE    = 0
    ODOM    = 1
    AMCL    = 2
    CHARUCO = 3
    KINECT  = 4


# Global Variables
pose_best_source = POSE_SOURCES.NONE

pose_odom    = Pose()
pose_amcl    = Pose()
pose_charuco = Pose()
pose_kinect  = Pose()


def pub_reliable_pose(source):
    global pose_best_source, pose_odom, pose_amcl, pose_charuco, pose_kinect

    # Less reliable, don't use
    if source < pose_best_source:
        return

    pose_best_source = source

    if pose_best_source == POSE_SOURCES.KINECT:
        pose_source = pose_kinect
    elif pose_best_source == POSE_SOURCES.CHARUCO:
        pose_source = pose_charuco
    elif pose_best_source == POSE_SOURCES.AMCL:
        pose_source = pose_amcl
    else: # pose_best_source == POSE_SOURCES.ODOM:
        pose_source = pose_odom

    reliable_pose = PoseStamped()
    reliable_pose.header.frame_id = "map"
    reliable_pose.pose.position.x = pose_source.position.x
    reliable_pose.pose.position.y = pose_source.position.y
    reliable_pose.pose.position.z = pose_source.position.z
    reliable_pose.pose.orientation.x = pose_source.orientation.q.x
    reliable_pose.pose.orientation.y = pose_source.orientation.q.y
    reliable_pose.pose.orientation.z = pose_source.orientation.q.z
    reliable_pose.pose.orientation.w = pose_source.orientation.q.w

    publisher = rospy.Publisher('/group6/reliable_pose', data_class=PoseStamped, queue_size=10)
    publisher.publish(reliable_pose)



def cb_pos_in_odom(p):
    global pose_best_source, pose_odom
    pose_odom.last_update = rospy.get_time()
    pose_odom = p
    pub_reliable_pose(POSE_SOURCES.ODOM)


def cb_pos_in_amcl(p):
    global pose_best_source, pose_amcl
    pose_amcl.last_update = rospy.get_time()
    pose_amcl = p
    pub_reliable_pose(POSE_SOURCES.AMCL)


def cb_pos_in_charuco(p):
    global pose_best_source, pose_charuco
    pose_charuco.last_update = rospy.get_time()
    pose_charuco = p
    pub_reliable_pose(POSE_SOURCES.CHARUCO)


def cb_pos_in_kinect(p):
    global pose_best_source, pose_kinect
    pose_kinect.last_update = rospy.get_time()
    pose_kinect = p
    pub_reliable_pose(POSE_SOURCES.KINECT)


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
    try:
        main()
    except rospy.ROSInterruptException:
        pass
