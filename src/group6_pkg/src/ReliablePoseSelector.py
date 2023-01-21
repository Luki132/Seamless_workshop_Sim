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


# Enhanced version of the built-in dict class that allows
# access to elements using .name syntax:
#   od["something"] == od.something
# Source: https://goodcode.io/articles/python-dict-object/
class ObjectDict(dict):
    def __getattr__(self, name):
        if name in self:
            return self[name]
        else:
            raise AttributeError("No such attribute: " + name)

    def __setattr__(self, name, value):
        self[name] = value

    def __delattr__(self, name):
        if name in self:
            del self[name]
        else:
            raise AttributeError("No such attribute: " + name)


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
    # Values represent priorities. Since they're also used to distinguish them,
    # each source must have a unique value/priority
    @dataclass
    class SOURCES:
        NONE = 0
        ODOM = 1
        AMCL = 2
        CHARUCO = 3
        KINECT = 4

    # "Static" class variables
    best_source = SOURCES.NONE
    reliable_pose_publisher = rospy.Publisher('/group6/reliable_pose', data_class=PoseStamped, queue_size=10)

    def __init__(self, priority=0, timeout=1.0):
        # Self-explanatory
        self.position = Position()
        self.orientation = QOrientation()
        self.last_update = 0.0
        self.timeout = timeout

        # Low priority data is not published if higher-priority data is available
        self.priority = priority

        # Timeout can cause this to go to false. Automatically set to true if new data is published.
        self.available = False

    def update(self, position=Position(), orientation=QOrientation.Quaternion()):
        # This works for all objects that have the right parameters (x,y,z - x,y,z,w)
        # TODO: Think about error handling.
        self.position.__dict__.update(position.__dict__)
        self.orientation.__dict__.update(orientation.__dict__)
        self.last_update = rospy.get_time()
        self.available = True
        self.pub_reliable_pose()

    def pub_reliable_pose(self):
        # Less reliable, don't use
        if self.priority < Pose.best_source:
            return

        Pose.best_source = self.priority

        reliable_pose = PoseStamped()
        reliable_pose.header.frame_id = "map"
        reliable_pose.pose.position.__dict__.update(self.position.__dict__)
        reliable_pose.pose.orientation.__dict__.update(self.orientation.q.__dict__)

        self.reliable_pose_publisher.publish(reliable_pose)


poses = ObjectDict()
poses.odom    = Pose(priority=Pose.SOURCES.ODOM)
poses.amcl    = Pose(priority=Pose.SOURCES.AMCL)
poses.charuco = Pose(priority=Pose.SOURCES.CHARUCO)
poses.kinect  = Pose(priority=Pose.SOURCES.KINECT)


def cb_pos_in_odom(p: Odometry):
    global poses
    poses.odom.update(position=p.pose.pose.position, orientation=p.pose.pose.orientation)


def cb_pos_in_amcl(p: PoseWithCovarianceStamped):
    global poses
    poses.amcl.update(position=p.pose.pose.position, orientation=p.pose.pose.orientation)


def cb_pos_in_charuco(p: PoseStamped):
    global poses
    poses.charuco.update(position=p.pose.position, orientation=p.pose.orientation)


def cb_pos_in_kinect(p: PoseStamped):
    global poses
    poses.kinect.update(position=p.pose.position, orientation=p.pose.orientation)


def check_source_timeout():
    global poses

    t = rospy.get_time()
    for pose in poses.values():
        pose: Pose
        if pose.last_update >= pose.timeout:
            pose.available = False
    new_best_source = Pose.SOURCES.NONE
    for pose in poses.values():
        if pose.available and pose.priority > new_best_source:
            new_best_source = pose.priority


def main():
    rospy.init_node('ReliablePose')
    rospy.Subscriber("/turtlebot1/odom", data_class=Odometry, callback=cb_pos_in_odom)
    rospy.Subscriber("/turtlebot1/move_base/amcl_pose", data_class=PoseWithCovarianceStamped, callback=cb_pos_in_amcl)
    rospy.Subscriber("/turtlebot1/camera/image_charuco_pose", PoseStamped, callback=cb_pos_in_charuco)
    rospy.Subscriber("/group6/kinect_pose", PoseStamped, callback=cb_pos_in_kinect)

    while not rospy.is_shutdown():
        rospy.sleep(0.05)
        check_source_timeout()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
