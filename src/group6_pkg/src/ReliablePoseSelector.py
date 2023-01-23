# !/usr/bin/env python


import json
import math
from pathlib import Path
import rospy
import tf2_geometry_msgs
import tf2_ros
from tf2_geometry_msgs import PoseStamped  # Not used but the import runs some necessary background code.
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


# Combined logging for PYthon and ROs.
def pyrolog(level, msg):
    ros_loggers = {
        "debug": rospy.logdebug,
        "info": rospy.loginfo,
        "warning": rospy.logwarn,
        "error": rospy.logerr,
    }

    level = level.lower()
    if level not in ros_loggers:
        pyrolog("Error",
                f"Unknown log level (case insensitive): {level}. Log message: {msg}")

    print(f"{level.title()}: {msg}")
    ros_loggers[level](msg)


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


# Not much to see here...
class Position:
    def __int__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


# Values represent priorities. Since they're also used to distinguish them,
# each source must have a unique value/priority
# Values provided by config file
sources = ObjectDict()
sources.none = 0

# Map the numbers back to the names without having to redefine the list explicitly.
# F.ex.: SOURCES_LOOKUP[0] == "NONE"
sources_lookup = {v: k for k, v in sources.items()}


class Pose:
    # "Static" class variables (Like global vars but without the need to declare them as global in every function)
    best_source = sources.none
    last_source = sources.none
    reliable_pose_publisher = rospy.Publisher('/group6/reliable_pose', data_class=PoseStamped, queue_size=10)
    reliable_pose_frame = "map"
    # Requires listener!! (Cannot be here because it needs to happen hafter the ros node init in main()
    tfBuffer = tf2_ros.Buffer(cache_time=rospy.Duration(10))

    def __init__(self, priority=0, timeout=1.0):
        # Self-explanatory
        self.position = Position()
        self.orientation = QOrientation()
        self.last_update = 0.0
        self.timeout = float(timeout)

        # Low priority data is not published if higher-priority data is available
        self.priority = int(priority)

        # Timeout can cause this to go to false. Automatically set to true if new data is published.
        self.available = False

    def update(self, pose: PoseStamped):
        # Transform the given pose to the desired target frame
        if pose.header.frame_id != Pose.reliable_pose_frame:
            try:
                transform = Pose.tfBuffer.lookup_transform(
                    target_frame=Pose.reliable_pose_frame,
                    source_frame=pose.header.frame_id,
                    time=rospy.Time(0),
                    timeout=rospy.Duration(0.1)
                )
                pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
                # pose = Pose.tfBuffer.transform(
                #     object_stamped=pose,
                #     target_frame=Pose.reliable_pose_frame,
                #     timeout=rospy.Duration(0.1)
                # )
            except Exception as e:
                pyrolog("warning", f"TF Error: {e}")
                return

        self.position.x = pose.pose.position.x
        self.position.y = pose.pose.position.y
        self.position.z = pose.pose.position.z
        self.orientation.set_quaternion(
            x=pose.pose.orientation.x,
            y=pose.pose.orientation.y,
            z=pose.pose.orientation.z,
            w=pose.pose.orientation.w,
        )
        self.last_update = rospy.get_time()
        self.available = True
        self.pub_reliable_pose()

    def pub_reliable_pose(self):
        # Less reliable, don't use
        if self.priority < Pose.best_source:
            return

        Pose.best_source = self.priority

        if Pose.best_source != Pose.last_source:
            pyrolog("info",
                    f"Switching source from {sources_lookup[Pose.last_source].upper()} "
                    f"to {sources_lookup[Pose.best_source].upper()}. ")
            Pose.last_source = Pose.best_source

        reliable_pose = PoseStamped()
        reliable_pose.header.frame_id    = Pose.reliable_pose_frame
        reliable_pose.header.stamp       = rospy.Time.from_sec(self.last_update)
        reliable_pose.pose.position.x    = self.position.x
        reliable_pose.pose.position.y    = self.position.y
        reliable_pose.pose.position.z    = self.position.z
        reliable_pose.pose.orientation.x = self.orientation.q.x
        reliable_pose.pose.orientation.y = self.orientation.q.y
        reliable_pose.pose.orientation.z = self.orientation.q.z
        reliable_pose.pose.orientation.w = self.orientation.q.w

        self.reliable_pose_publisher.publish(reliable_pose)


# So far these are "empty" poses, their properties are loaded in load_settings()
# Still, they're all listed here because they need to match the ones provided in
# load_settings, otherwise either this source or the settings file needs adjustment.
poses = ObjectDict()
poses.odom = Pose()
poses.amcl = Pose()
poses.charuco = Pose()
poses.kinect = Pose()


def load_settings():
    global poses, sources, sources_lookup
    path = Path("../param/group6_params.json").resolve()
    with open(path) as f:
        settings = json.load(f)
    settings = settings["Navigation"]

    Pose.reliable_pose_frame = settings["ReliablePoseFrame"]

    # Check if settings for all poses are provided
    poses_code = set(poses.keys())
    poses_settings = set(settings["PoseSources"].keys())

    poses_code, poses_settings = poses_code - poses_settings, poses_settings - poses_code

    if poses_code:
        raise KeyError(f"No settings provided for pose(s) '{poses_code}' (Settings file: '{path}')")
    if poses_settings:
        raise KeyError(f"Unknown pose(s) '{poses_settings}' specified in settings file. (Settings file: '{path}')")

    for src, properties in settings["PoseSources"].items():
        priority = properties["priority"]
        timeout = properties["timeout"]
        poses[src] = Pose(priority=priority, timeout=timeout)
        sources[src] = priority
        sources_lookup[priority] = src


def cb_pos_in_odom(p: Odometry):
    global poses
    pose = PoseStamped()
    pose.header = p.header
    pose.pose = p.pose.pose
    poses.odom.update(pose)


def cb_pos_in_amcl(p: PoseWithCovarianceStamped):
    global poses
    pose = PoseStamped()
    pose.header = p.header
    pose.pose = p.pose.pose
    poses.amcl.update(pose)


def cb_pos_in_charuco(p: PoseStamped):
    global poses

    # The pose published here actually points from the turtlebot to the
    # fixed (!) charuco marker. To get the position of the turtlebot,
    # set it to 0 (origin = picamera on the turtlebot).

    p.pose.position.x = 0
    p.pose.position.y = 0
    p.pose.position.z = 0

    poses.charuco.update(p)


def cb_pos_in_kinect(p: PoseStamped):
    global poses
    poses.kinect.update(p)


def check_source_timeout(event_info: rospy.timer.TimerEvent):
    global poses

    t = event_info.current_real.to_sec()
    for name, source in poses.items():
        source: Pose
        if not source.available:
            continue
        delta_t = t - source.last_update
        if delta_t >= source.timeout:
            pyrolog("info",
                    f"Timeout on source {name.upper()}! "
                    f"Last update: {int(source.last_update)}, "
                    f"elapsed time: {delta_t:.03}, "
                    f"timeout: {source.timeout:.03}")
            source.available = False
    new_best_source = sources.none
    for source in poses.values():
        if source.available and source.priority > new_best_source:
            new_best_source = source.priority
    Pose.best_source = new_best_source


def main():
    rospy.init_node('ReliablePose')

    load_settings()

    # Must be here because it needs to happen after the node init.
    tfListener = tf2_ros.TransformListener(Pose.tfBuffer)

    rospy.Subscriber("/turtlebot1/odom", data_class=Odometry, callback=cb_pos_in_odom)
    rospy.Subscriber("/turtlebot1/amcl_pose", data_class=PoseWithCovarianceStamped, callback=cb_pos_in_amcl)
    rospy.Subscriber("/turtlebot1/camera/image_charuco_pose", PoseStamped, callback=cb_pos_in_charuco)
    rospy.Subscriber("/group6/kinect_pose", PoseStamped, callback=cb_pos_in_kinect)

    # Timer for checking source timeouts
    source_timeout_timer = rospy.Timer(period=rospy.Duration(0.05), callback=check_source_timeout)

    while not rospy.is_shutdown():
        # ATM nothing to do
        break

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
