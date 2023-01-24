# !/usr/bin/env python


import json
import math
from pathlib import Path
import rospy
import tf2_geometry_msgs
import tf2_ros
from tf2_geometry_msgs import PoseStamped  # Not used but the import runs some necessary background code.
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import group6_pkg.msg


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

    def init_from_dict(self, d:dict):
        for k, v in d.items():
            if type(v) in (dict, ObjectDict):
                self[k] = ObjectDict()
                self[k].init_from_dict(v)
            else:
                self[k] = v


def copy_trio(src, dst):
    dst.x = src.x
    dst.y = src.y
    dst.z = src.z


def copy_quat(src, dst):
    dst.x = src.x
    dst.y = src.y
    dst.z = src.z
    dst.w = src.w


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
            x, y, z = self.get_deg().values()
            return f"Euler: x={x:.03} y={y:.03} z={z:.03} [deg]"

        def set_quaternion(self, quat:ObjectDict):
            x, y, z, w = quat.x, quat.y, quat.z, quat.w
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
            e = ObjectDict()
            e.x = math.degrees(self.x)
            e.y = math.degrees(self.y)
            e.z = math.degrees(self.z)
            return e

        def get_rad(self):
            e = ObjectDict()
            copy_trio(src=self, dst=e)
            return e

    def __init__(self):
        self.q = QOrientation.Quaternion()
        self.e = QOrientation.Euler()

    def __repr__(self):
        return f"{self.q.__repr__()} - {self.e.__repr__()}"

    def set_quaternion(self, quat:ObjectDict):
        copy_quat(src=quat, dst=self.q)
        self.e.set_quaternion(quat)

    def set_euler_rad(self, euler):
        copy_trio(src=euler, dst=self.e)
        self.q.set_euler_rad(euler)

    def set_euler_deg(self, euler):
        e = ObjectDict()
        e.x = math.radians(euler.x)
        e.y = math.radians(euler.y)
        e.z = math.radians(euler.z)
        self.set_euler_rad(e)

    def get_euler_deg(self):
        return self.e.get_deg()

    def get_euler_rad(self):
        return self.e.get_rad()

    def get_quaternion(self):
        q = ObjectDict()
        copy_quat(src=self.q, dst=q)
        return q


# Not much to see here...
class Position:
    def __int__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


# Settings and where to get them from
settings_source = Path("../param/group6_params.json").resolve()
settings = ObjectDict()


def load_settings():
    global settings, poses, sources, sources_lookup

    with open(settings_source) as f:
        settings.init_from_dict(json.load(f))

    settings.workstation = settings.workstations[settings.workstations.active]

    ReliablePose.target_frame = settings.navigation.reliable_pose.target_frame
    ReliablePose.publisher = rospy.Publisher(
        name=settings.navigation.reliable_pose.topic,
        data_class=group6_pkg.msg.ReliablePoseStamped,
        queue_size=10
    )

    # Check if settings for all poses are provided
    poses_code = set(poses.keys())
    poses_settings = set(settings.navigation.reliable_pose.sources.keys())

    poses_code, poses_settings = poses_code - poses_settings, poses_settings - poses_code

    if poses_code:
        raise KeyError(f"No settings provided for pose(s) '{poses_code}' (Settings file: '{settings_source}')")
    if poses_settings:
        raise KeyError(f"Unknown pose(s) '{poses_settings}' specified in settings file. (Settings file: '{settings_source}')")

    for src, properties in settings.navigation.reliable_pose.sources.items():
        priority = properties["priority"]
        timeout = properties["timeout"]
        poses[src] = ReliablePose(priority=priority, timeout=timeout)
        sources[src] = priority
        sources_lookup[priority] = src


# Values represent priorities. Since they're also used to distinguish them,
# each source must have a unique value/priority
# Values provided by config file
sources = ObjectDict()
sources.none = 0

# Map the numbers back to the names without having to redefine the list explicitly.
# F.ex.: SOURCES_LOOKUP[0] == "NONE"
sources_lookup = {v: k for k, v in sources.items()}


class ReliablePose:
    # "Static" class variables (Like global vars but without the need to declare them as global in every function)
    best_source = sources.none
    last_source = sources.none
    publisher: rospy.Publisher
    target_frame = "map"
    # Requires listener!! (Cannot be here because it needs to happen hafter the ros node init in main()
    tfBuffer: tf2_ros.Buffer

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
        copy_trio(src=pose.pose.position, dst=self.position)
        self.orientation.set_quaternion(pose.pose.orientation)
        self.last_update = rospy.get_time()
        self.available = True
        self.publish()

    def publish(self):
        # Less reliable, don't use
        if self.priority < ReliablePose.best_source:
            return

        ReliablePose.best_source = self.priority

        if ReliablePose.best_source != ReliablePose.last_source:
            pyrolog("info",
                    f"Switching source from {sources_lookup[ReliablePose.last_source].upper()} "
                    f"to {sources_lookup[ReliablePose.best_source].upper()}. ")
            ReliablePose.last_source = ReliablePose.best_source

        pose = group6_pkg.msg.ReliablePoseStamped()
        pose.header.frame_id    = ReliablePose.target_frame
        pose.header.stamp       = rospy.Time.from_sec(self.last_update)
        pose.pose.timeout       = self.timeout
        pose.pose.source        = sources_lookup[self.priority]

        copy_trio(src=self.position, dst=pose.pose.position)
        copy_trio(src=self.orientation.get_euler_deg(), dst=pose.pose.orientation)

        ReliablePose.publisher.publish(pose)


# So far these are "empty" poses, their properties are loaded in load_settings()
# Still, they're all listed here because they need to match the ones provided in
# load_settings, otherwise either this source or the settings file needs adjustment.
poses = ObjectDict()
poses.odom = ReliablePose()
poses.amcl = ReliablePose()
poses.charuco = ReliablePose()
poses.kinect = ReliablePose()


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

    # The coordinates of this transformation actually contain the
    # pose of the turtlebot relative to the charuco frame.
    tf_tb_base_to_charuco = ReliablePose.tfBuffer.lookup_transform(
        target_frame="charuco",
        source_frame="turtlebot1/base_link",
        time=rospy.Time(0),
        timeout=rospy.Duration(0.1)
    )
    # Convert the transformation to a pose
    pose = PoseStamped()
    pose.header = p.header
    pose.header.frame_id = "charuco"
    copy_trio(src=tf_tb_base_to_charuco.transform.translation,
               dst=pose.pose.position)
    copy_quat(src=tf_tb_base_to_charuco.transform.rotation,
              dst=pose.pose.orientation)

    # Generate the transformation from charuco to world
    # TODO: This one should probably be done once only and ahead of time.
    tf_charuco_to_world = TransformStamped()
    tf_charuco_to_world.header.stamp = p.header.stamp
    tf_charuco_to_world.header.frame_id = settings.navigation.reliable_pose.target_frame
    tf_charuco_to_world.child_frame_id = "charuco"
    copy_trio(src=settings.workstation.positions.charuco.position,
               dst=tf_charuco_to_world.transform.translation)
    copy_quat(src=settings.workstation.positions.charuco.orientation,
              dst=tf_charuco_to_world.transform.rotation)

    # Convert charuco localization to world coordinates
    # using the pose and transform from above
    pose = tf2_geometry_msgs.do_transform_pose(pose=pose, transform=tf_charuco_to_world)

    poses.charuco.update(pose)


def cb_pos_in_kinect(p: PoseStamped):
    global poses
    poses.kinect.update(p)


def check_source_timeout(event_info: rospy.timer.TimerEvent):
    global poses

    t = event_info.current_real.to_sec()
    for name, source in poses.items():
        source: ReliablePose
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
    ReliablePose.best_source = new_best_source


def main():
    rospy.init_node('ReliablePose')

    load_settings()

    # Must be here because it needs to happen after the node init.
    ReliablePose.tfBuffer = tf2_ros.Buffer(cache_time=rospy.Duration(10))
    tfListener = tf2_ros.TransformListener(ReliablePose.tfBuffer)

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
