# !/usr/bin/env python


import json
import math
from pathlib import Path
import rospy
from geometry_msgs.msg import Twist
import tf2_ros
import tf2_geometry_msgs
from ReliablePoseSelector import QOrientation, ObjectDict
from group6_pkg.msg import ReliablePoseStamped


settings = ObjectDict()
settings_source = Path("../param/group6_params.json")

tfBuffer: tf2_ros.Buffer


class Navigation:
    class Target:
        def __init__(self, x, y, dir, tolerance, min_updates):
            self.x = x
            self.y = y
            self.dir = dir
            self.tolerance = tolerance
            self.min_updates = min_updates

    pub: rospy.Publisher
    watchdog: rospy.Timer
    pose: ReliablePoseStamped
    ordered_path = ""
    path_index = 0
    target: Target(0, 0, 0, 0)
    dist = 0.0
    adist = 0.0
    vel = 0.0
    avel = 0.0
    actual_vel = 0.0
    actual_avel = 0.0

    @staticmethod
    def update(new_pose: ReliablePoseStamped):
        Navigation.watchdog.shutdown()
        Navigation.watchdog = rospy.Timer(
            period=rospy.Duration(new_pose.pose.timeout),
            callback=cb_stop,
            oneshot=True
        )

        Navigation.pose = new_pose

        diff = ObjectDict()
        diff.x = Navigation.target.x - new_pose.pose.position.x
        diff.y = Navigation.target.y - new_pose.pose.position.y

        Navigation.dist = math.sqrt(diff.x ** 2 + diff.y ** 2)

        if Navigation.dist < Navigation.target.tolerance:
            new_target = Navigation.next_target()
            if new_target:
                Navigation.update(new_pose)
            else:
                cb_stop()
            return

        target_angle = math.atan2(diff.y, diff.x)
        adist = target_angle - new_pose.pose.orientation.z
        if adist <= -180:
            adist += 360
        elif adist > 180:
            adist -= 360
        Navigation.adist = adist

        Navigation.vel = min(Navigation.dist / new_pose.pose.timeout, settings.navigation.max_vel)
        Navigation.avel = min(Navigation.adist / new_pose.pose.timeout, settings.navigation.max_vel_angular)

    @staticmethod
    def accel_controller(event_info: rospy.timer.TimerEvent):
        time_to_goal = Navigation.dist / Navigation.vel

        # Acceleration from current speed to desired speed as well as the
        # necessary slowdown from desired speed to 0
        acceleration_time = (abs(Navigation.actual_vel - Navigation.vel) + Navigation.vel) \
                            / settings.navigation.max_acc
        

        cmd = Twist()

        cmd.linear = Navigation.vel
        cmd.angular = math.radians(Navigation.avel)

        Navigation.pub.publish(cmd)

    @staticmethod
    def next_target():
        if Navigation.ordered_path in settings.navigation.paths:
            targets = settings.navigation.paths[Navigation.ordered_path]
            if Navigation.target.index >= len(targets):
                # Reached end of path, wait for further updates.
                # TODO: Publish end of path!
                Navigation.ordered_path = ""
                Navigation.path_index = 0
            else:
                Navigation.path_index += 1
                Navigation.target = Navigation.Target(**targets[Navigation.path_index])
                return True
        return False


def cb_steering():
    pass


def cb_reliable_pose(p: ReliablePoseStamped):
    Navigation.update(p)


def cb_stop(data="dontcare"):
    # It's been too long since the last pose update, stop moving.
    stop = Twist()
    stop.linear = 0
    stop.angular = 0
    Navigation.pub.publish(stop)


def load_settings():
    global settings

    with open(settings_source) as f:
        settings.init_from_dict(json.load(f))

    settings.workstation = settings.workstations[settings.workstations.active]
    Navigation.slowdown_range = (settings.navigation.max_vel ** 2) \
                                / (2 * settings.navigation.max_acc)
    Navigation.slowdown_range_angular = (settings.navigation.max_vel_angular ** 2) \
                                        / (2 * settings.navigation.max_acc_angular)


def main():
    global tfBuffer

    rospy.init_node('ReliablePose')

    load_settings()

    # Must be here because it needs to happen after the node init.
    tfBuffer = tf2_ros.Buffer(cache_time=rospy.Duration(10))
    tfListener = tf2_ros.TransformListener(tfBuffer)

    rospy.Subscriber("/group6/reliable_pose", data_class=ReliablePoseStamped, callback=cb_reliable_pose)
    Navigation.pub = rospy.Publisher("/turtlebot1/cmd_vel", data_class=Twist, queue_size=10)

    # Timer for controlling the acceleration of the turtlebot
    acceleration_control_timer = rospy.Timer(period=rospy.Duration(0.05), callback=Navigation.accel_controller)

    while not rospy.is_shutdown():
        # ATM nothing to do
        break

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
