# !/usr/bin/env python


import json
import math
from pathlib import Path
import rospy
from std_msgs.msg import String
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

        if Navigation.target.dir < 0:
            target_angle += 180

        adist = target_angle - new_pose.pose.orientation.z
        if adist <= -180:
            adist += 360
        elif adist > 180:
            adist -= 360
        Navigation.adist = adist

        if adist >= 10:
            # Angle too far off; don't drive into the wrong direction. 
            vel = 0.0
        else:
            vel = Navigation.dist / (new_pose.pose.timeout * Navigation.target.min_updates)
        avel = Navigation.adist / (new_pose.pose.timeout * Navigation.target.min_updates)

        Navigation.vel = min(vel, settings.navigation.max_vel) * Navigation.target.dir
        Navigation.avel = min(avel, settings.navigation.max_vel_angular)

    @staticmethod
    def accel_controller(target_vel, actual_vel, max_vel, min_vel, max_acc, dist_to_goal, delta_t):
        # Desired speed:
        vel = target_vel

        # Distance required to slow down to 0.
        stop_dist = (actual_vel ** 2) / (2 * max_acc)

        # Some safety margin
        stop_dist *= 1.05

        # Time to slow down
        if dist_to_goal <= stop_dist:
            vel = 0

        # Accelerate in discrete steps whose size is determined by the call frequency of this function
        delta_v = delta_t * max_acc

        if actual_vel > vel:
            actual_vel += delta_v
        else:
            actual_vel -= delta_v

        # Maintain a minimum speed til the goal is reached. Minimum speed is expected
        # to allow "instantaneous" stopping without controlling the acceleration.
        if abs(actual_vel) <= min_vel:
            if actual_vel < 0:
                actual_vel = -min_vel
            else:
                actual_vel = min_vel

        return actual_vel


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


def cb_accel_control(event_info: rospy.timer.TimerEvent):
    cmd = Twist()

    cmd.linear = Navigation.accel_controller(
        target_vel=Navigation.vel,
        actual_vel=Navigation.actual_vel,
        max_vel=settings.navigation.max_vel,
        min_vel=settings.navigation.min_vel,
        max_acc=settings.navigation.max_acc,
        dist_to_goal=Navigation.dist,
        delta_t=(event_info.current_real-event_info.last_real),
    )

    cmd.angular = Navigation.accel_controller(
        target_vel=Navigation.avel,
        actual_vel=Navigation.actual_avel,
        max_vel=settings.navigation.max_vel_angular,
        min_vel=settings.navigation.min_vel_angular,
        max_acc=settings.navigation.max_acc_angular,
        dist_to_goal=Navigation.adist,
        delta_t=(event_info.current_real-event_info.last_real),
    )

    cmd.angular = math.radians(cmd.angular)

    Navigation.pub.publish(cmd)


def cb_reliable_pose(p: ReliablePoseStamped):
    Navigation.update(p)


def cb_stop(data="dontcare"):
    # It's been too long since the last pose update, stop moving.
    stop = Twist()
    stop.linear = 0
    stop.angular = 0
    Navigation.pub.publish(stop)


def cb_path(data: String):
    if data.data in settings.navigation.paths:
        Navigation.ordered_path = data.data
    else:
        raise IndexError(f"Desired path '{data.data}' unknown. ")


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
    rospy.Subscriber("/group6/tb_path", data_class=String, callback=cb_path)
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
