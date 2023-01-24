# !/usr/bin/env python


import json
import math
from pathlib import Path
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import tf2_ros
import tf2_geometry_msgs
from ReliablePoseSelector import QOrientation, ObjectDict, pyrolog
from group6_pkg.msg import ReliablePoseStamped


settings = ObjectDict()
settings_source = Path("../param/group6_params.json")

tfBuffer: tf2_ros.Buffer


class Navigation:
    class Target:
        def __init__(self, x, y, dir, tolerance, tolerance_angle, min_updates):
            self.x = x
            self.y = y
            self.dir = dir
            self.tolerance = tolerance
            self.tolerance_angle = tolerance_angle
            self.min_updates = min_updates

    pub: rospy.Publisher
    watchdog: rospy.Timer
    accel_control_timer: rospy.Timer
    pose: ReliablePoseStamped
    ordered_path = ""
    path_index = -1
    target = Target(0, 0, 0, 0, 0, 0)
    dist = 0.0
    adist = 0.0
    vel = 0.0
    avel = 0.0
    actual_vel = 0.0
    actual_avel = 0.0
    accel_control_enable = False

    @staticmethod
    def update(new_pose: ReliablePoseStamped):
        Navigation.watchdog.shutdown()

        # Publishing script must provide an update within the specified time.
        set_watchdog(new_pose.pose.timeout)

        Navigation.pose = new_pose

        if Navigation.path_index < 0:
            # No path to follow; nothing to do.
            return

        diff = ObjectDict()
        diff.x = Navigation.target.x - new_pose.pose.position.x
        diff.y = Navigation.target.y - new_pose.pose.position.y

        Navigation.dist = math.sqrt(diff.x * diff.x + diff.y * diff.y)

        if Navigation.dist < Navigation.target.tolerance:
            pyrolog("Info", f"Within target tolerance ({Navigation.dist:.3f}m < {Navigation.target.tolerance:.3f}m).")
            new_target = Navigation.next_target()
            if new_target:
                Navigation.update(new_pose)
                Navigation.accel_control_enable = True
            else:
                cb_stop()
            return

        target_angle = math.degrees(math.atan2(diff.y, diff.x))

        if Navigation.target.dir < 0:
            pyrolog("Info", "Driving backwards. ")
            target_angle += 180

        adist = target_angle - new_pose.pose.orientation.z
        if adist <= -180:
            adist += 360
        elif adist > 180:
            adist -= 360
        Navigation.adist = adist

        if abs(adist) >= Navigation.target.tolerance_angle:
            pyrolog("Info", f"Angle too far off: {adist:.1f} > {Navigation.target.tolerance_angle:.1f}. "
                            f"Blocking translation.")
            # Angle too far off; don't drive into the wrong direction. 
            vel = 0.0
            avel = Navigation.adist / (new_pose.pose.timeout * Navigation.target.min_updates)
        else:
            pyrolog("Info", f"Angle ok. Driving. Remaining dist: {Navigation.dist:.3f}m")
            vel = Navigation.dist / (new_pose.pose.timeout * Navigation.target.min_updates)
            avel = 0

        Navigation.vel = vel * Navigation.target.dir
        Navigation.avel = avel

    @staticmethod
    def accel_controller(target_vel, actual_vel, max_vel, min_vel, max_acc, dist_to_goal, delta_t):
        # Desired speed:
        vel = target_vel

        if abs(vel) > abs(max_vel):
            vel = math.copysign(max_vel, vel)

        # Distance required to slow down to 0.
        stop_dist = (actual_vel ** 2) / (2 * max_acc)

        # Some safety margin
        stop_dist *= 1.05

        # Time to slow down
        if abs(dist_to_goal) <= stop_dist:
            pyrolog("Info", f"In stop range. dist: {dist_to_goal:.3f}, stop_dist: {stop_dist:.3f}")
            vel = 0

        # Accelerate in discrete steps whose size is determined by the call frequency of this function
        delta_v = delta_t * max_acc

        # Make sure we don't accelerate further than desired
        delta_v = min(delta_v, abs(vel - actual_vel))

        # Acceleration direction
        if actual_vel > vel:
            delta_v = -delta_v

        # Apply acceleration. No further checks needed (in theory)
        actual_vel += delta_v

        # Maintain a minimum speed til the goal is reached. Minimum speed is expected
        # to allow "instantaneous" stopping without controlling the acceleration.
        # However, do not move if value equals 0 (a.k.a. we shall stop).
        if vel and abs(actual_vel) <= min_vel:
            # pyrolog("Info", f"Actual velocity lower than min: {actual_vel:.3f}. Maintaining min of {min_vel:.3f}.")
            actual_vel = math.copysign(min_vel, actual_vel)

        return actual_vel

    @staticmethod
    def next_target():
        if Navigation.ordered_path in settings.navigation.paths:
            targets = settings.navigation.paths[Navigation.ordered_path]
            i = Navigation.path_index + 1
            if i >= len(targets):
                # Reached end of path, wait for further updates.
                # TODO: Publish end of path!
                pyrolog("Info", f"End of path '{Navigation.ordered_path}' reached.")
                Navigation.path_index = -1
                Navigation.ordered_path = ""
            else:
                pyrolog("Info", f"New target: {Navigation.ordered_path}[{i}]")
                Navigation.target = Navigation.Target(**targets[i])
                Navigation.path_index = i
                return True
        return False


def cb_accel_control(event_info: rospy.timer.TimerEvent):
    if not event_info.last_real:
        # First run
        return

    if not Navigation.accel_control_enable:
        return

    delta_t = (event_info.current_real-event_info.last_real).to_sec()

    Navigation.actual_vel = Navigation.accel_controller(
        target_vel=Navigation.vel,
        actual_vel=Navigation.actual_vel,
        max_vel=settings.navigation.max_vel,
        min_vel=settings.navigation.min_vel,
        max_acc=settings.navigation.max_acc,
        dist_to_goal=Navigation.dist,
        delta_t=delta_t,
    )

    Navigation.actual_avel = Navigation.accel_controller(
        target_vel=Navigation.avel,
        actual_vel=Navigation.actual_avel,
        max_vel=settings.navigation.max_vel_angular,
        min_vel=settings.navigation.min_vel_angular,
        max_acc=settings.navigation.max_acc_angular,
        dist_to_goal=Navigation.adist,
        delta_t=delta_t,
    )

    vel_rad = math.radians(Navigation.actual_avel)

    cmd = Twist()
    cmd.linear.x = Navigation.actual_vel
    cmd.linear.y = 0
    cmd.linear.z = 0
    cmd.angular.x = 0
    cmd.angular.y = 0
    cmd.angular.z = vel_rad

    Navigation.pub.publish(cmd)


def cb_reliable_pose(p: ReliablePoseStamped):
    Navigation.update(p)


def cb_stop(data="dontcare"):
    # It's been too long since the last pose update, stop moving.
    Navigation.accel_control_enable = False
    stop = Twist()
    stop.linear.x = 0
    stop.linear.y = 0
    stop.linear.z = 0
    stop.angular.x = 0
    stop.angular.y = 0
    stop.angular.z = 0
    Navigation.pub.publish(stop)


def cb_path(data: String):
    if data.data in settings.navigation.paths:
        Navigation.ordered_path = data.data
        Navigation.next_target()
    else:
        raise IndexError(f"Desired path '{data.data}' unknown. ")


def set_watchdog(timeout = 10):
    Navigation.watchdog = rospy.Timer(
        period=rospy.Duration(timeout),
        callback=cb_stop,
        oneshot=True
    )


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

    rospy.init_node('unified_navigation')

    load_settings()

    # Must be here because it needs to happen after the node init.
    tfBuffer = tf2_ros.Buffer(cache_time=rospy.Duration(10))
    tfListener = tf2_ros.TransformListener(tfBuffer)

    rospy.Subscriber("/group6/reliable_pose", data_class=ReliablePoseStamped, callback=cb_reliable_pose)
    rospy.Subscriber("/group6/tb_path", data_class=String, callback=cb_path)
    Navigation.pub = rospy.Publisher("/turtlebot1/cmd_vel", data_class=Twist, queue_size=10)

    # Timer for controlling the acceleration of the turtlebot
    Navigation.accel_control_timer = rospy.Timer(period=rospy.Duration(0.05), callback=cb_accel_control, oneshot=False)

    # Watchdog in case there's no pose update anymore.
    set_watchdog()

    # Make sure nothing is running!
    cb_stop()

    while not rospy.is_shutdown():
        # Nothing to do here atm
        break

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
