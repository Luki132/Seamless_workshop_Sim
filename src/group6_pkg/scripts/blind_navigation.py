#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan



velocity_publisher: rospy.Publisher


# laser_points = [0,0,0,0] # 0, 90, 180, 270
#
#
# def scan_callback(msg: LaserScan):
#     global laser_points
#
#     laser_points[0] = msg.ranges[0]
#     laser_points[1] = msg.ranges[89]
#     laser_points[2] = msg.ranges[179]
#     laser_points[3] = msg.ranges[269]
#
#
# def print_current_pos():
#     global laser_points, velocity_publisher
#     t0 = rospy.get_time()
#
#     vel_msg = Twist()
#
#     velocity_publisher.publish(vel_msg)
#     current_time = 0
#
#     while(current_time < (1)):
#         t1 = rospy.get_time()
#         current_time = t1-t0
#
#     print(f"Laser Pos:")
#     print(f"    0: {laser_points[0]}")
#     print(f"   90: {laser_points[1]}")
#     print(f"  180: {laser_points[2]}")
#     print(f"  270: {laser_points[3]}")


class Laser:
    LOCK_COUNT = 2

    def __init__(self):
        self.angle = 0
        self.last  = 0.0
        self.dist  = 0.0
        self.diff  = 0.0
        self.dir = "f"
        self.speed_lin = 0.05
        self.speed_rot = math.radians(10)
        self.check = False
        self.reaction_lock = Laser.LOCK_COUNT
        self.speed_corr = 1.0
        self.dir_increase = "f"
        self.angle_spread = 2

    def opposite_dir(self, dir: str):
        return {"f": "b", "b": "f", "l": "r", "r": "l"}[dir[0].lower()]

    def cb_ok(self):
        self.check = False
        navigate_step("stop", 0.0)

    def cb_continue(self):
        navigate_step(
            self.dir,
            0.0,
            speed_lin=self.speed_lin*self.speed_corr,
            speed_rot=self.speed_rot*self.speed_corr,
        )

    def change_dir(self, new_dir=""):
        if new_dir == self.dir:
            return

        if new_dir:
            self.dir = new_dir
        else:
            self.dir = self.opposite_dir(self.dir)

        print(f"Laser: Changing direction to {self.dir}")

        self.speed_corr = 0.5
        self.cb_continue()
        self.speed_corr = 1.0
        self.reaction_lock = Laser.LOCK_COUNT

    def set_check(self, angle, diff, dist=0.0, block=True, angle_spread=2):
        # if dist is specified, the distance to the obstacle is checked and must be withing diff
        # if dist is not specified, the turtlebot is turned til the threshold is met.
        self.angle = int(angle)
        self.diff  = diff
        self.dist  = dist
        self.last  = -1
        self.last_avg = -1
        self.angle_spread = angle_spread
        self.reaction_lock = Laser.LOCK_COUNT
        if dist:
            self.dir = "f"
        else:
            self.dir = "l"

        if 90 <= angle < 270:
            self.dir_increase = "f"
        else:
            self.dir_increase = "b"

        self.check = True

        if block:
            while self.check:
                rospy.sleep(0.01)

    def cb_scan(self, scan: LaserScan):
        current_dist = scan.ranges[self.angle]
        current_dist_avg = (scan.ranges[self.angle - 1] + scan.ranges[self.angle] + scan.ranges[self.angle + 1]) / 3
        change = current_dist - self.last
        error = current_dist_avg - self.dist

        if not current_dist:
            print(f"Laser: Invalid data! (all zero)")
            return

        if not self.check:
            return
        if self.last < 0:
            self.last = current_dist
            self.last_avg = current_dist_avg
            return

        #angle_diff = scan.ranges[self.angle + self.angle_spread] - scan.ranges[self.angle - self.angle_spread] / current_dist

        relative_change = change / current_dist

        print(f"Laser: dist={current_dist}, dist_avg={current_dist_avg} error={error}, change={change}, relative_change% = {relative_change}")

        if self.reaction_lock:
            print(f"Laser: reaction lock {self.reaction_lock}")
            self.reaction_lock -= 1
        elif self.dist:
            if abs(error) < self.diff:
                print(f"Laser: Within lin tolerance ({self.diff})")
                self.cb_ok()
            else:
                if current_dist_avg < self.dist:
                    self.change_dir(self.dir_increase)
                else:
                    self.change_dir(self.opposite_dir(self.dir_increase))
                self.cb_continue()
        else:
            if abs(relative_change) < self.diff:
                print(f"Laser: Within rot tolerance ({self.diff})")
                self.cb_ok()
            else:
                print("")
                if relative_change > 0:
                    # Turning away from the wall. change direction
                    self.change_dir()
                else:
                    self.cb_continue()

        self.last = current_dist
        self.last_avg = current_dist_avg


laser = Laser()


def navigate_step(dir: str, delta_t: float, speed_lin=0.1, speed_rot=math.radians(30), laser_angle=0, laser_dist=0):
    global velocity_publisher

    print(dir)

    dir = dir.lower()

    twist = Twist()

    #[F]orward
    if dir.startswith("f"):
        twist.linear.x = 1
    #[B]ackward
    elif dir.startswith("b"):
        twist.linear.x = -1
    #[L]eft
    elif dir.startswith("l"):
        twist.angular.z = 1
    #[R]ight
    elif dir.startswith("r"):
        twist.angular.z = -1

    twist.linear.x  *= speed_lin
    twist.angular.z *= speed_rot

    velocity_publisher.publish(twist)
    rospy.sleep(delta_t)


def navigate():
    global velocity_publisher, laser
    rospy.init_node('blind_navigation', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtlebot1/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/turtlebot1/scan', LaserScan, laser.cb_scan)

    # Set linear velocity in the x-axis
    navigate_step("stop    ", 2.00)
    navigate_step("forward ", 1.30)
    laser.set_check(angle=0, diff=0.02, dist=0.24, block=True)
    navigate_step("left    ", 1.00)
    laser.set_check(angle=180, diff=0.002, block=True)
    navigate_step("forward ", 3.50)
    laser.set_check(angle=0, diff=0.04, dist=0.45, block=True)
    navigate_step("left    ", 1.6)
    laser.set_check(angle=90, diff=0.002, block=True)
    navigate_step("backward", 4.00)
    laser.set_check(angle=0, diff=0.05, dist=1.40, block=True)
    navigate_step("stop    ", 5.00)

    navigate_step("forward ", 4.50)
    laser.set_check(angle=0, diff=0.03, dist=0.95, block=True)
    navigate_step("left    ", 1.50)
    laser.set_check(angle=180, diff=0.002, block=True)
    navigate_step("forward ", 3.00)
    laser.set_check(angle=0, diff=0.01, dist=0.28, block=True)
    navigate_step("left    ", 1.00)
    laser.set_check(angle=270, diff=0.002, block=True)
    navigate_step("stop    ", 2.00)


if __name__ == '__main__':
    try:
        navigate()
    except rospy.ROSInterruptException:
        pass

