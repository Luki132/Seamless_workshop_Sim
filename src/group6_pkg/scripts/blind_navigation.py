#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import Bool, String
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


class Extrapolated:
    def __init__(self, depth=3):
        self.actual_depth = 0
        self.index = 0
        self.depth = depth
        self.reset()

    def set(self, val, ignore_zero=True):
        if ignore_zero and not val:
            self.vals[self.index] = self.get()
        self.vals[self.index] = val
        self.index = (self.index + 1) % self.depth
        if self.actual_depth < self.depth:
            self.actual_depth += 1

    def get(self, future_steps=0):
        val = 0

        val = self.vals[self.index - 1]

        if future_steps <= 0:
            return val

        diff0 = [0] * (self.actual_depth - 1)
        diff1 = [0] * (self.actual_depth - 2)
        diff2 = [0] * (self.actual_depth - 3)

        diff0 = [self.vals[self.index - i] - self.vals[self.index - i - 1] for i in range(1, self.actual_depth)]
        diff1 = [diff0[-i] - diff0[-i - 1] for i in range(0, len(diff0) - 1)]
        diff2 = [diff1[-i] - diff1[-i - 1] for i in range(0, len(diff1) - 1)]

        # future_steps += 1
        # future = [val] * future_steps
        #
        # diff0 = diff0[-1] if diff0 else 0
        # diff1 = diff1[-1] if diff1 else 0
        # diff2 = diff2[-1] if diff2 else 0
        # for i in range(1, future_steps):
        #     diff1 += diff2
        #     diff0 += diff1
        #     future[i] = future[i-1] + diff0
        # return future[-1]

        avg_diff = sum(diff0) / len(diff0)

        val += avg_diff * future_steps
        return val


    def reset(self, val = 0.0):
        self.vals = [val] * self.depth


class Laser:
    LOCK_COUNT = 1

    def __init__(self):
        self.angle = 0
        self.last  = 0.0
        self.dist  = 0.0
        self.diff  = 0.0
        self.dir = "f"
        self.speed_lin = 0.05
        self.speed_rot = math.radians(20)
        self.check = False
        self.reaction_lock = Laser.LOCK_COUNT
        self.speed_corr = 1.0
        self.dir_increase = "f"
        self.angle_spread = 2
        self.last_avg = -1
        self.eval = Extrapolated()
        self.eval_l = Extrapolated()
        self.eval_r = Extrapolated()
        self.ref_diff = 0.0
        self.last_good = False

    def opposite_dir(self, dir: str):
        return {"s": "s", "f": "b", "b": "f", "l": "r", "r": "l"}[dir[0].lower()]

    def cb_ok(self):
        self.check = False
        navigate_step("stop", 0.0)

    def cb_continue(self):
        print(f"speed_corr: {self.speed_corr}")
        navigate_step(
            self.dir,
            0.0,
            speed_lin=self.speed_lin*self.speed_corr,
            speed_rot=self.speed_rot*self.speed_corr,
        )

    def change_dir(self, new_dir=""):
        if new_dir == self.dir:
            self.cb_continue()
            return

        if self.dir != "s":
            self.reaction_lock = Laser.LOCK_COUNT

        if new_dir:
            self.dir = new_dir
        else:
            self.dir = self.opposite_dir(self.dir)

        print(f"Laser: Changing direction to {self.dir}")

        self.eval.reset()
        self.eval_l.reset()
        self.eval_r.reset()

        self.cb_continue()

    def set_check(self, angle, diff, dist=0.0, block=True, angle_spread=2):
        # if dist is specified, the distance to the obstacle is checked and must be withing diff
        # if dist is not specified, the turtlebot is turned til the threshold is met.
        self.angle = int(angle)
        self.diff  = diff
        self.dist  = dist
        self.last  = -1
        self.last_avg = -1
        self.angle_spread = angle_spread
        # self.reaction_lock = Laser.LOCK_COUNT
        self.last_good = False
        self.eval.reset()
        self.eval_l.reset()
        self.eval_r.reset()
        if dist:
            self.dir = "f"
            self.ref_diff = 4 * diff
        else:
            self.dir = "l"
            self.ref_diff = 20 * diff

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
            print(f"Laser: Invalid data for ray {self.angle}! (all zero)")
            return

        self.eval.set(current_dist)
        self.eval_l.set(scan.ranges[self.angle - self.angle_spread])
        self.eval_r.set(scan.ranges[self.angle + self.angle_spread])

        if not self.check:
            return
        if self.last < 0:
            self.last = current_dist
            self.last_avg = current_dist_avg
            return

        angle_diff = (scan.ranges[self.angle + self.angle_spread] - scan.ranges[self.angle - self.angle_spread]) / current_dist
        angle_diff_predicted = (self.eval_r.get(0) - self.eval_l.get(0)) / self.eval.get(0)

        print(f"Laser: dist={current_dist}, dist_avg={current_dist_avg} error={error}, change={change}, angle_diff={angle_diff}, predicted={angle_diff_predicted}")

        if self.reaction_lock:
            print(f"Laser: reaction lock {self.reaction_lock}")
            self.reaction_lock -= 1
        elif self.dist:
            error = abs(error)
            self.speed_corr = min(1.0, max(0.3, error / self.ref_diff))
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
            min_diff = abs(angle_diff_predicted)
            self.speed_corr = min(1.0, max(0.05, min_diff / self.ref_diff))
            if min_diff < self.diff:
                if self.last_good:
                    print(f"Laser: Within rot tolerance ({min_diff})")
                    self.cb_ok()
                else:
                    self.last_good = True
                    self.change_dir("s")
            else:
                self.last_good = False
                if angle_diff > 0:
                    # Turning away from the wall. change direction
                    self.change_dir("r")
                else:
                    self.change_dir("l")

        self.last = current_dist
        self.last_avg = current_dist_avg


laser = Laser()


speed_factor = 1.5


def navigate_step(dir: str, delta_t: float, speed_lin=0.1, speed_rot=math.radians(30), laser_angle=0, laser_dist=0):
    global velocity_publisher, speed_factor

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

    twist.linear.x  *= speed_lin * speed_factor
    twist.angular.z *= speed_rot * speed_factor

    velocity_publisher.publish(twist)
    rospy.sleep(delta_t / speed_factor)


def path_docking_to_conveyor():
    navigate_step("stop    ", 2.00)
    navigate_step("forward ", 1.30)
    laser.set_check(angle=0, diff=0.02, dist=0.24, block=True)
    navigate_step("left    ", 1.00)
    laser.set_check(angle=180, diff=0.0025, block=True)
    navigate_step("forward ", 3.50)
    laser.set_check(angle=0, diff=0.04, dist=0.45, block=True)
    navigate_step("left    ", 1.5)
    laser.set_check(angle=270, diff=0.0025, block=True)
    navigate_step("backward", 3.50)
    laser.set_check(angle=0, diff=0.05, dist=1.45, block=True)
    navigate_step("stop    ", 0.00)


def path_conveyor_to_parking():
    navigate_step("forward ", 3.50)
    laser.set_check(angle=0, diff=0.03, dist=1.00, block=True)
    navigate_step("left    ", 0.50)
    laser.set_check(angle=180, diff=0.0025, block=True)
    navigate_step("forward ", 3.00)
    laser.set_check(angle=0, diff=0.005, dist=0.29, block=True)
    navigate_step("left    ", 1.10)
    laser.set_check(angle=267, diff=0.0025, block=True)
    navigate_step("stop    ", 0.00)


paths = {
    "conveyor": path_docking_to_conveyor,
    "parking":  path_conveyor_to_parking,
}

goal_list = []
pub_state: rospy.Publisher

def cb_goal(msg: String):
    global  goal_list, paths, pub_state
    if msg.data in paths:
        pub_state.publish(f"accepted:{msg.data}")
        goal_list.append(msg.data)
    else:
        pub_state.publish(f"refused:{msg.data}")


def navigate():
    global velocity_publisher, laser, paths, goal_list, pub_state
    rospy.init_node('navigation', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtlebot1/cmd_vel', data_class=Twist, queue_size=10)
    sub_laser_scan = rospy.Subscriber('/turtlebot1/scan', data_class=LaserScan, callback=laser.cb_scan)

    sub_goal = rospy.Subscriber('/group6/nav_goal', data_class=String, callback=cb_goal)
    pub_state = rospy.Publisher('/group6/nav_state', data_class=String, queue_size=10)

    while not rospy.is_shutdown():
        if goal_list:
            p = goal_list.pop(0)
            # Drive to the desired positions.
            pub_state.publish(f"driving:{p}")
            paths[p]()
            pub_state.publish(f"reached:{p}")


if __name__ == '__main__':
    try:
        navigate()
    except rospy.ROSInterruptException:
        pass

