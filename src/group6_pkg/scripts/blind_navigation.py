#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from Settings import settings


velocity_publisher: rospy.Publisher


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

    def set_check(self, angle, diff, dist=0.0, block=True, angle_spread=2, speed_lin=None, speed_rot=None):
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
        if speed_lin:
            self.speed_lin = speed_lin
        if speed_rot:
            self.speed_rot = speed_rot
        if dist:
            self.dir = "f"
            self.ref_diff = max(0.05, 4 * diff) # min 5cm
            self.angle_spread = 1
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
        change = current_dist - self.last

        self.eval.set(current_dist)
        self.eval_l.set(scan.ranges[self.angle - self.angle_spread])
        self.eval_r.set(scan.ranges[self.angle + self.angle_spread])

        if not self.eval.get() * self.eval_l.get() * self.eval_r.get():
            print(f"Some laser rays are zero (l, c, r): {self.eval_l.get():.3f}, {self.eval.get():.3f}, {self.eval_r.get():.3f}")
            return

        current_dist_avg = (self.eval_l.get() + self.eval.get() + self.eval_r.get()) / 3
        error = current_dist_avg - self.dist

        if not self.check:
            return
        if self.last < 0:
            self.last = current_dist
            self.last_avg = current_dist_avg
            return

        # angle_diff = (scan.ranges[self.angle + self.angle_spread] - scan.ranges[self.angle - self.angle_spread]) / self.eval.get()
        angle_diff_predicted = (self.eval_r.get() - self.eval_l.get()) / self.eval.get()

        print(f"Laser: dist={current_dist}, dist_avg={current_dist_avg} error={error}, change={change}, predicted={angle_diff_predicted}")

        if self.reaction_lock:
            print(f"Laser: reaction lock {self.reaction_lock}")
            self.reaction_lock -= 1
        elif self.dist:
            error = abs(error)
            self.speed_corr = min(1.0, max(0.1, error / self.ref_diff))
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
                if angle_diff_predicted > 0:
                    # Turning away from the wall. change direction
                    self.change_dir("r")
                else:
                    self.change_dir("l")

        self.last = current_dist
        self.last_avg = current_dist_avg


laser = Laser()


speed_factor = 1.5


def navigate_step(dir: str, blind_time: float, speed_lin=0.1, speed_rot=math.radians(30), laser_angle=0, laser_dist=0):
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
    rospy.sleep(blind_time / speed_factor)


def drive_along_path(p):
    global laser
    path = list(settings.navigation[p].values())

    for step in path:
        step: dict
        blind_step = step.get("blind", None)
        laser_data = step.get("laser", None)
        if blind_step:
            navigate_step(**blind_step)
        if laser_data:
            laser.set_check(**laser_data)


goal_list = []
pub_state: rospy.Publisher


def cb_goal(msg: String):
    global goal_list,  pub_state
    if msg.data in settings.navigation:
        pub_state.publish(f"accepted:{msg.data}")
        goal_list.append(msg.data)
    else:
        pub_state.publish(f"refused:{msg.data}")

#########################################
nav_state = ""
park_state = False


def cb_nav(msg: String):
    global nav_state
    print("cbn")
    nav_state = msg.data


def cb_park(msg: Bool):
    global park_state
    print("cbp")
    park_state = msg.data

def automated_ping_pong():
    #rospy.init_node('autonav', anonymous=True)
    sub_nav_state = rospy.Subscriber('/group6/nav_state', data_class=String, callback=cb_nav)
    sub_park_state = rospy.Subscriber('/group6/parking_state', data_class=Bool, callback=cb_park)
    pub_nav_goal = rospy.Publisher('/group6/nav_goal', data_class=String, queue_size=10)
    while not rospy.is_shutdown():
        goal = String()
        print(1)
        goal.data = "conveyor"
        pub_nav_goal.publish(goal)
        print(2)
        goal.data = "parking"
        pub_nav_goal.publish(goal)
        print(3)
        rospy.sleep(5)
        while nav_state != "reached:parking":
            pass
        print(4)
        while park_state:
            pass
        print(5)
        while not park_state:
            pass

def navigate():
    global velocity_publisher, laser, goal_list, pub_state
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
            drive_along_path(p)
            pub_state.publish(f"reached:{p}")


if __name__ == '__main__':
    try:
        navigate()
    except rospy.ROSInterruptException:
        pass

