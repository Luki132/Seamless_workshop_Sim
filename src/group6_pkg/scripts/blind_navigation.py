#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from navigation_setup.Settings import settings

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
        if self.ref_diff:
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

    def set_check(
            self,
            angle,
            diff,
            dist=0.0,
            block=True,
            angle_spread=2,
            speed_lin=None,
            speed_rot=None,
            ref_diff_factor=-1.0,
            dir="",
    ):
        # if dist is specified, the distance to the obstacle is checked and must be withing diff
        # if dist is not specified, the turtlebot is turned til the threshold is met.
        self.diff  = diff
        self.dist  = dist
        self.last  = -1
        self.last_avg = -1
        self.angle_spread = angle_spread
        # self.reaction_lock = Laser.LOCK_COUNT
        self.last_good = False
        angle = int(angle)

        if self.angle != angle:
            self.eval.reset()
            self.eval_l.reset()
            self.eval_r.reset()
        elif self.angle_spread != angle_spread:
            self.eval_l.reset()
            self.eval_r.reset()
        self.angle = angle

        if speed_lin:
            self.speed_lin = speed_lin
        if speed_rot:
            self.speed_rot = speed_rot
        if dist:
            if dir not in "fb":
                dir = "f"
            min_diff = 0.05
            if ref_diff_factor < 0:
                ref_diff_factor = 4.0
            elif ref_diff_factor == 0.0:
                min_diff = 0.0
            self.ref_diff = max(min_diff, ref_diff_factor * diff)
            self.angle_spread = 1
        else:
            if dir not in "rl":
                self.dir = "l"
            if ref_diff_factor < 0:
                ref_diff_factor = 20.0
            self.ref_diff = ref_diff_factor * diff

        if 90 <= angle < 270:
            self.dir_increase = "f"
        else:
            self.dir_increase = "b"

        self.dir = dir

        self.check = True

        if block:
            while self.check:
                rospy.sleep(0.01)

    def cb_scan(self, scan: LaserScan):
        current_dist = scan.ranges[self.angle]
        change = current_dist - self.last

        self.eval.set(current_dist)
        self.eval_l.set(scan.ranges[self.angle - self.angle_spread])
        self.eval_r.set(scan.ranges[(self.angle + self.angle_spread) % 360])

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

        print(f"Laser: dist={current_dist}, dist_avg={current_dist_avg} error={error}, change={change}, angle_diff={angle_diff_predicted}")

        if self.reaction_lock:
            print(f"Laser: reaction lock {self.reaction_lock}")
            self.reaction_lock -= 1
        elif self.dist:
            error = abs(error)
            if self.ref_diff:
                self.speed_corr = min(1.0, max(0.1, error / self.ref_diff))
            else:
                self.speed_corr = 1.0
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
            if self.ref_diff:
                self.speed_corr = min(1.0, max(0.05, min_diff / self.ref_diff))
            else:
                self.speed_corr = 1.0
            if min_diff < self.diff:
                if self.last_good:
                    print(f"Laser: Within rot tolerance ({min_diff})")
                    self.cb_ok()
                else:
                    self.last_good = True
                    if self.ref_diff:
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


speed_factor = 2.0


def normalize_dir(d: str):
    d = d[0].lower()
    return d if d in "fblr" else "s"


def navigate_step(dir: str, blind_time: float, speed_lin=0.1, speed_rot=math.radians(30), laser_angle=0, laser_dist=0):
    global velocity_publisher, speed_factor

    print(dir)

    dir = normalize_dir(dir)

    twist = Twist()

    lin = {"f": 1, "b": -1}
    rot = {"l": 1, "r": -1}

    if dir in lin:
        twist.linear.x = lin[dir] * speed_lin * speed_factor
    elif dir in rot:
        twist.angular.z = rot[dir] * speed_rot * speed_factor

    velocity_publisher.publish(twist)
    rospy.sleep(blind_time / (speed_factor * 1.1))


def drive_along_path(p):
    global laser
    path = list(settings.navigation[p].values())

    for step in path:
        step: dict
        blind_step: dict = step.get("blind", None)
        laser_data: dict = step.get("laser", None)
        dir = ""
        if blind_step:
            navigate_step(**blind_step)
            dir = normalize_dir(blind_step["dir"])
        if laser_data:
            if dir:
                laser_data["dir"] = dir
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

