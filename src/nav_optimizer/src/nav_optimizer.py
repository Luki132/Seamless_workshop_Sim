#!/usr/bin/env python
import time
from dataclasses import dataclass
from pathlib import Path
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from std_srvs.srv import Empty


@dataclass
class NAV_STATES:
    GOAL_REACHED = 3
    DRIVING = 1
    IDLE = 0


robot_pos = None
goal_pos = None
state = NAV_STATES.IDLE
timeout = 20
t0 = 0


def reset():
    #reset_simulation = rospy.ServiceProxy("/gazebo/reset_simulation", Empty)
    reset_world = rospy.ServiceProxy("/gazebo/reset_world", Empty)
    #reset_simulation()
    reset_world()


def update_robot_position(data:Odometry):
    global robot_pos
    robot_pos = data.pose.pose.position


def update_goal_position(data:PoseStamped):
    global goal_pos
    goal_pos = data.pose.position


def update_state(data:GoalStatusArray):
    global state
    state = data.status_list[0].status


def main():
    global timeout, t0

    rospy.init_node(Path(__file__).stem, anonymous=True)
    rospy.Subscriber("/turtlebot1/odom", data_class=Odometry, callback=update_robot_position)
    rospy.Subscriber("/turtlebot1/move_base_simple/goal", data_class=PoseStamped, callback=update_goal_position)
    rospy.Subscriber("/turtlebot1/move_base/status", data_class=GoalStatusArray, callback=update_state)

    reset()

    while 42:
        if state == NAV_STATES.DRIVING:
            if t0:
                drive_time = time.time() - t0
                print(f"Driving since {drive_time}s.")
                if drive_time > timeout:
                    t0 = 0
                    reset()
            else:
                t0 = time.time()
        elif state == NAV_STATES.GOAL_REACHED:
            t0 = 0

        time.sleep(0.1)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
