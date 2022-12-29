# !/usr/bin/env python


from dataclasses import dataclass
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray
import time
from math import dist, sqrt, sin, cos, pi


# right = 0
# up = 90

@dataclass
class ORIENTATIONS_DEG:
    RIGHT = 0
    UP = 90
    LEFT = 180
    DOWN = -90


def orientation_from_deg(deg:float):
    rad = pi * deg / 180
    return {
        "x": 0.00,
        "y": 0.00,
        "z": sin(rad / 2),
        "w": cos(rad / 2),
    }


poses = {
    "intermediary": {
        "position": {
            "x":  0.88,
            "y":  0.45,
            "z":  0.00,
        },
        "orientation": orientation_from_deg(ORIENTATIONS_DEG.LEFT)
    },

    "bay-up": {
        "position": {
            "x":  1.05,
            "y":  0.80,
            "z":  0.00,
        },
        "orientation": orientation_from_deg(ORIENTATIONS_DEG.DOWN)
    },
    "bay-left": {
        "position": {
            "x":  1.05,
            "y":  0.80,
            "z":  0.00,
        },
        "orientation": orientation_from_deg(ORIENTATIONS_DEG.LEFT)
    },

    "conveyor": {
        "position": {
            "x":  0.43,
            "y":  0.32,
            "z":  0.00,
        },
        "orientation": orientation_from_deg(ORIENTATIONS_DEG.LEFT)
    },
}


poses_chain = ["bay-left", "bay-up", "intermediary", "conveyor"]


def in_range(
        pose_a: tuple,
        pose_b: tuple,
        range: float
) -> bool:
    #x = dist(pose_a, pose_b)
    #print("dist", x)
    return dist(pose_a, pose_b) <= range


def publish_goal_pose(pose):
    # Create a publisher for the goal pose topic
    goal_pose_pub = rospy.Publisher('/turtlebot1/move_base_simple/goal', data_class=PoseStamped, queue_size=10)

    # Create a PoseStamped message
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.pose.position.x = pose["position"]["x"]
    goal_pose.pose.position.y = pose["position"]["y"]
    goal_pose.pose.position.z = pose["position"]["z"]
    goal_pose.pose.orientation.x = pose["orientation"]["x"]
    goal_pose.pose.orientation.y = pose["orientation"]["y"]
    goal_pose.pose.orientation.z = pose["orientation"]["z"]
    goal_pose.pose.orientation.w = pose["orientation"]["w"]

    # Publish the goal pose
    # For some ******** reason, after starting the python script,
    # the first message always gets lost. Thus play safe and
    # publish two messages.
    # ON TOP OF THAT... it seems that restarting this script while
    # keeping the rest running causes issues because the seq[uence]
    # number makes a jump back to 1 (first message for the script,
    # but not for the rest). So although one message actually gets
    # published, it's still ignored. Therefore: publish 3. One gets
    # lost, one gets ignored, and one finally works.
    rate = rospy.Rate(10)
    i = 0
    while not rospy.is_shutdown() and i < 1:
        i += 1
        goal_pose_pub.publish(goal_pose)
        rate.sleep()


position = None


def pos_callback(p):
    global position
    position = p


@dataclass
class NAV_STATES:
    OSCILLATING = 4
    GOAL_REACHED = 3
    CANCELLED_AND_NEW = 2
    DRIVING = 1
    IDLE = 0


state = NAV_STATES.IDLE
state_changed = True


def update_state(data:GoalStatusArray):
    global state, state_changed
    new_state = data.status_list[0].status
    if new_state != state:
        state = new_state
        state_changed = True
    else:
        state_changed = False


def user_controlled_goal_pose():
    # Initialize the ROS node
    rospy.init_node('goal_pose_publisher')

    while not rospy.is_shutdown():
        i = ""
        while not i:
            i = input("Desired position: ").lower()
        for k, v in poses.items():
            if k.startswith(i):
                i = "ok"
                print("Publishing", k)
                publish_goal_pose(v)
                publish_goal_pose(v)
                break
        if i != "ok":
            # unknown goal pos
            print(f"Unknown goal pose: {i}")
            print("Available poses:", [p for p in poses.keys()])


def ping_pong():
    global position, state, state_changed
    rospy.init_node('ping_pong')
    rospy.Subscriber("/turtlebot1/odom", data_class=Odometry, callback=pos_callback)
    rospy.Subscriber("/turtlebot1/move_base/status", data_class=GoalStatusArray, callback=update_state)
    pose_dst = "bay"
    pose = "bay"
    pose_index = 0
    pose_dir = -1
    pose = poses_chain[pose_index]

    # Publish one dummy pose because reasons.
    publish_goal_pose(poses["bay-left"])

    timeout = 20

    t0 = time.time()

    other_reason = True

    while not rospy.is_shutdown():
        t1 = time.time()
        if t1-t0 > timeout:
            other_reason = True
            print("Timeout")

        if state_changed and state == NAV_STATES.GOAL_REACHED:
            state_changed = False
            other_reason = True

        if other_reason:
            other_reason = False
            t0 = t1

            # At either end of the chain, switch direction - and wait a few secs
            if pose_index <= 0 or pose_index >= len(poses_chain) - 1:
                time.sleep(5)
                pose_dir *= -1
            pose_index += pose_dir
            print(pose_index, pose_dir)
            pose = poses_chain[pose_index]

            # if pose == "intermediary":
            #     pose = pose_dst
            # else:
            #     # Actual goal reached. Wait for a few seconds.
            #     time.sleep(5)
            #     pose = "intermediary"
            #     if pose_dst == "bay":
            #         pose_dst = "conveyor"
            #     else:
            #         pose_dst = "bay"
            # if pose == "bay":
            #     pose = "conveyor"
            # else:
            #     pose = "bay"
            # time.sleep(5)
            print("Heading towards", pose)
            publish_goal_pose(poses[pose])

        elif pose == "intermediary":
            ax = position.pose.pose.position.x
            ay = position.pose.pose.position.y
            bx = poses[pose]["position"]["x"]
            by = poses[pose]["position"]["y"]
            if in_range((ax, ay), (bx, by), 0.1):
                print("Close to intermediary")
                other_reason = True


if __name__ == '__main__':
    try:
        ping_pong()
    except rospy.ROSInterruptException:
        pass
