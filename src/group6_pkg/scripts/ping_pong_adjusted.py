# !/usr/bin/env python


from dataclasses import dataclass
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Bool ## Caleb did this
import time
from math import dist, sqrt, sin, cos, pi



def fix_start_pose():
    start_pose_pub = rospy.Publisher('/turtlebot1/initialpose', data_class=PoseWithCovarianceStamped, queue_size=10)
    start_pose = PoseWithCovarianceStamped()
    start_pose.pose.pose.position.x = 1.268201
    start_pose.pose.pose.position.y = 0.783632
    start_pose.pose.pose.position.z = 0.000000
    start_pose.pose.pose.orientation.x = 0.0
    start_pose.pose.pose.orientation.y = 0.0
    start_pose.pose.pose.orientation.z = 1.0
    start_pose.pose.pose.orientation.w = 0.0
    # covariance can remain all 0 as it is by default.
    start_pose_pub.publish(start_pose)


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
    "intermediary-down": {
        "position": {
            "x":  0.88,
            "y":  0.45,
            "z":  0.00,
        },
        "orientation": orientation_from_deg(ORIENTATIONS_DEG.LEFT)
    },
    "intermediary-up": {
        "position": {
            "x": 1.15,
            "y": 0.48,
            "z": 0.00,
        },
        "orientation": orientation_from_deg(ORIENTATIONS_DEG.UP)
    },

    "bay-up": {
        "position": {
            "x":  1.1,
            "y":  0.80,
            "z":  0.00,
        },
        "orientation": orientation_from_deg(ORIENTATIONS_DEG.DOWN)
    },
    "bay-left": {
        "position": {
            "x":  1.1,
            "y":  0.80,
            "z":  0.00,
        },
        "orientation": orientation_from_deg(ORIENTATIONS_DEG.LEFT)
    },

    "conveyor-left": {
        "position": {
            "x": 0.43,
            "y": 0.32,
            "z": 0.00,
        },
        "orientation": orientation_from_deg(ORIENTATIONS_DEG.LEFT)
    },
    "conveyor-right": {
        "position": {
            "x":  0.60,
            "y":  0.32,
            "z":  0.00,
        },
        "orientation": orientation_from_deg(ORIENTATIONS_DEG.RIGHT)
    },
}


poses_chain = [
    "bay-left", "pause",
    "intermediary-down",
    "conveyor-right", "pause",
    "intermediary-up",
]

pose_index = 0
pose = poses_chain[pose_index]

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
    #print("State:", state, {0:"idle", 1:"driving", 2:"cancelled", 3:"goal", 4:"osc"}.get(state, "unknown"))
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


def startup_delay():
    print("Starting in 3", end=" ")
    rospy.sleep(1)
    print("2", end=" ")
    rospy.sleep(1)
    print("1", end=" ")

def move_callback(data):
    global state_changed, pose_index, poses_chain, pose
    print("BINGO")
    timeout = 20

    t0 = rospy.get_time()

    other_reason = True
    
    while not rospy.is_shutdown(): ## Not a good idea for now but I wanna test something
        t1 = rospy.get_time()
        if t1-t0 > timeout:
            other_reason = True
            print("Timeout")

        if state_changed:
            state_changed = False
            if state == NAV_STATES.GOAL_REACHED:
                other_reason = True

        if other_reason:
            other_reason = False
            t0 = t1

            # At the end of the chain, restart. Explicit pause markers
            pose_index = (pose_index + 1) % len(poses_chain)
            pose = poses_chain[pose_index]
            if pose == "pause":
                pause_delay = 0.5
                print(f"Pausing for {pause_delay:.1}s")
                rospy.sleep(pause_delay)
                other_reason = True
                continue

            print("Heading towards", pose)
            publish_goal_pose(poses[pose])

        elif pose.startswith("intermediary"):
            ax = position.pose.pose.position.x
            ay = position.pose.pose.position.y
            bx = poses[pose]["position"]["x"]
            by = poses[pose]["position"]["y"]
            if in_range((ax, ay), (bx, by), 0.12):
                print("Close to intermediary")
                other_reason = True

def ping_pong():
    global position, state, state_changed
    rospy.init_node('ping_pong')
    rospy.Subscriber("/turtlebot1/odom", data_class=Odometry, callback=pos_callback)
    rospy.Subscriber("/turtlebot1/move_base/status", data_class=GoalStatusArray, callback=update_state)
    rospy.Subscriber("/cargo_ready", Bool, callback=move_callback)

    print("Fixing turtlebot position... ", end="")
    fix_start_pose()
    rospy.sleep(1)  # because reasons
    fix_start_pose()
    print("Done.")

    # pose_index = 0
    # pose = poses_chain[pose_index]

    # Publish one dummy pose because reasons.
    publish_goal_pose(poses["bay-left"])

    rospy.spin() ### This was Caleb as well

    # timeout = 20

    # t0 = rospy.get_time()

    # other_reason = True

    ## Forgive me Max. I'm gonna mesh up your code to try something

    # while not rospy.is_shutdown():
    #     t1 = rospy.get_time()
    #     if t1-t0 > timeout:
    #         other_reason = True
    #         print("Timeout")

    #     if state_changed:
    #         state_changed = False
    #         if state == NAV_STATES.GOAL_REACHED:
    #             other_reason = True

    #     if other_reason:
    #         other_reason = False
    #         t0 = t1

    #         # At the end of the chain, restart. Explicit pause markers
    #         pose_index = (pose_index + 1) % len(poses_chain)
    #         pose = poses_chain[pose_index]
    #         if pose == "pause":
    #             pause_delay = 0.5
    #             print(f"Pausing for {pause_delay:.1}s")
    #             rospy.sleep(pause_delay)
    #             other_reason = True
    #             continue

    #         print("Heading towards", pose)
    #         publish_goal_pose(poses[pose])

    #     elif pose.startswith("intermediary"):
    #         ax = position.pose.pose.position.x
    #         ay = position.pose.pose.position.y
    #         bx = poses[pose]["position"]["x"]
    #         by = poses[pose]["position"]["y"]
    #         if in_range((ax, ay), (bx, by), 0.12):
    #             print("Close to intermediary")
    #             other_reason = True


if __name__ == '__main__':
    try:
        ping_pong()
    except rospy.ROSInterruptException:
        pass
