#!/usr/bin/env python3
import rospy
from robis_messages.msg import MoveAction, MoveGoal, GraspAction, GraspGoal
from group_messages.srv import store_cube, move_int, grip_uarm_test
import actionlib
from std_msgs.msg import Float64
import time


def handle_grip_uarm(order):
    # create ROS action client:
    grip_client2 = actionlib.SimpleActionClient('uarm2/grip', GraspAction)
    grip_client3 = actionlib.SimpleActionClient('uarm3/grip', GraspAction)
    uarm = int(order.uarm)
    print("connecting")
    # Waits until the action server has started up and started
    # listening for goals.
    if uarm == 2:
        grip_client2.wait_for_server()
        print("connected")
        # Creates a goal to send to the action server.
        goal = GraspGoal(grab=order.grip)
        print("goal defined")
        # Sends the goal to the action server.
        grip_client2.send_goal(goal)
        print("goal sent")
        # Waits for the server to finish performing the action.
        grip_client2.wait_for_result()
        # or shorter alternative to three commands above:
        # move_client.send_goal_and_wait(MoveGoal(target=[x, y, z, w]))
        # gets the result of the task
        return grip_client2.get_result()
    elif uarm == 3:
        grip_client3.wait_for_server()
        print("connected")
        # Creates a goal to send to the action server.
        goal = GraspGoal(grab=order.pos)
        print("goal defined")
        # Sends the goal to the action server.
        grip_client3.send_goal(goal)
        print("goal sent")
        # Waits for the server to finish performing the action.
        grip_client3.wait_for_result()
        # or shorter alternative to three commands above:
        # move_client.send_goal_and_wait(MoveGoal(target=[x, y, z, w]))
        # gets the result of the task
        return grip_client3.get_result()

    return True







if __name__ == '__main__':
    # Starts a new node
    rospy.init_node("handle_uarm_grip")
    rospy.loginfo("uarm gripping handler started")
    # create new Service server
    s = rospy.Service('uarm_testgrip/move', grip_uarm_test, handle_grip_uarm)
    rospy.spin()