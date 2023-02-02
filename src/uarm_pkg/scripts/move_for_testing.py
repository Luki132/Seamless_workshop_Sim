#!/usr/bin/env python3
import rospy
from robis_messages.msg import MoveAction, MoveGoal, GraspAction, GraspGoal
from group_messages.srv import store_cube, move_int, move_uarm_test
import actionlib
from std_msgs.msg import Float64
import time


def handle_move_uarm(order):
    # create ROS action client:
    move_client2 = actionlib.SimpleActionClient('uarm2/move', MoveAction)
    grip_client2 = actionlib.SimpleActionClient('uarm2/grip', GraspAction)
    move_client3 = actionlib.SimpleActionClient('uarm3/move', MoveAction)
    grip_client3 = actionlib.SimpleActionClient('uarm3/grip', GraspAction)

    move_client1 = actionlib.SimpleActionClient('uarm1/move', MoveAction)
    grip_client1 = actionlib.SimpleActionClient('uarm1/grip', GraspAction)
    uarm = int(order.uarm)
    print("connecting")
    # Waits until the action server has started up and started
    # listening for goals.
    if uarm == 1:
        move_client1.wait_for_server()
        print("connected")
        # Creates a goal to send to the action server.
        goal = MoveGoal(target=order.pos)
        print("goal defined")
        # Sends the goal to the action server.
        move_client1.send_goal(goal)
        print("goal sent")
        # Waits for the server to finish performing the action.
        move_client1.wait_for_result()
        # or shorter alternative to three commands above:
        # move_client.send_goal_and_wait(MoveGoal(target=[x, y, z, w]))
        # gets the result of the task
        return move_client1.get_result()
    elif uarm == 2:
        move_client2.wait_for_server()
        print("connected")
        # Creates a goal to send to the action server.
        goal = MoveGoal(target=order.pos)
        print("goal defined")
        # Sends the goal to the action server.
        move_client2.send_goal(goal)
        print("goal sent")
        # Waits for the server to finish performing the action.
        move_client2.wait_for_result()
        # or shorter alternative to three commands above:
        # move_client.send_goal_and_wait(MoveGoal(target=[x, y, z, w]))
        # gets the result of the task
        return move_client2.get_result()
    elif uarm == 3:
        move_client3.wait_for_server()
        print("connected")
        # Creates a goal to send to the action server.
        goal = MoveGoal(target=order.pos)
        print("goal defined")
        # Sends the goal to the action server.
        move_client3.send_goal(goal)
        print("goal sent")
        # Waits for the server to finish performing the action.
        move_client3.wait_for_result()
        # or shorter alternative to three commands above:
        # move_client.send_goal_and_wait(MoveGoal(target=[x, y, z, w]))
        # gets the result of the task
        return move_client3.get_result()

    return True







if __name__ == '__main__':
    # Starts a new node
    rospy.init_node('handle_uarm_movement')
    rospy.loginfo("uarm movement handler started")
    # create new Service server
    s = rospy.Service('uarm_testcontroll/move', move_uarm_test, handle_move_uarm)
    rospy.spin()