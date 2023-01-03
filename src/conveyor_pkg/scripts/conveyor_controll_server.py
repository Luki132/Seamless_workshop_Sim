#!/usr/bin/env python
import rospy
from robis_messages.msg import MoveAction, MoveGoal
from group_messages.srv import move_int
import actionlib
import time

def handle_move_conveyor(data):
    # create ROS action client:
    # CLIENT-NAME = actionlib.SimpleActionClient(TOPIC NAME, TYPE OF MESSAGE)
    conveyor_client = actionlib.SimpleActionClient('conveyor1/move', MoveAction)
    # gets dataand calls actionserver to move the conveyor
    conveyor_client.wait_for_server()
    print("connected")
    rospy.loginfo("I heard %s", data.length)

    # Creates a goal to send to the action server.
    goal = MoveGoal(target=[data.length, 0, 0, 0])
    print("goal conveyor defined")

    # Sends the goal to the action server.
    conveyor_client.send_goal(goal)
    print("goal sent")

    # Waits for the server to finish performing the action.
    conveyor_client.wait_for_result()

    # or shorter alternative to three commands above:
    # move_client.send_goal_and_wait(MoveGoal(target=[x, y, z, w]))

    # gets the result of the task
    # return conveyor_client.get_result()
    return True


if __name__ == '__main__':
    # Starts a new node
    rospy.init_node('conveyor_server')
    rospy.loginfo("I heard TEst")
    # create new Service server
    s = rospy.Service('conveyor_controll/move', move_int, handle_move_conveyor)
    rospy.spin()