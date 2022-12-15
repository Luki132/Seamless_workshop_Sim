#!/usr/bin/env python3


import rospy
import actionlib
import time
from std_msgs.msg import Float64

from robis_messages.msg import MoveAction, GraspAction, MoveGoal, GraspGoal

# create ROS action client:
# CLIENT-NAME = actionlib.SimpleActionClient(TOPIC NAME, TYPE OF MESSAGE)
conveyor_client = actionlib.SimpleActionClient('conveyor1/move', MoveAction)

# start node
rospy.init_node('conveyor_test_client', anonymous=True)

def move_conveyor_callback(data):

    # gets dataand calls actionserver to move the conveyor
    conveyor_client.wait_for_server()
    print("connected")
    rospy.loginfo("I heard %s", data.data)

    # Creates a goal to send to the action server.
    goal = MoveGoal(target=[data.data, 0, 0, 0])
    print("goal conveyor defined")

    # Sends the goal to the action server.
    conveyor_client.send_goal(goal)
    print("goal sent")

    # Waits for the server to finish performing the action.
    conveyor_client.wait_for_result()

    # or shorter alternative to three commands above:
    # move_client.send_goal_and_wait(MoveGoal(target=[x, y, z, w]))

    return conveyor_client.get_result()

# move_conveyor_callback(200)

def Conveyor1CommandMovement():
    print("listening to move/command")
    rospy.Subscriber("/conveyor_controll1/move/command", Float64, callback=move_conveyor_callback)
    rospy.spin()



if __name__ == '__main__':
    # start node
    Conveyor1CommandMovement()