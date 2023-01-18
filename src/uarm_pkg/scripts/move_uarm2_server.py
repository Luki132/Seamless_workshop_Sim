#!/usr/bin/env python3
import rospy
from robis_messages.msg import MoveAction, MoveGoal, GraspAction, GraspGoal
from group_messages.srv import store_cube, move_int
import actionlib
# from std_msgs.msg import Float64, bool
import time

pos_cube1 = []
storage2 = [250, -40, 120, 90]

def handle_move_uarm2(order):
    rospy.set_param('/Uarm2_took_cube_from_conv', False)
    pos = order.cube_pos
    move_uarm2([pos[0], pos[1], pos[2]+20, pos[3]])
    move_uarm2([pos[0], pos[1], pos[2], pos[3]])
    grip_uarm2(True)
    #time.sleep(1)
    move_uarm2([pos[0], pos[1], pos[2]+50, pos[3]])
    rospy.set_param('/Uarm2_took_cube_from_conv', True)
    move_uarm2([pos[0] + 80, pos[1]-50, pos[2]+80, pos[3]])
    move_uarm2(storage2)
    grip_uarm2(False)
    time.sleep(1)
    move_uarm2([pos[0], pos[1], pos[2]+50, pos[3]])

    return True


    """
    # create ROS action client:
    move_client = actionlib.SimpleActionClient('uarm2/move', MoveAction)
    grip_client = actionlib.SimpleActionClient('uarm2/grip', GraspAction)
    print("connecting")
    # Waits until the action server has started up and started
    # listening for goals.        
    move_client.wait_for_server()
    print("connected")

    # Creates a goal to send to the action server.
    goal = MoveGoal(target="")
    print("goal defined")

    # Sends the goal to the action server.
    move_client.send_goal(goal)
    print("goal sent")

    # Waits for the server to finish performing the action.
    move_client.wait_for_result()

    # or shorter alternative to three commands above:
    # move_client.send_goal_and_wait(MoveGoal(target=[x, y, z, w]))

    # gets the result of the task
    return move_client.get_result() """

def move_uarm2(pos):
    """
    publishes goal for uarm and waits until it is reached
    @param x:
    @param y:
    @param z:
    @param w:
    @return: MoveAction.action_result
    """
    # create ROS action client:
    # CLIENT-NAME = actionlib.SimpleActionClient(TOPIC NAME, TYPE OF MESSAGE)
    move_client = actionlib.SimpleActionClient('uarm2/move', MoveAction)

    print("connecting")
    # Waits until the action server has started up and started
    # listening for goals.        
    move_client.wait_for_server()
    print("connected")

    # Creates a goal to send to the action server.
    goal = MoveGoal(target=pos)
    print("goal defined")

    # Sends the goal to the action server.
    move_client.send_goal(goal)
    print("goal sent")

    # Waits for the server to finish performing the action.
    move_client.wait_for_result()

    # or shorter alternative to three commands above:
    # move_client.send_goal_and_wait(MoveGoal(target=[x, y, z, w]))

    # gets the result of the task
    return move_client.get_result()

def grip_uarm2(grab):
    """
    publishes goal for uarm gripper and waits until it is executed
    @param grab:
    @return: GraspAction.action_result
    """
    # create ROS action client:
    # CLIENT-NAME = actionlib.SimpleActionClient(TOPIC NAME, TYPE OF MESSAGE)
    grip_client = actionlib.SimpleActionClient('uarm2/grip', GraspAction)

    print("connecting")
    # Waits until the action server has started up and started
    # listening for goals.
    grip_client.wait_for_server()
    print("connected")

    # Creates a goal to send to the action server.
    goal = GraspGoal(grab=grab)
    print("goal defined")

    # Sends the goal to the action server.
    grip_client.send_goal(goal)
    print("goal sent")

    # Waits for the server to finish performing the action.
    grip_client.wait_for_result()

    # gets the result of the task
    return grip_client.get_result()



if __name__ == '__main__':
    # Starts a new node
    rospy.init_node('handle_uarm2_server')
    rospy.loginfo("starting handle_uarm2_server")
    # create new Service server
    # s = rospy.Subscriber('uarm2_controll/move', store_cube, handle_move_uarm2)
    s = rospy.Service('uarm2_controll/move', store_cube, handle_move_uarm2)
    rospy.set_param('/Uarm2_took_cube_from_conv', True)
    rospy.spin()