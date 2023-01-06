#!/usr/bin/env python
import rospy
from robis_messages.msg import MoveAction, MoveGoal, GraspAction, GraspGoal
from group_messages.srv import order_cube1, move_int, store_cube
import actionlib
from std_msgs.msg import Float64
import time

pos_cube1 = [80, 148, 58, 90]
pos_cube2 = [80, 180, 150, 90]

po_onTur1 = [150, -120, 141, 90]


def handle_move_uarm3(order):
    rospy.loginfo("I heard %s", order.storage1)
    rospy.logdebug("I heard %s", order.storage2)
    print("data: " + str(order.storage1) + " " + str(order.storage2) + " " + str(order.storage3))

    storage = [order.storage1, order.storage2, order.storage3]
    positions = [order.pos1, order.pos2, order.pos3]
    for cube in range(3):
        if storage[cube]:
            rospy.loginfo("Getting Cube Nr. %s out of 3", cube)
            pos = positions[cube]
            move_uarm3([pos[0], pos[1]+80, pos[2]+20, pos[3]])
            move_uarm3([pos[0], pos[1], pos[2]+20, pos[3]])
            move_uarm3([pos[0], pos[1], pos[2], pos[3]])
            time.sleep(1)
            grip_uarm3(True)
            time.sleep(1)
            move_uarm3([pos[0], pos[1], pos[2]+30, pos[3]])
            move_uarm3([pos[0]-50, pos[1], pos[2]+30, pos[3]])
            move_uarm3([pos[0]-50, pos[1]+80, pos[2]+30, pos[3]])
            #time.sleep(1)
            move_uarm3([pos_cube1[0], pos_cube1[1], pos_cube1[2]+100, pos_cube1[3]])
            move_uarm3(pos_cube1)
            #time.sleep(1)
            grip_uarm3(False)
            #time.sleep(1)
            move_uarm3([pos_cube1[0], pos_cube1[1], pos_cube1[2]+50, pos_cube1[3]])
            time.sleep(1)
        else:
            # move uarm to turtelbot and take the stone and put it on the conveyor
            pass
    rospy.wait_for_service('conveyor_controll/move')
    conveyor_client = rospy.ServiceProxy('conveyor_controll/move', move_int)
    goal = move_int._request_class(length=-300)
    result = conveyor_client(goal)
    print("The result of the Service is: " + str(result))

    #uarm2_client = rospy.ServiceProxy('uarm2_controll/move', store_cube)
    #goal = store_cube._request_class(cube_pos=[75, 147, 58, 90])
    #result = uarm2_client(goal)
    
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

def move_uarm3(pos):
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
    move_client = actionlib.SimpleActionClient('uarm3/move', MoveAction)

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

def grip_uarm3(grab):
    """
    publishes goal for uarm gripper and waits until it is executed
    @param grab:
    @return: GraspAction.action_result
    """
    # create ROS action client:
    # CLIENT-NAME = actionlib.SimpleActionClient(TOPIC NAME, TYPE OF MESSAGE)
    grip_client = actionlib.SimpleActionClient('uarm3/grip', GraspAction)

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
    rospy.init_node('handle_uarm3_server')
    rospy.loginfo("I heard TEst")
    # create new Service server
    s = rospy.Service('uarm3_controll/move', order_cube1, handle_move_uarm3)
    rospy.spin()