#!/usr/bin/env python3
import rospy
from robis_messages.msg import MoveAction, MoveGoal
from group_messages.srv import store_cube
from group_messages.srv import move_int
import actionlib
import time


def move_conveyor_callback(data):
  while True:
    if rospy.has_param('/Uarm2_took_cube_from_conv') and rospy.get_param('/Uarm2_took_cube_from_conv'):
      handle_move_conveyor(data)
      rospy.set_param('/Uarm2_took_cube_from_conv', False)
      uarm2_client = rospy.ServiceProxy('uarm2_controll/move', store_cube)
      uarm2_client.wait_for_service()
      goal = store_cube._request_class(cube_pos=[96, 159, 60, 90])
      result = uarm2_client(goal)
      # uarm2_publisher = rospy.Publisher('uarm2_controll/move', store_cube
      # goal = store_cube._request_class(cube_pos=[108, 167, 46, 90])
      # uarm2_publisher.publish(goal)
      return True
    else:
      time.sleep(2)





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
    s = rospy.Service('conveyor_controll/move', move_int, move_conveyor_callback)
    rospy.spin()


    """rostopic pub -1 /turtlebot1/initialpose geometry_msgs/PoseWithCovarianceStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose:
  pose:
    position: {x: 0.5, y: 0.44, z: 0.0}
    orientation: {x: 3.141593, y: 0.0, z: 1.0, w: 0.0}
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"""