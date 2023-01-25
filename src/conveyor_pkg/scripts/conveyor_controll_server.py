#!/usr/bin/env python3
import rospy
from robis_messages.msg import MoveAction, MoveGoal
from group_messages.srv import store_cube
from group_messages.msg import conv_cube
from std_msgs.msg import Int64
import actionlib
import time


def move_conveyor_callback(data):
  rospy.loginfo("I test heard %s", data.length)
  while True:
    if rospy.has_param('/Uarm2_took_cube_from_conv') and rospy.get_param('/Uarm2_took_cube_from_conv'):
      handle_move_conveyor(int(data.length))
      rospy.set_param('/conveyor_moved', True)
      rospy.set_param('/Uarm2_took_cube_from_conv', False)
      uarm2_client = rospy.ServiceProxy('uarm2_controll/move', store_cube)
      uarm2_client.wait_for_service()
      if int(data.size) == 1:
        goal = store_cube._request_class(cube_pos=[87, 159, 36, 90], storagebox = 2)
      else:
        goal = store_cube._request_class(cube_pos=[87, 159, 50, 90], storagebox = 2)
      result = uarm2_client(goal)
      # uarm2_publisher = rospy.Publisher('uarm2_controll/move', store_cube
      # goal = store_cube._request_class(cube_pos=[108, 167, 46, 90])
      # uarm2_publisher.publish(goal)
      return True
    else:
      time.sleep(1)





def handle_move_conveyor(data):
    # create ROS action client:
    # CLIENT-NAME = actionlib.SimpleActionClient(TOPIC NAME, TYPE OF MESSAGE)
    conveyor_client = actionlib.SimpleActionClient('conveyor1/move', MoveAction)
    # gets dataand calls actionserver to move the conveyor
    conveyor_client.wait_for_server()
    print("connected")
    # Creates a goal to send to the action server.
    goal = MoveGoal(target=[data, 0, 0, 0])
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
    rospy.set_param("/conveyor_moved", True)
    # create new Service server
    s = rospy.Subscriber('conveyor_controll/move', conv_cube, move_conveyor_callback)
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