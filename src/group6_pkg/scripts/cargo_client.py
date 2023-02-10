#! /usr/bin/env python3

import rospy

import actionlib
from group6_pkg.msg import cargo_actionAction
from group6_pkg.msg import cargo_actionGoal
from group6_pkg.msg import cargo_actionResult
from group6_pkg.msg import cargo_actionFeedback
from std_msgs.msg import Int64MultiArray, Bool, Int64
import actionlib_tutorials.msg
from actionlib_msgs.msg import GoalStatus
import math
import numpy as np
import time 

class SomeActionClient:
    def __init__(self):
        self._client = actionlib.SimpleActionClient("cargo_server", cargo_actionAction)
        self._client.wait_for_server()
    
    def send_goal(self):
        goal = cargo_actionGoal()
        goal.num_objects = [0,0,0,0,1,1]
        goal.stow_box = 3
        self._client.send_goal(goal, self._done_cb, self._active_cb, self._feedback_cb)
    
    def _done_cb(self, status, result):
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Action succeeded")
        else:
            rospy.logerr("Action failed with status: %d" % status)
    
    def _active_cb(self):
        rospy.loginfo("Action started")
    
    def _feedback_cb(self, feedback):
        rospy.loginfo("Received feedback: %s" % feedback.percent_complete)
        rospy.loginfo("Received feedback: %s" % feedback.estimated_finish_time)

if __name__ == "__main__":
    rospy.init_node("some_action_client")
    client = SomeActionClient()
    client.send_goal()
    rospy.spin()