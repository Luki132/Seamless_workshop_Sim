#! /usr/bin/env python3

import rospy

import actionlib
from group6_pkg.msg import cargo_actionAction
from group6_pkg.msg import cargo_actionGoal
from group6_pkg.msg import cargo_actionResult
from group6_pkg.msg import cargo_actionFeedback
from std_msgs.msg import Int64MultiArray
import actionlib_tutorials.msg

class CargoAction(object):
    # create messages that are used to publish feedback/result
    _feedback = cargo_actionFeedback()
    _result = cargo_actionResult()
    _goal = cargo_actionGoal()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, cargo_actionAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self._feedback.percent_complete = 0.0

      
    def execute_cb(self, goal):
        # helper variables
        print("BINGO")
        max_order = 0
        stow_box = goal.priority
        order_num = goal.num_objects
        order_num = [elem for elem in order_num]
        for i in range(6):
            max_order = max_order + order_num[i]
        r = rospy.Rate(1)
        success = True

        system_order = Int64MultiArray()
        system_order.data = [0,1,0,1,0,1]
        # print(system_order.data)
        pub1.publish(system_order)
        # append the seeds for the fibonacci sequence

        
        # publish info to the console for the user
        # rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (self._action_name, goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))
        
        # start executing the action
        for i in range(10):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            self._feedback.percent_complete = self._feedback.percent_complete + 1.0
            # publish the feedback
            self._as.publish_feedback(self._feedback)
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()
          
        if success:
            self._result.finish_time = 100
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

        
        
if __name__ == '__main__':
    rospy.init_node('cargo_server')
    pub1 = rospy.Publisher("/cargo_order",Int64MultiArray, queue_size=10)
    server = CargoAction(rospy.get_name())
    rospy.spin()