#! /usr/bin/env python3

import rospy

import actionlib
from group6_pkg.msg import cargo_actionAction
from group6_pkg.msg import cargo_actionGoal
from group6_pkg.msg import cargo_actionResult
from group6_pkg.msg import cargo_actionFeedback
from std_msgs.msg import Int64MultiArray, Bool, Int64
import actionlib_tutorials.msg

import math
import numpy as np
import time 

class CargoAction(object):
    # create messages that are used to publish feedback/result
    _feedback = cargo_actionFeedback()
    _result = cargo_actionResult()
    _goal = cargo_actionGoal()
    _execution_one = 35 + 25 + 30
    _execution_two = 70 + 25 + 50
    _execution_three = 105 + 25 + 70



    def __init__(self, name):
        self._execution_done = False
        self._order_done = False

        self.pub1 = rospy.Publisher("/cargo_order",Int64MultiArray, queue_size=10)
        self.pub2 = rospy.Publisher("/stow_order",Int64, queue_size=10)

        rospy.Subscriber("/Hello", Bool, callback=self.execuction_done)
        rospy.Subscriber("/order_done", Bool, callback=self.order_done)

        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, cargo_actionAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self._feedback.percent_complete = 0.0

    def execuction_done(self,data):
        # execucte_done_bool = Bool()
        self._execution_done = data.data
        print(data.data)

    def order_done(self,data):
        # execucte_done_bool = Bool()
        self._order_done = data.data
        print(data.data)
      
    def execute_cb(self, goal):
        # helper variables
        self._execution_done = False

        slot1 = True
        slot2 = True
        slot3 = True
        first_trip = True
        order = np.zeros((3,6), dtype=np.int64)
        order_done = False
        print("BINGO")
        max_order = 0
        num_of_big = 0
        num_of_small = 0
        stow_box = goal.priority
        order_num = goal.num_objects
        order_num = [elem for elem in order_num]
        for i in range(6):
            max_order = max_order + order_num[i]
        # if max_order > 3:
        #     print("ERROR: Please give only max. 3 cargo per order")
        for i in range(3):
            num_of_big = num_of_big + order_num[i]
        for i in range(3):
            num_of_small = num_of_small + order_num[i+3]
        print("No. of big:", num_of_big)
        print("No. of small:", num_of_small)

        num_of_trip = num_of_big

        if num_of_big == 0:
            num_of_trip = math.ceil(num_of_small/2)
            
        for i in range(num_of_trip):
            for j in range(3):
                if order_num[j] > 0 and slot1 == True: 
                    order[i,j] = order[i,j] + 1
                    order_num[j] = order_num[j] - 1
                    slot1 = False
            for j in range(3):
                if order_num[j + 3] > 0 and slot2 == True or order_num[j + 3] > 0 and slot3 == True:
                    if order_num[j + 3] > 1:
                        order[i, j + 3] = order[i, j + 3] + 2
                        order_num[j + 3] = order_num[j + 3] - 2
                        slot2 = False
                        slot3 = False
                    else:
                        order[i, j + 3] = order[i, j + 3] + 1
                        order_num[j + 3] = order_num[j + 3] - 1
                        if slot2 == True and slot3 == True:
                            slot2 = False
                        elif slot2 == False and slot3 == True:
                            slot3 = False
                        elif slot2 == False and slot3 == False:
                            pass
            slot1 = True
            slot2 = True
            slot3 = True
        print(num_of_trip)
        print(order[0,:])
        print(order[1,:])                   
        print(order[2,:]) 

        stow_order = Int64()
        stow_order.data = goal.stow_box
        self.pub2.publish(stow_order)                  
        # rospy.wait_for_message("/Hello", Bool)
        # print("It waited")
        r = rospy.Rate(1)
        success = True

        system_order = Int64MultiArray()

        # for i in range(num_of_trip)
        #     system_order.data = order[i,:]
        #     pub1.publish(system_order)
        count = 0
        start_time = rospy.get_time()
        expected_end_time = self._execution_three
        while not self._execution_done:
            if self._order_done == True or first_trip == True:
                print("Publishing to uArm + Slider")
                first_trip = False
                self._order_done = False
                system_order.data = order[count,:].tolist()
                self.pub1.publish(system_order)
                count = count + 1
                if count > 2:
                    count = 2
            current_time = rospy.get_time() - start_time
            percentage_done = 42.0
            self._feedback.percent_complete = current_time/expected_end_time
            # self._feedback.estimated_finish_time = rospy.Time.now()
            self._feedback.estimated_finish_time = rospy.Time(expected_end_time - current_time)

            # publish the feedback
            self._as.publish_feedback(self._feedback)
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()
            # print(self._feedback.percent_complete)
            # print(rospy.get_time())
            # print(rospy.get_rostime())
            # print(start_time)
            # print(expected_end_time)
            # print(current_time)
            # print(self._feedback.estimated_finish_time)
            # print(rospy.Time(self._feedback.estimated_finish_time))
            # print(rospy.Time.now())

        
        # # publish info to the console for the user
        # # rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (self._action_name, goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))
        
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

    server = CargoAction(rospy.get_name())
    rospy.spin()