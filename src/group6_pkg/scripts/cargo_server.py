#! /usr/bin/env python3

import rospy

import actionlib
from robis_messages.msg import OrderAction
from robis_messages.msg import OrderActionGoal
from robis_messages.msg import OrderActionFeedback
from robis_messages.msg import OrderActionResult
# from group6_pkg.msg import cargo_actionAction
# from group6_pkg.msg import cargo_actionGoal
# from group6_pkg.msg import cargo_actionResult
# from group6_pkg.msg import cargo_actionFeedback
from std_msgs.msg import Int64MultiArray, Bool, Int64, String
import actionlib_tutorials.msg

import math
import numpy as np
import time 

t_sum = 0.0

# Receive /cargo_ready from uArm + Slider when stacking is complete
# Time to take one cube for converyor is 40 seconds
class CargoAction(object):
    # create messages that are used to publish feedback/result
    # _feedback = cargo_actionFeedback()
    # _result = cargo_actionResult()
    # _goal = cargo_actionGoal()
    _feedback = OrderActionFeedback
    _result = OrderActionResult
    _goal = OrderActionGoal
    _time_conveyor_to_stow_one = 48 # Add 25 seconds for every subsequent cube
    _time_wait_kinect = 12
    _time_conveyor_one = 30 # the time to just pick the cube and immediately return to source to start next trip. Add 25 seconds to every subsequent cube
    _time_add_cargo = 30
    _time_turtlebot_to_conveyor = 25 # should be 20 seconds but an additional 5 seconds for tolerance
    _time_turtlebot_to_parking = 28 # should be 25 seconds but an additional 3 seconds for tolerance
    _time_parking = 25 # should be 20 seconds but an additional 5 seconds for tolerance
    _time_slider_best_case = 40
    _time_slider_normal_case = 43
    _time_slider_worst_case = 48
    _avg_time_slider_large = (_time_slider_best_case + _time_slider_normal_case)/2
    _avg_time_slider_small = (_time_slider_worst_case + _time_slider_normal_case)/2

    phase1 = False
    phase2 = False
    phase3 = False
    phase4 = False



    # Case 1. For 3 big cargo (3 trips)
    _execution_time_3_big = _avg_time_slider_large*3 + _time_turtlebot_to_conveyor*3 + _time_conveyor_one*2 + _time_turtlebot_to_parking*2 + _time_parking*2 + _time_conveyor_to_stow_one + _time_wait_kinect*3
    # Case 2. For 2 big cargo (2 trips)
    _execution_time_2_big = _avg_time_slider_large*2 + _time_turtlebot_to_conveyor*2 + _time_conveyor_one*1 + _time_turtlebot_to_parking*1 + _time_parking*1 + _time_conveyor_to_stow_one + _time_wait_kinect*2
    # Case 3. For 1 big cargo (1 trip)
    _execution_time_1_big = _avg_time_slider_large*1 + _time_turtlebot_to_conveyor*1 + _time_conveyor_one*0 + _time_turtlebot_to_parking*0 + _time_parking*0 + _time_conveyor_to_stow_one  + _time_wait_kinect*1  
    # Case 4.For 2 big cargo + 1 small cargo (2 trip)
    _execution_time_2_big_1_small = _avg_time_slider_large*2 + _avg_time_slider_small*1 + _time_turtlebot_to_conveyor*2 + (_time_conveyor_one*1 +30) + _time_turtlebot_to_parking*1 + _time_parking*1 + _time_conveyor_to_stow_one + _time_wait_kinect*2
    # Case 5. For 1 big cargo + 2 small cargo (1 trip)
    _execution_time_1_big_2_small = _avg_time_slider_large + _avg_time_slider_small*2 +_time_turtlebot_to_conveyor + _time_conveyor_to_stow_one + 30*2 + _time_wait_kinect*1
    # Case 6. For 1 big cargo + 1 small cargo (1 trip)
    _execution_time_1_big_1_small = _avg_time_slider_large + _avg_time_slider_small +_time_turtlebot_to_conveyor + _time_conveyor_to_stow_one + 30 + _time_wait_kinect*1
    # Case 7. For 3 small cargo (2 trip)
    _execution_time_3_small = _avg_time_slider_small*3 +_time_turtlebot_to_conveyor*2 + (_time_conveyor_one + 30) +_time_conveyor_to_stow_one + _time_parking + _time_turtlebot_to_parking + _time_wait_kinect*2
    # Case 8. For 2 small cargo (1 trip)
    _execution_time_2_small = _avg_time_slider_small*2 +_time_turtlebot_to_conveyor +_time_conveyor_to_stow_one + 30 + _time_wait_kinect*1
    # Case 9. For 1 small cargo (1 trip)
    _execution_time_1_small = _avg_time_slider_small*1 +_time_turtlebot_to_conveyor +_time_conveyor_to_stow_one + _time_wait_kinect*1
    # _execution_two = 70 + 25 + 50
    # _execution_three = 105 + 25 + 70



    def __init__(self, name):
        self._execution_done = False
        self._order_done = False

        self.count_done = 0
        self.order_completed = False

        self.pub1 = rospy.Publisher("/group6/cargo_order",Int64MultiArray, queue_size=10) ## uArm + Slider
        self.pub2 = rospy.Publisher("/group6/emergency_shutdown", Bool, queue_size=10)
        rospy.Subscriber("/group6/nav_goal", String, callback=self.update_nav_goal)
        rospy.Subscriber("/group6/nav_state", String, callback=self.update_nav_state)
        rospy.Subscriber("/Hello", Bool, callback=self.execuction_done) # From conveyor
        rospy.Subscriber("/group6/parking_state", Bool, callback=self.trip_done) # From parking

        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, OrderAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self._feedback.percent_complete = 0.0

    def update_nav_goal(self, data):
        if data.data == "conveyor":
            self.phase1 = True
            print("Phase 1 set to:", True)
        elif data.data == "parking":
            self.phase3 = True


    def update_nav_state(self, data):
        if data.data == "reached:conveyor":
            self.phase2 = True
        elif data.data == "reached:parking":
            self.phase4 = True


    def execuction_done(self,data):
        if data.data == True:
            self._execution_done = data.data
            self.phase4 = True
            print("Order completed",data.data)

    def trip_done(self,data):
        self._order_done = data.data
        print("Trip completed",data.data)
        if self._order_done == True:
            print("Trip completed",data.data)
            self.count_done = self.count_done + 1

            


    #if the sequence is compromised, the emergency stop should be activated
    def check_state(self): 
        shutdown = Bool()
        shutdown.data = False
        if self.phase4 == True: 
            if self.phase1 == False or self.phase2 == False or self.phase3 == False:
                print("Sequence compromised. ABORT!")
                shutdown.data = True

        elif self.phase3 == True:
            if self.phase1 == False or self.phase2 == False:
                print("Sequence compromised. ABORT!")
                shutdown.data = True
        elif self.phase2 == True:
            if self.phase1 == False:
                print("Sequence compromised. ABORT!")
                shutdown.data = True

        self.pub2.publish(shutdown)

    def calculate_time_slot(self, count, num_of_trip, order): 
        global t1,t2,t3,t4,t_sum
        num_of_big_for_trip = 0
        num_of_small_for_trip = 0
        total_cargo = 0
        self.phase1 = False
        self.phase2 = False
        self.phase3 = False
        self.phase4 = False
        if count < num_of_trip: 
            for i in range(3):
                num_of_big_for_trip = num_of_big_for_trip + order[count, i]
                num_of_small_for_trip = num_of_small_for_trip + order[count, i + 3]
                total_cargo = num_of_small_for_trip + num_of_big_for_trip

        t1 = self._avg_time_slider_large*num_of_big_for_trip + self._avg_time_slider_small*num_of_small_for_trip
        t2 = self._time_turtlebot_to_conveyor

        if count == num_of_trip - 1:
            t3 = self._time_conveyor_to_stow_one + self._time_add_cargo*(total_cargo- 1) + self._time_wait_kinect
            t4 = 0
            self.phase4 = True # Not the final trip, so system does not need to check for this
        else:
            t3 = self._time_conveyor_one + self._time_add_cargo*(total_cargo - 1) + self._time_wait_kinect
            t4 = self._time_turtlebot_to_parking + self._time_parking
        
        t_sum = t_sum + t1 + t2 + t3 + t4

        print("time to stack cargo",  t1)
        print("time to drive to conveyor",  t2)
        print("time to wait till conveyor is done",  t3)
        print("time to return and park",  t4)
        print("Sum of previous and current trip time:", t_sum)



    def is_system_on_time(self, t1, t2, t3, t4, t_sum, current_time):
        if self.phase1 == False:
            if current_time > t1 + t_sum:
                print("System is behind phase 1")
        elif self.phase2 == False:
            if current_time > t1 + t2 + t_sum:
                print("System is behind phase 2")
        elif self.phase3 == False:
            if current_time > t1 + t2 + t3 + t_sum:
                print("System is behind phase 3")
        elif self.phase4 == False:
            if current_time > t1 + t2 + t3 + t4 + t_sum:
                print("System is behind phase 4")

    def execute_cb(self, goal):
        global num_of_trip
        # helper variables
        self._execution_done = False
        self.count_done = 0
        self.order_completed = False

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
        stow_order = goal.stow_box
        rospy.set_param("/group6/stow_order", stow_order)
        # rospy.wait_for_message("/Hello", Bool)
        # print("It waited")
        r = rospy.Rate(1)
        self.success = True
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

        # Calculate the number of trips needed
        num_of_trip = num_of_big

        if num_of_big == 0:
            num_of_trip = math.ceil(num_of_small/2)
            
        # Assign the cargo to different trips
        for i in range(num_of_trip):
            # Plan trip for large cargo
            for j in range(3):
                if order_num[j] > 0 and slot1 == True: 
                    order[i,j] = order[i,j] + 1
                    order_num[j] = order_num[j] - 1
                    slot1 = False
            # Plan trip for small cargo
            for j in range(3):
                if order_num[j + 3] > 0 and slot2 == True or order_num[j + 3] > 0 and slot3 == True: 
                    # Allow a trip to take 2 small cargo, otherwise it may only take one small cargo
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
        print(order[0,:]) #Print the first trip
        print(order[1,:]) #Print the second trip                   
        print(order[2,:]) #Print the third trip
        expected_end_time = None
        if num_of_trip == 3: 
            expected_end_time = self._execution_time_3_big

        elif num_of_trip == 2: 
            if num_of_big == 0:
                expected_end_time = self._execution_time_3_small
            elif num_of_small == 0:
                expected_end_time = self._execution_time_2_big
            else:
                expected_end_time = self._execution_time_2_big_1_small

        elif num_of_trip == 1: 
            if num_of_small == 0:
                expected_end_time = self._execution_time_1_big
            elif num_of_small == 2:
                if num_of_big == 1:
                    expected_end_time = self._execution_time_1_big_2_small
                else: 
                    expected_end_time = self._execution_time_2_small
            elif num_of_small == 1:
                if num_of_big == 1:
                    expected_end_time = self._execution_time_1_big_1_small
                else: 
                    expected_end_time = self._execution_time_1_small



        system_order = Int64MultiArray()

        # for i in range(num_of_trip)
        #     system_order.data = order[i,:]
        #     pub1.publish(system_order)
        count = 0
        start_time = rospy.get_time() # Get the start time

        while not self._execution_done and not rospy.is_shutdown() and not self.order_completed: #If KeyboardInterrupt is triggered the system will stop
            if self._order_done == True or first_trip == True:
                print("Publishing to uArm + Slider")
                first_trip = False
                self._order_done = False
                if sum(order[count,:]) != 0:
                    system_order.data = order[count,:].tolist() #convert the numpy array into a list, that can be publish
                    self.pub1.publish(system_order) 
                
                self.calculate_time_slot(count, num_of_trip, order)

                count = count + 1
                if count > 2: # Ensure that maximum trip is kept to 3 trips
                    count = 2
            
            current_time = rospy.get_time() - start_time
            print("This is the current time", current_time)

            self.is_system_on_time(t1, t2, t3, t4, t_sum, current_time)
            # self._feedback.estimated_finish_time = rospy.Time.now()
            remainder_time = (expected_end_time - current_time)/1000000000
            if remainder_time > 0:
                self._feedback.estimated_finish_time = rospy.Time(remainder_time) 
            else: 
                self._feedback.estimated_finish_time = rospy.Time(0)
            self._feedback.percent_complete = current_time/expected_end_time 
            if self._feedback.percent_complete > 1.0:
                print("time exceeded")
                self._feedback.percent_complete = 1.0

            print(("This is the feedback time",self._feedback.estimated_finish_time ))
            # publish the feedback
            self._as.publish_feedback(self._feedback)

            r.sleep()

            if self.count_done == num_of_trip:
                self.order_completed = True
                self.success = True


        print("Order completed")
        # if self._execution_done == True:
        #     self.success = True
        # self.success = True
        # # start executing the action
        # for i in range(10):
        #     # check that preempt has not been requested by the client
        #     if self._as.is_preempt_requested():
        #         rospy.loginfo('%s: Preempted' % self._action_name)
        #         self._as.set_preempted()
        #         self.success = False
        #         break
        #     self._feedback.percent_complete = self._feedback.percent_complete + 1.0
        #     # publish the feedback
        #     self._as.publish_feedback(self._feedback)
        #     # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
        #     r.sleep()
          
        if self.success:
            self._result.finish_time = rospy.Time(current_time/1000000000)
            self._result.duration = start_time - current_time
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

        
        
if __name__ == '__main__':
    try:
        rospy.init_node('cargo_server')
        server = CargoAction(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass