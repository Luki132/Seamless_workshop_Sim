#!/usr/bin/env python3

import rospy
import actionlib
import time
from std_msgs.msg import Float64, Int64MultiArray, Bool

from robis_messages.msg import MoveAction, GraspAction, MoveGoal, GraspGoal

# create ROS action client:

move_client = actionlib.SimpleActionClient('uarm1/move', MoveAction)
grip_client = actionlib.SimpleActionClient('uarm1/grip', GraspAction)

big_object_done = False
small_object_done = False
small_object_counter = 0

# start node
#rospy.init_node('uarm1_client', anonymous=True)


##################################################################
#    Added on 29/12   # testing of slider
##################################################################

moveslide_client = actionlib.SimpleActionClient('/slider1/move', MoveAction)

# start node
rospy.init_node('slider1_client','uarm1_client', anonymous=True)#added 29/12/2022

pub_cargo_ready = rospy.Publisher("/cargo_ready", Bool, queue_size=10)
cargo_ready = Bool()
cargo_ready = False

def move_slider(x, y, z, w) -> MoveAction.action_result:

    moveslide_client.wait_for_server()
    goal = MoveGoal(target=[x, y, z, w]) 
    moveslide_client.send_goal(goal)
    #moveslide_client.wait_for_result()
    return moveslide_client.get_result()

##################################################################
#               methods that call the services                   #
##################################################################


def move(x, y, z, w) -> MoveAction.action_result:
    
    move_client.wait_for_server()
    goal = MoveGoal(target=[x, y, z, w]) 
    move_client.send_goal(goal)
    move_client.wait_for_result()
    return move_client.get_result()


def grip(grab) -> GraspAction.action_result:

    grip_client.wait_for_server()
    goal = GraspGoal(grab=grab)
    grip_client.send_goal(goal)
    grip_client.wait_for_result()
    return grip_client.get_result()


def execute_order(data):
    global big_object_done
    global small_object_done
    global small_object_counter
    global cargo_ready
    big_object_done = False
    small_object_done = False

    if data.data[0] == 1:
        print("Pick big red object")
        pass
    elif data.data[1] == 1 and big_object_done == False:
        print("Pick big green object")
        time.sleep(1)
        print(move_slider(50, 0, 0, 0)) #slider movement
    
        print(move(200, 175, 0, 90))

        print(move(200, 175, -31, 90)) #for green rect(2nd one at the storage)
        #print(move(130, 180, -31, 90)) #for red rect(1st one at the storage), no slider movement
        print(grip(True)) #grabbing the box
        #time.sleep(1)
        print(move(200, 175, 0, 90))
        #time.sleep(1)
        print(move_slider(0, 0, 0, 0))
        #print(move(180, 0, 100, 90))#(180,0,130,90)
        #time.sleep(1)

        print(move(75, -225, 120, 25)) #move above abit on turtlebot
        time.sleep(1)
            
        print(move(75, -225, 105, 20)) #for the rect boxes
        #print(move(65, -267, 105, 25)) #for square boxes

        time.sleep(1)
        print(grip(False))

        print(move(75, -225, 120, 25)) #move above abit on turtlebot
        time.sleep(1)
        print(move(180, 0, 130, 90))
        big_object_done = True
    elif data.data[2] == 1 and big_object_done == False:
        print("Pick big blue object")
        pass
    if data.data[3] == 1: 
        print("Pick small red object")
        small_object_counter = small_object_counter + 1
        print(move_slider(260, 0, 0, 0)) #slider movement , for red square
        time.sleep(2) 
        print(move(200, 175, 0, 90))
        moveslide_client.wait_for_result()

        time.sleep(1)
        print(move(200, 175, -31, 90))#move near the box
        time.sleep(1)
        print(grip(True)) #grabbing the box
        #time.sleep(1)
        print(move(200, 175, 0, 90))
        time.sleep(1)
        #
        #time.sleep(1)

        print(move_slider(0, 0, 0, 0))
        print(move(180, 0, 130, 90))

        print(move(65, -267, 120, 25)) #move above abit on turtlebot
        time.sleep(2)
        moveslide_client.wait_for_result()
        print(move(65, -267, 105, 25)) #for square boxes
        time.sleep(1)
        print(grip(False))
        print(move(65, -267, 120, 25)) #move above abit
        print(move(180, 0, 130, 90))  
        time.sleep(1)
        if small_object_counter == 2:
            small_object_done = True
    if data.data[4] == 1 and small_object_done == False:
        print("Pick small green object")
        small_object_counter = small_object_counter + 1 
        if small_object_counter == 2:
            small_object_done = True
    if data.data[5] == 1 and small_object_done == False:
        print("Pick small blue object")
        print(move_slider(423, 0, 0, 0)) #slider movement , for blue square
        time.sleep(3)
        print(move(205, 175, 0, 90))
        time.sleep(1)
        moveslide_client.wait_for_result()
        print(move(205, 175, -31, 90))#move near the box
        time.sleep(1)
        print(grip(True)) #grabbing the box
        time.sleep(1)
        print(move(205, 175, 0, 90))
        time.sleep(1)
        print(move_slider(0, 0, 0, 0))
        #time.sleep(1)

        print(move(180, 0, 130, 90))
        #time.sleep(1)

        print(move(110, -267, 120, 25)) #move above abit
        time.sleep(3)
        moveslide_client.wait_for_result()


        print(move(100, -267, 105, 20)) #for square boxes
        time.sleep(1)

        
        print(grip(False))
        print(move(100, -267, 120, 25)) #move above abit
        time.sleep(1)
        print(move(180, 0, 130, 90)) 
    
    rospy.loginfo("Cargo is ready!!!")
    cargo_ready = True
    pub_cargo_ready.publish(cargo_ready)
    cargo_ready = False
    

#########################################################
#      example for using the defined functions          #
#########################################################

def Uarm1Movement():
    print("listenning to move and grab/command")
    rospy.Subscriber("/cargo_order", Int64MultiArray,callback=execute_order)
    rospy.spin()

if __name__ == '__main__':

    Uarm1Movement()
    
    # time.sleep(1)
    # print(move_slider(50, 0, 0, 0)) #slider movement
 
    # print(move(200, 175, 0, 90))

    # print(move(200, 175, -31, 90)) #for green rect(2nd one at the storage)
    # #print(move(130, 180, -31, 90)) #for red rect(1st one at the storage), no slider movement
    # print(grip(True)) #grabbing the box
    # #time.sleep(1)
    # print(move(200, 175, 0, 90))
    # #time.sleep(1)
    
 
 
    # print(move_slider(0, 0, 0, 0))
    # #print(move(180, 0, 100, 90))#(180,0,130,90)
    # #time.sleep(1)

    # print(move(75, -225, 120, 25)) #move above abit on turtlebot
    # time.sleep(1)
    
    # print(move(75, -225, 105, 20)) #for the rect boxes
    # #print(move(65, -267, 105, 25)) #for square boxes

    # time.sleep(1)
    # print(grip(False))

    # print(move(75, -225, 120, 25)) #move above abit on turtlebot
    # time.sleep(1)
    # print(move(180, 0, 130, 90))


    #####2nd box##########
    # print(move_slider(260, 0, 0, 0)) #slider movement , for red square
    # time.sleep(2) 
    # print(move(200, 175, 0, 90))
    # moveslide_client.wait_for_result()

    # time.sleep(1)
    # print(move(200, 175, -31, 90))#move near the box
    # time.sleep(1)
    # print(grip(True)) #grabbing the box
    # #time.sleep(1)
    # print(move(200, 175, 0, 90))
    # time.sleep(1)
    # #
    # #time.sleep(1)

    # print(move_slider(0, 0, 0, 0))
    # print(move(180, 0, 130, 90))

    # print(move(65, -267, 120, 25)) #move above abit on turtlebot
    # time.sleep(2)
    # moveslide_client.wait_for_result()
    # print(move(65, -267, 105, 25)) #for square boxes
    # time.sleep(1)
    # print(grip(False))
    # print(move(65, -267, 120, 25)) #move above abit
    # print(move(180, 0, 130, 90))  
    # time.sleep(1)

    # ######3rd box#######
    # print(move_slider(423, 0, 0, 0)) #slider movement , for blue square
    # time.sleep(3)
    # print(move(205, 175, 0, 90))
    # time.sleep(1)
    # moveslide_client.wait_for_result()
    # print(move(205, 175, -31, 90))#move near the box
    # time.sleep(1)
    # print(grip(True)) #grabbing the box
    # time.sleep(1)
    # print(move(205, 175, 0, 90))
    # time.sleep(1)
    # print(move_slider(0, 0, 0, 0))
    # #time.sleep(1)

    # print(move(180, 0, 130, 90))
    # #time.sleep(1)

    # print(move(110, -267, 120, 25)) #move above abit
    # time.sleep(3)
    # moveslide_client.wait_for_result()


    # print(move(100, -267, 105, 20)) #for square boxes
    # time.sleep(1)

    
    # print(grip(False))
    # print(move(100, -267, 120, 25)) #move above abit
    # time.sleep(1)
    # print(move(180, 0, 130, 90)) 
