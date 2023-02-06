# #!/usr/bin/env python3

# import rospy
# import actionlib
# import time
# from std_msgs.msg import Float64

# from robis_messages.msg import MoveAction, GraspAction, MoveGoal, GraspGoal

# # create ROS action client:

# move_client = actionlib.SimpleActionClient('uarm1/move', MoveAction)
# grip_client = actionlib.SimpleActionClient('uarm1/grip', GraspAction)

# # start node
# #rospy.init_node('uarm1_client', anonymous=True)


# ##################################################################
# #    Added on 29/12   # testing of slider
# ##################################################################

# moveslide_client = actionlib.SimpleActionClient('/slider1/move', MoveAction)

# # start node
# rospy.init_node('slider1_client','uarm1_client', anonymous=True)#added 29/12/2022

# def move_slider(x, y, z, w) -> MoveAction.action_result:

#     moveslide_client.wait_for_server()
#     goal = MoveGoal(target=[x, y, z, w]) 
#     moveslide_client.send_goal(goal)
#     moveslide_client.wait_for_result()
#     return moveslide_client.get_result()

# ##################################################################
# #               methods that call the services                   #
# ##################################################################


# def move(x, y, z, w) -> MoveAction.action_result:
    
#     move_client.wait_for_server()
#     goal = MoveGoal(target=[x, y, z, w]) 
#     move_client.send_goal(goal)
#     move_client.wait_for_result()
#     return move_client.get_result()


# def grip(grab) -> GraspAction.action_result:

#     grip_client.wait_for_server()
#     goal = GraspGoal(grab=grab)
#     grip_client.send_goal(goal)
#     grip_client.wait_for_result()
#     return grip_client.get_result()


# #########################################################
# #      example for using the defined functions          #
# #########################################################

# def Uarm1Movement():
#     print("listenning to move and grab/command")
#     rospy.Subscriber("uarm1/move", Float64,callback=move)
#     rospy.spin()

# if __name__ == '__main__':
    

#     ######for first rect box#####

#     # print(move_slider(50, 0, 0, 0)) #slider movement, checked initial(50, 0, 0, 0)
#     # print(move(160, 160, 80, 90)) #for first rect box intermediate position, checked
#     # print(move(140, 185, -47, 90)) #for first one at the storage, touch box, checked
#     # print(grip(True)) #grabbing the box
#     # time.sleep(1)
#     # print(move(160, 160, 80, 90))
#     # time.sleep(1)
#     # print(move(180, 0, 130, 90)) 
#     # print(move_slider(0, 0, 0, 0))

#     # print(move(75, -235, 120, 150))  #for the rect boxes intermediate position
#     # print(move(79, -240, 73, 150))   #for the rect box in turtlebox
#     # print(grip(False)) 
#     # print(move(75, -235, 120, 150))  #for the rect boxes intermediate position
#     # print(move(180, 0, 130, 90))     #homed position

#     # # #######Second box########

#     print(move_slider(165, 0, 0, 0)) #slider movement
#     print(move(160, 160, 80, 90))    #for second rect box intermediate position, checked
#     print(move(140, 185, -46, 90))   #touch box
#     print(grip(True))                #grabbing the box
#     time.sleep(1)
#     print(move(160, 160, 80, 90))    #moving back to intermediate position
#     time.sleep(1)
#     print(move(180, 0, 130, 90))     #homed position
#     print(move_slider(0, 0, 0, 0))   #homed position
#     print(move(75, -235, 120, 150))  #for the rect boxes intermediate position
#     print(move(79, -240, 73, 150))   #for the rect box in turtlebox
#     print(grip(False)) 
#     print(move(75, -235, 120, 150))  #for the rect boxes intermediate position
#     print(move(180, 0, 130, 90))     #homed position

#     # # # ###### 3rd rect box#####
#     # print(move_slider(276, 0, 0, 0)) #slider movement
#     # print(move(160, 160, 80, 90)) #for 3rd rect box intermediate position, checked
#     # print(move(140, 185, -46, 90)) #touch box
#     # print(grip(True)) #grabbing the box
#     # time.sleep(1)
#     # print(move(160, 160, 80, 90))
#     # time.sleep(1)
#     # print(move(180, 0, 130, 90))
#     # print(move_slider(0, 0, 0, 0))   #homed position
#     # print(move(75, -235, 120, 150))  #for the rect boxes intermediate position
#     # print(move(79, -240, 73, 150))   #for the rect box in turtlebox
#     # print(grip(False)) 
#     # print(move(75, -235, 120, 150))  #for the rect boxes intermediate position
#     # print(move(180, 0, 130, 90))     #homed position

#     # # #######4th square box#####
#     print(move_slider(370, 0, 0, 0)) #slider movement
#     print(move(180, 160, 80, 90)) #for 4th square box intermediate position, checked
#     print(move(140, 185, -31, 90)) #touch box
#     print(grip(True)) #grabbing the box
#     time.sleep(1)
#     print(move(160, 160, 80, 90))    
#     time.sleep(1)
#     print(move(180, 0, 130, 90))
#     time.sleep(1)
#     print(move_slider(0, 0, 0, 0)) #slider movement
#     print(move(60, -280, 120, 145)) 
#     time.sleep(1)
#     print(move(60, -280, 95, 145)) #for the square boxes on turtlebot right most
#     print(grip(False)) 
#     print(move(60, -280, 120, 145)) #for the square box position, intermediate position , right most
#     print(move(180, 0, 130, 90))

#     # # # #######5th square box#####

#     # print(move_slider(452, 0, 0, 0)) #slider movement
#     # print(move(180, 160, 80, 90)) #for 5th square box intermediate position, checked
#     # print(move(140, 185, -31, 90)) #touch box
#     # print(grip(True)) #grabbing the box
#     # time.sleep(1)
#     # print(move(160, 160, 80, 90))
#     # time.sleep(1)
#     # print(move(180, 0, 130, 90))
#     # print(move_slider(0, 0, 0, 0)) #slider movement
#     # print(move(60, -280, 120, 145)) 
#     # time.sleep(1)
#     # print(move(60, -280, 95, 145)) #for the square boxes on turtlebot right most
#     # print(grip(False)) 
#     # print(move(60, -280, 120, 145)) #for the square box position, intermediate position , right most
#     # print(move(180, 0, 130, 90))


#     ########## 6th square box#####

#     print(move_slider(535, 0, 0, 0)) #slider movement
#     print(move(180, 160, 80, 90)) #for 6th square box intermediate position, checked
#     print(move(140, 185, -31, 90)) #touch box
#     print(grip(True)) #grabbing the box
#     time.sleep(1)
#     print(move(160, 160, 80, 90))
#     time.sleep(1)
#     print(move(180, 0, 130, 90))
#     time.sleep(1)
#     print(move_slider(0, 0, 0, 0)) #slider movement
#     print(move(100, -280, 120, 150)) 
#     time.sleep(1)
#     print(move(97, -278, 95, 150)) #for the square boxes on turtlebot right most
#     print(grip(False)) 
#     print(move(100, -280, 120, 150))  #for the square box position, intermediate position , left most
#     print(move(180, 0, 130, 90))


#!/usr/bin/env python3

import rospy
import actionlib
import time
from std_msgs.msg import Float64, Int64MultiArray, Bool, String

from robis_messages.msg import MoveAction, GraspAction, MoveGoal, GraspGoal

# create ROS action client:

move_client = actionlib.SimpleActionClient('uarm1/move', MoveAction)
grip_client = actionlib.SimpleActionClient('uarm1/grip', GraspAction)

big_object_done = False
small_object_done = False
small_object_counter = 0
right_space_occupied = 0


##################################################################
#    Added on 29/12   # testing of slider
##################################################################

moveslide_client = actionlib.SimpleActionClient('/slider1/move', MoveAction)
navigation_pub = rospy.Publisher("/group6/nav_goal", String, queue_size=10)

# start node
rospy.init_node('slider1_client','uarm1_client', anonymous=True)#added 29/12/2022

pub_cargo_ready = rospy.Publisher("/cargo_ready", Bool, queue_size=10)
cargo_ready = Bool()
cargo_ready = False

def move_slider(x, y, z, w) -> MoveAction.action_result:

    moveslide_client.wait_for_server()
    goal = MoveGoal(target=[x, y, z, w]) 
    moveslide_client.send_goal(goal)
    moveslide_client.wait_for_result()
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
    global cargo_ready
    big_object_done = False
    small_object_done = False
    small_object_counter = 0
    right_space_occupied = 0

    interPoss = [100, -275, 120, 145]  # intermediate positions
    inter_pos_rec = [75, -235, 120, 150]  # intermediate positions of the rectangle boxes
    interPos_slide = [140, 160, 80, 90]
    height_square = -33
    hweight_rec = -47
    y_pos = 186
    x_pos = 140
    turtle_1 = [65, -280, 95, 145] # for the square boxes on turtlebot right most
    turtle_2 = [102, -277, 95, 150]  # for the square boxes on turtlebot right most
    turtle_big = [81, -240, 73, 150]

   

    if data.data[0] == 1:
        rospy.logerr("Pick big red object")
        print(move_slider(55, 0, 0, 0)) #slider movement, checked initial(50, 0, 0, 0)
        print(move(interPos_slide[0], interPos_slide[1], interPos_slide[2], interPos_slide[3]))  #for first rect box intermediate position, checked
        print(move(x_pos, y_pos, hweight_rec, 90)) #for first one at the storage, touch box, checked
        print(grip(True)) #grabbing the box
        time.sleep(1)
        print(move(interPos_slide[0], interPos_slide[1], interPos_slide[2], interPos_slide[3])) 
        time.sleep(1)
        print(move(180, 0, 130, 90)) 
        print(move_slider(0, 0, 0, 0))

        print(move(inter_pos_rec[0], inter_pos_rec[1], inter_pos_rec[2], inter_pos_rec[3])) #for the rect boxes intermediate position
        print(move(turtle_big[0], turtle_big[1], turtle_big[2], turtle_big[3])) # for the big box on turtlebot
        print(grip(False)) 
        print(move(inter_pos_rec[0], inter_pos_rec[1], inter_pos_rec[2], inter_pos_rec[3])) # for the rect boxes intermediate position
        print(move(180, 0, 130, 90))     #homed position
        big_object_done = True

    if data.data[1] == 1 and big_object_done == False:
        rospy.logerr("Pick big green object")
        print(move_slider(165, 0, 0, 0)) #slider movement
        print(move(interPos_slide[0], interPos_slide[1], interPos_slide[2], interPos_slide[3]))    #for second rect box intermediate position, checked
        print(move(140, y_pos, hweight_rec, 90))   #touch box
        print(grip(True))                #grabbing the box
        time.sleep(1)
        print(move(interPos_slide[0], interPos_slide[1], interPos_slide[2], interPos_slide[3]))     #moving back to intermediate position
        time.sleep(1)
        print(move(180, 0, 130, 90))     #homed position
        print(move_slider(0, 0, 0, 0))   #homed position
        print(move(inter_pos_rec[0], inter_pos_rec[1], inter_pos_rec[2], inter_pos_rec[3])) #for the rect boxes intermediate position
        print(move(turtle_big[0], turtle_big[1], turtle_big[2], turtle_big[3])) # for the big box on turtlebot
        print(grip(False)) 
        print(move(inter_pos_rec[0], inter_pos_rec[1], inter_pos_rec[2], inter_pos_rec[3])) # for the rect boxes intermediate position
        print(move(180, 0, 130, 90))     #homed position
        big_object_done = True

    if data.data[2] == 1 and big_object_done == False:
        rospy.logerr("Pick big blue object")
        print(move_slider(276, 0, 0, 0)) #slider movement
        print(move(interPos_slide[0], interPos_slide[1], interPos_slide[2], interPos_slide[3]))     #for 3rd rect box intermediate position, checked
        print(move(140, y_pos, hweight_rec, 90))   #touch box
        print(grip(True))                #grabbing the box
        time.sleep(1)
        print(move(interPos_slide[0], interPos_slide[1], interPos_slide[2], interPos_slide[3])) 
        time.sleep(1)
        print(move(180, 0, 130, 90))
        print(move_slider(0, 0, 0, 0))   #homed position
        print(move(inter_pos_rec[0], inter_pos_rec[1], inter_pos_rec[2], inter_pos_rec[3])) # for the rect boxes intermediate position
        print(move(turtle_big[0], turtle_big[1], turtle_big[2], turtle_big[3])) # for the big box on turtlebot
        print(grip(False)) 
        print(move(inter_pos_rec[0], inter_pos_rec[1], inter_pos_rec[2], inter_pos_rec[3])) # for the rect boxes intermediate position
        print(move(180, 0, 130, 90))     #homed position
        big_object_done = True

    if data.data[3] > 0:
        for i in range(data.data[3]): 
            rospy.logerr("Pick small red object")
            small_object_counter = small_object_counter + 1
            print(move_slider(374, 0, 0, 0)) #slider movement
            print(move(interPos_slide[0], interPos_slide[1], interPos_slide[2], interPos_slide[3]))     #for 4th square box intermediate position, checked
            print(move(140, y_pos, height_square, 90))   #touch box
            print(grip(True))                #grabbing the box
            time.sleep(1)
            print(move(interPos_slide[0], interPos_slide[1], interPos_slide[2], interPos_slide[3]))   
            time.sleep(1)
            print(move(180, 0, 130, 90))
            time.sleep(1)
            print(move_slider(0, 0, 0, 0)) #slider movement

            if right_space_occupied == 0:
                print(move(interPoss[0], interPoss[1], interPoss[2], interPoss[3]))  # intermediate positions
                time.sleep(1)
                print(move(turtle_1[0], turtle_1[1], turtle_1[2]+20, turtle_1[3]))  #right space on the turtlebot
                print(move(turtle_1[0], turtle_1[1], turtle_1[2], turtle_1[3]))  #right space on the turtlebot
                right_space_occupied = 1
            else:  
                print(move(interPoss[0], interPoss[1], interPoss[2], interPoss[3]))
                time.sleep(1)
                print(move(turtle_2[0], turtle_2[1], turtle_2[2]+20, turtle_2[3]))  #left space on the turtlebot
                print(move(turtle_2[0], turtle_2[1], turtle_2[2], turtle_2[3]))  #left space on the turtlebot


            print(grip(False)) 
            print(move(60, -280, 120, 145)) #for the square box position, intermediate position , right most
            print(move(180, 0, 130, 90))

        if small_object_counter == 2:
            small_object_done = True

    if data.data[4] > 0 and small_object_done == False:
        for i in range(data.data[4]):
            if small_object_counter >= 2:
                small_object_done = True
                continue
            else:
                rospy.logerr("Pick small green object")
                small_object_counter = small_object_counter + 1 
                print(move_slider(461, 0, 0, 0)) #slider movement
                print(move(interPos_slide[0], interPos_slide[1], interPos_slide[2], interPos_slide[3]))     #for 5th square box intermediate position, checked
                print(move(140, y_pos, height_square, 90))   #touch box
                print(grip(True))                #grabbing the box
                time.sleep(1)
                print(move(interPos_slide[0], interPos_slide[1], interPos_slide[2], interPos_slide[3])) 
                time.sleep(1)
                print(move(180, 0, 130, 90))
                print(move_slider(0, 0, 0, 0))   #slider movement


                if right_space_occupied == 0:
                    print(move(interPoss[0], interPoss[1], interPoss[2], interPoss[3]))  # intermediate positions
                    time.sleep(1)
                    print(move(turtle_1[0], turtle_1[1], turtle_1[2]+20, turtle_1[3])) # for the square boxes on turtlebot right most
                    print(move(turtle_1[0], turtle_1[1], turtle_1[2], turtle_1[3])) # for the square boxes on turtlebot right most
                    right_space_occupied = 1
                else:  
                    print(move(interPoss[0], interPoss[1], interPoss[2], interPoss[3])) 
                    time.sleep(1)
                    print(move(turtle_2[0], turtle_2[1], turtle_2[2]+20, turtle_2[3])) # for the square boxes on turtlebot left most
                    print(move(turtle_2[0], turtle_2[1], turtle_2[2], turtle_2[3])) # for the square boxes on turtlebot left most

                print(grip(False)) 
                print(move(interPoss[0], interPoss[1], interPoss[2], interPoss[3]))     #for the square box position, intermediate position , right most
                print(move(180, 0, 130, 90))

            if small_object_counter == 2:
                small_object_done = True

    if data.data[5] > 0 and small_object_done == False:
        for i in range(data.data[5]):
            if small_object_counter >= 2:
                small_object_done = True
                continue
            else:
                rospy.logerr("Pick small blue object")
                print(move_slider(544, 0, 0, 0))  #slider movement
                print(move(interPos_slide[0], interPos_slide[1], interPos_slide[2], interPos_slide[3]))      #for 6th square box intermediate position, checked
                print(move(140, y_pos, height_square, 90))    #touch box
                print(grip(True))                 #grabbing the box
                time.sleep(1)
                print(move(interPos_slide[0], interPos_slide[1], interPos_slide[2], interPos_slide[3])) 
                time.sleep(1)
                print(move(180, 0, 130, 90))
                time.sleep(1)
                print(move_slider(0, 0, 0, 0))    #slider movement

                if right_space_occupied == 0:
                    print(move(interPoss[0], interPoss[1], interPoss[2], interPoss[3]))  # intermediate positions
                    time.sleep(1)
                    print(move(turtle_1[0], turtle_1[1], turtle_1[2]+20, turtle_1[3])) # for the square boxes on turtlebot right most
                    print(move(turtle_1[0], turtle_1[1], turtle_1[2], turtle_1[3])) # for the square boxes on turtlebot right most
                    right_space_occupied = 1
                else:  
                    print(move(interPoss[0], interPoss[1], interPoss[2], interPoss[3]))
                    time.sleep(1)
                    print(move(turtle_2[0], turtle_2[1], turtle_2[2]+20, turtle_2[3])) # for the square boxes on turtlebot left most
                    print(move(turtle_2[0], turtle_2[1], turtle_2[2], turtle_2[3])) # for the square boxes on turtlebot left most

                print(grip(False)) 
                print(move(100, -280, 120, 150))  #for the square box position, intermediate position , left most
                print(move(180, 0, 130, 90))
        
    rospy.logerr("Cargo is ready!!!")
    cargo_ready = True
    pub_cargo_ready.publish(cargo_ready)
    cargo_ready = False
    right_space_occupied = 0

    goal_move = String()
    goal_move.data = "conveyor"
    navigation_pub.publish(goal_move)
    

#########################################################
#      example for using the defined functions          #
#########################################################

def Uarm1Movement():
    print("listenning to move and grab/command")
    rospy.Subscriber("/group6/cargo_order", Int64MultiArray,callback=execute_order)
    rospy.spin()

if __name__ == '__main__':

    Uarm1Movement()
