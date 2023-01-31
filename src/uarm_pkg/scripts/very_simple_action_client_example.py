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
from std_msgs.msg import Float64, Int64MultiArray, Bool

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
    global big_object_done
    global small_object_done
    global small_object_counter
    global cargo_ready
    big_object_done = False
    small_object_done = False
   

    if data.data[0] == 1:
        print("Pick big red object")
        print(move_slider(50, 0, 0, 0)) #slider movement, checked initial(50, 0, 0, 0)
        print(move(160, 160, 80, 90)) #for first rect box intermediate position, checked
        print(move(140, 185, -47, 90)) #for first one at the storage, touch box, checked
        print(grip(True)) #grabbing the box
        time.sleep(1)
        print(move(160, 160, 80, 90))
        time.sleep(1)
        print(move(180, 0, 130, 90)) 
        print(move_slider(0, 0, 0, 0))

        print(move(75, -235, 120, 150))  #for the rect boxes intermediate position
        print(move(79, -240, 73, 150))   #for the rect box in turtlebox
        print(grip(False)) 
        print(move(75, -235, 120, 150))  #for the rect boxes intermediate position
        print(move(180, 0, 130, 90))     #homed position
        big_object_done = True

    elif data.data[1] == 1 and big_object_done == False:
        print("Pick big green object")
        print(move_slider(165, 0, 0, 0)) #slider movement
        print(move(160, 160, 80, 90))    #for second rect box intermediate position, checked
        print(move(140, 185, -46, 90))   #touch box
        print(grip(True))                #grabbing the box
        time.sleep(1)
        print(move(160, 160, 80, 90))    #moving back to intermediate position
        time.sleep(1)
        print(move(180, 0, 130, 90))     #homed position
        print(move_slider(0, 0, 0, 0))   #homed position
        print(move(75, -235, 120, 150))  #for the rect boxes intermediate position
        print(move(79, -240, 73, 150))   #for the rect box in turtlebox
        print(grip(False)) 
        print(move(75, -235, 120, 150))  #for the rect boxes intermediate position
        print(move(180, 0, 130, 90))     #homed position
        big_object_done = True

    elif data.data[2] == 1 and big_object_done == False:
        print("Pick big blue object")
        print(move_slider(276, 0, 0, 0)) #slider movement
        print(move(160, 160, 80, 90)) #for 3rd rect box intermediate position, checked
        print(move(140, 185, -46, 90)) #touch box
        print(grip(True)) #grabbing the box
        time.sleep(1)
        print(move(160, 160, 80, 90))
        time.sleep(1)
        print(move(180, 0, 130, 90))
        print(move_slider(0, 0, 0, 0))   #homed position
        print(move(75, -235, 120, 150))  #for the rect boxes intermediate position
        print(move(79, -240, 73, 150))   #for the rect box in turtlebox
        print(grip(False)) 
        print(move(75, -235, 120, 150))  #for the rect boxes intermediate position
        print(move(180, 0, 130, 90))     #homed position
        big_object_done = True

    if data.data[3] == 1: 
        print("Pick small red object")
        small_object_counter = small_object_counter + 1
        print(move_slider(370, 0, 0, 0)) #slider movement
        print(move(180, 160, 80, 90)) #for 4th square box intermediate position, checked
        print(move(140, 185, -31, 90)) #touch box
        print(grip(True)) #grabbing the box
        time.sleep(1)
        print(move(160, 160, 80, 90))    
        time.sleep(1)
        print(move(180, 0, 130, 90))
        time.sleep(1)
        print(move_slider(0, 0, 0, 0)) #slider movement
        
       
        print(move(60, -280, 120, 145)) 
        time.sleep(1)
        print(move(60, -280, 95, 145)) #for the square boxes on turtlebot right most
        print(grip(False)) 
        print(move(60, -280, 120, 145)) #for the square box position, intermediate position , right most
        print(move(180, 0, 130, 90))
        right_space_occupied = 1

        if small_object_counter == 2:
            small_object_done = True

    if data.data[4] == 1 and small_object_done == False:
        print("Pick small green object")
        small_object_counter = small_object_counter + 1 
        print(move_slider(452, 0, 0, 0)) #slider movement
        print(move(180, 160, 80, 90)) #for 5th square box intermediate position, checked
        print(move(140, 185, -31, 90)) #touch box
        print(grip(True)) #grabbing the box
        time.sleep(1)
        print(move(160, 160, 80, 90))
        time.sleep(1)
        print(move(180, 0, 130, 90))
        print(move_slider(0, 0, 0, 0)) #slider movement
       

        if right_space_occupied == 0:
            print(move(60, -280, 120, 145)) # intermediate positions
            time.sleep(1)
            print(move(60, -280, 95, 145)) #for the square boxes on turtlebot right most
        else:  
            print(move(60, -280, 120, 145)) 
            time.sleep(1)
            print(move(97, -278, 95, 150))  #left space on the turtlebot

        print(grip(False)) 
        print(move(60, -280, 120, 145)) #for the square box position, intermediate position , right most
        print(move(180, 0, 130, 90))

        if small_object_counter == 2:
            small_object_done = True

    if data.data[5] == 1 and small_object_done == False:
        print("Pick small blue object")
        print(move_slider(535, 0, 0, 0)) #slider movement
        print(move(180, 160, 80, 90)) #for 6th square box intermediate position, checked
        print(move(140, 185, -31, 90)) #touch box
        print(grip(True)) #grabbing the box
        time.sleep(1)
        print(move(160, 160, 80, 90))
        time.sleep(1)
        print(move(180, 0, 130, 90))
        time.sleep(1)
        print(move_slider(0, 0, 0, 0)) #slider movement
        print(move(100, -280, 120, 150)) 
        time.sleep(1)
        print(move(97, -278, 95, 150)) #for the square boxes on turtlebot left most
        print(grip(False)) 
        print(move(100, -280, 120, 150))  #for the square box position, intermediate position , left most
        print(move(180, 0, 130, 90))
        
    rospy.loginfo("Cargo is ready!!!")
    cargo_ready = True
    pub_cargo_ready.publish(cargo_ready)
    cargo_ready = False
    right_space_occupied = 0 
    

#########################################################
#      example for using the defined functions          #
#########################################################

def Uarm1Movement():
    print("listenning to move and grab/command")
    rospy.Subscriber("/cargo_order", Int64MultiArray,callback=execute_order)
    rospy.spin()

if __name__ == '__main__':

    Uarm1Movement()
