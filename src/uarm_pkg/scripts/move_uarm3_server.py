#!/usr/bin/env python3
import rospy
from robis_messages.msg import MoveAction, MoveGoal, GraspAction, GraspGoal
from group_messages.srv import order_cube1, move_int, move_uarm_test
# from group_messages.msg import turtle_pos
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseArray
import time
import actionlib
import math
from tf.transformations import euler_from_quaternion

class get_turtle_pos:

    def __init__(self):
        self.initial_odom_x = 1268.201
        self.initial_odom_y = 783.632
        self.z_pos_turtlebot = 141 # in mm

        self.pos_cube3_on_conv = [200, 159, 60, 90]
        self.pos_onTur1 = [169, -83, 141, 90]
        self.pos_onTur2 = [150, -120, 141, 90]
        self.pos_onTur3 = [187, -120, 141, 90]
        self.pos_of_turtle = [1268.201, 783.632] # initialization of position
        self.zorient_ofturtle = 0

        # rospy.Subscriber("/turtlebot1/odom", Odometry, self.callback_odom)
        s = rospy.Service('uarm3_controll/move', order_cube1, self.handle_move_uarm3_sim)
        rospy.Subscriber("/uarm3_move_odom/move", Float64, self.callback_move_uarm3_odom)
        rospy.Subscriber("/cargo_position", PoseArray, self.callback_cargo_pos)

    def callback_move_uarm3_odom(self, data):
        # pos = self.get_turtle_pos()
        # cube_nr = int(data.data)
        # move_pos = pos[cube_nr]
        # self.move_uarm3([move_pos[0], -move_pos[1],200, 90])
        # self.move_uarm3([move_pos[0], -move_pos[1], 141, 90])
        # return True

        cube_nr = int(data.data)
        pos_get = [100, 100, 150, 90]
        if cube_nr == 0:
            pos_get = self.pos_onTur1
        elif cube_nr == 1:
            pos_get = self.pos_onTur2
        elif cube_nr == 2:
            pos_get = self.pos_onTur3
        
        self.move_uarm3([pos_get[0], -pos_get[1] + 50 ,pos_get[2] + 60, pos_get[3]])
        self.move_uarm3([pos_get[0], -pos_get[1],pos_get[2] + 60, pos_get[3]])
        self.move_uarm3([pos_get[0], -pos_get[1], pos_get[2], pos_get[3]])
        return True


    def get_turtle_pos(self):
        # berechnungen wie man auf die arm pose kommt...
        # ich nehme den X-wer: ziehe 250 ab und nochmal 127 um zu boxen 2 und 3 zu kommen, und 167 um zu box 1 zu kommen
        # ich nehme den Y- Wert und ziehe 270 ab um zu Box 1 zu kommen; ziehe 32 ab um zur Box2 zu kommen und addiere 32 um zu Box 1 zu kommen
        offset_x_uarm3 = 250 # in mm
        offset_y_uarm3 = 270 # in mm
        z_pos_turtlebot = 141 # in mm
        if self.pos_of_turtle[0] < 470 and 100 < self.pos_of_turtle[1] < 440:
            pos1 = [self.pos_of_turtle[1]-251, self.pos_of_turtle[0]-379]
            pos2 = [self.pos_of_turtle[1]-289, self.pos_of_turtle[0]-379]
            pos3 = [self.pos_of_turtle[1]-270, self.pos_of_turtle[0]-416]
            pos =  [pos1,pos2 ,pos3]
            return pos
        else:
            print("can't get the position of the turtlebot")
            pos1 = [self.pos_of_turtle[1]-251, self.pos_of_turtle[0]-379]
            pos2 = [self.pos_of_turtle[1]-289, self.pos_of_turtle[0]-379]
            pos3 = [self.pos_of_turtle[1]-270, self.pos_of_turtle[0]-416]
            pos =  [pos1,pos2 ,pos3]
            return pos

    def callback_odom(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        x_rad, y_rad, z_rad = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
        z_degree = math.degrees(z_rad)
        self.zorient_ofturtle = z_degree
        self.pos_of_turtle = [x*1000, y*1000]

    def callback_cargo_pos(self, coordinates):
        pos2 = [coordinates.poses[1].position.x*1000-250, coordinates.poses[1].position.y*1000-270]
        pos3 = [coordinates.poses[2].position.x*1000-250, coordinates.poses[2].position.y*1000-270]
        pos1 = [coordinates.poses[0].position.x*1000-250, coordinates.poses[0].position.y*1000-270]
        self.pos_onTur1 = [pos1[1], -pos1[0], 141, 90]
        self.pos_onTur2 = [pos2[1], -pos2[0], 141, 90]
        self.pos_onTur3 = [pos3[1], -pos3[0], 141, 90]


    def handle_move_uarm3_sim(self, order):
        rospy.logdebug("I heard %s", order.storage2)
        print("data: " + str(order.storage1) + " " + str(order.storage2) + " " + str(order.storage3))
    
        storage = [order.storage1, order.storage2, order.storage3]
        positions = [self.pos_onTur1, self.pos_onTur2, self.pos_onTur3]
        for cube in range(3):
            if storage[cube]:
                rospy.loginfo("Getting Cube Nr. %s out of 3", cube)
                pos = positions[cube]
                self.move_uarm3([pos[0], pos[1]+80, pos[2]+20, pos[3]])
                self.move_uarm3([pos[0], pos[1], pos[2]+20, pos[3]])
                self.move_uarm3([pos[0], pos[1], pos[2], pos[3]])
                time.sleep(1)
                self.grip_uarm3(True)
                time.sleep(1)
                self.move_uarm3([pos[0], pos[1], pos[2]+60, pos[3]])
                self.move_uarm3([pos[0], pos[1]+ 100, pos[2]+60, pos[3]])
                self.move_uarm3([pos[0], pos[1]+100, pos[2], pos[3]])
                #time.sleep(1)
                self.move_uarm3([self.pos_cube3_on_conv[0], self.pos_cube3_on_conv[1], self.pos_cube3_on_conv[2]+70, self.pos_cube3_on_conv[3]])
                self.move_uarm3(self.pos_cube3_on_conv)
                #time.sleep(1)
                self.grip_uarm3(False)
                #time.sleep(1)
                self.move_uarm3([self.pos_cube3_on_conv[0], self.pos_cube3_on_conv[1], self.pos_cube3_on_conv[2]+50, self.pos_cube3_on_conv[3]])
                time.sleep(1)
    
                rospy.wait_for_service('conveyor_controll/move')
                conveyor_client = rospy.ServiceProxy('conveyor_controll/move', move_int)
                goal = move_int._request_class(length=-250)
                result = conveyor_client(goal)
                print("The result of the Service is: " + str(result))
            else:
                # move uarm to turtelbot and take the stone and put it on the conveyor
                pass
        return True

    def handle_move_uarm3_real_hardware(self, order):
        rospy.loginfo("I heard %s", order.storage1)
        rospy.logdebug("I heard %s", order.storage2)
        print("data: " + str(order.storage1) + " " + str(order.storage2) + " " + str(order.storage3))

        storage = [order.storage1, order.storage2, order.storage3]
        positions = [order.pos1, order.pos2, order.pos3]
        # for cube in range(3):
            # if storage[cube]:
        rospy.loginfo("Getting Cube Nr. %s out of 3", storage[0])
        pos = positions[0]
        for i in range(2):
            self.move_uarm3([pos[0], pos[1], pos[2]+30, pos[3]])
            self.move_uarm3([pos[0], pos[1], pos[2]+10, pos[3]])
            self.move_uarm3([pos[0], pos[1], pos[2], pos[3]])
            time.sleep(1)
            self.grip_uarm3(True)
            time.sleep(1)
            self.move_uarm3([150, -120, 141, 90])
            time.sleep(2)
            self.move_uarm3([pos[0], pos[1], pos[2]+20, pos[3]])
            self.move_uarm3(pos)
            # time.sleep(1)
            self.grip_uarm3(False)
            # time.sleep(1)
            self.move_uarm3([pos[0], pos[1], pos[2]+20, pos[3]])
            self.move_uarm3([150, -120, 141, 90])
            time.sleep(1)

            # else:
                # move uarm to turtelbot and take the stone and put it on the conveyor
                # pass
            rospy.wait_for_service('conveyor_controll/move')
            conveyor_client = rospy.ServiceProxy('conveyor_controll/move', move_int)
            goal = move_int._request_class(length=220)
            result = conveyor_client(goal)
            print("The result of the Service is: " + str(result))

        return True


    def move_uarm3(self, pos):
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

    def grip_uarm3(self, grab):
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
    uarm3 = get_turtle_pos()
    rospy.spin()