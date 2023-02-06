#!/usr/bin/env python3
import rospy
from robis_messages.msg import MoveAction, MoveGoal, GraspAction, GraspGoal
from group_messages.srv import order_cube1, move_uarm_test
from group_messages.msg import conv_cube
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64,String, Int64, Bool
from geometry_msgs.msg import PoseArray
import time
import actionlib
import math
from tf.transformations import euler_from_quaternion

best_pos_of_turtle = [200, -170, 170, 90]

Area = "A"

class get_turtle_pos:

    def __init__(self):

        if Area == "A":
            self.z_tur_s = 140 # in mm
            self.z_tur_b = 124 # in mm
        elif Area == "B":
            self.z_tur_s = 129 # in mm
            self.z_tur_b = 115 # in mm
        else:
            self.z_tur_s = 140 # in mm
            self.z_tur_b = 124 # in mm

        self.pos_cube3_on_conv = [200, 159, 60, 90]
        self.pos_cube3_on_conv_1 = [200, 159, 36, 90] # big Cube but thin cube
        self.pos_cube3_on_conv_2 = [200, 159, 50, 90] # small quadratic cube
        self.x_val_raw  = [169, -80]
        self.y_val_raw = [150, -120]
        self.z_val_raw = [187, -120]

        self.pos_onTur1 = [169, -80]
        self.pos_onTur2 = [150, -120]
        self.pos_onTur3 = [187, -120]
        self.pos_of_turtle = [1268.201, 783.632] # initialization of position
        self.cubes_detected = [False, False, False]
        self.pos_of_big_cube = 3 # 1, 2 or 3

        # rospy.Subscriber("/turtlebot1/odom", Odometry, self.callback_odom)
        rospy.Subscriber("/group6/nav_state", String, self.Turtlebot_is_there)
        s = rospy.Service('uarm3_controll/move', order_cube1, self.handle_move_uarm3_real_hardware)
        self.navigation_pub = rospy.Publisher("/group6/nav_goal", String, queue_size=10)
        rospy.Subscriber("/uarm3_move_odom/move", Float64, self.callback_move_uarm3_odom)
        rospy.Subscriber("/cargo_position", PoseArray, self.callback_cargo_pos_raw)
        self.conveyor_pub = rospy.Publisher("conveyor_controll/move", conv_cube)

    def callback_move_uarm3_odom(self, data):
        # pos = self.get_turtle_pos()
        # cube_nr = int(data.data)
        # move_pos = pos[cube_nr]
        # self.move_uarm3([move_pos[0], -move_pos[1],200, 90])
        # self.move_uarm3([move_pos[0], -move_pos[1], 141, 90])
        # return True
        self.move_uarm3([150, -40, 145, 90])
        cube_nr = int(data.data)
        pos_get = [100, 100, 150, 90]
        
        if cube_nr == 1:
            if self.pos_of_big_cube == 1:
                pos_get = self.pos_onTur1
                pos_get.append(124)
            else:
                pos_get = self.pos_onTur1
                pos_get.append(140)
        elif cube_nr == 2:
            if self.pos_of_big_cube == 2:
                pos_get = self.pos_onTur2
                pos_get.append(124)
            else:
                pos_get = self.pos_onTur2
                pos_get.append(140)
        elif cube_nr == 3:
            if self.pos_of_big_cube == 3:
                pos_get = self.pos_onTur3
                pos_get.append(124)
            else:
                pos_get = self.pos_onTur3
                pos_get.append(140)
        
        
        self.move_uarm3([pos_get[0], pos_get[1] + 60, pos_get[2] + 10, 90])
        self.move_uarm3([pos_get[0], pos_get[1],  pos_get[2] + 10, 90])
        self.move_uarm3([pos_get[0], pos_get[1],  pos_get[2], 90])
        return True

    def Turtlebot_is_there(self, data):
        state = data.data
        if state == "reached:conveyor":
            goal_move = order_cube1._request_class(storage1 = True, storage2 = True, storage3 = True)
            self.handle_move_uarm3_real_hardware(goal_move)
        else:
            pass



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
        # odom data in Simulation!!!
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        x_rad, y_rad, z_rad = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
        z_degree = math.degrees(z_rad)
        self.zorient_ofturtle = z_degree
        self.pos_of_turtle = [x*1000, y*1000]

    def callback_cargo_pos_raw(self, coordinates):
        x_val = [coordinates.poses[0].position.x, coordinates.poses[1].position.x, coordinates.poses[2].position.x]
        y_val = [coordinates.poses[0].position.y, coordinates.poses[1].position.y, coordinates.poses[2].position.y]
        z_val = [coordinates.poses[0].position.z, coordinates.poses[1].position.z, coordinates.poses[2].position.z]
        self.x_val_raw = x_val
        self.y_val_raw = y_val
        self.z_val_raw = z_val
        # rospy.logerr("x: %s, y: %s, z: %s", x_val, y_val, z_val)
        return True

    def callback_cargo_pos(self):
        # kinect camera data: onlie works in simulation right now
        x_hardware_C = 240
        y_hardware_C = 260
        x_hardware_A = 232
        y_hardware_A = 254

        cube_det = [True, True, True] # assume that 3 cubes are on the turtlebot
        biggest_num_of_detected_cubes = 0 # innitialize that zero cubes are detected
        num_of_positions = 3
        for i in range(70):
            x_values_test = self.x_val_raw
            y_values_test = self.y_val_raw
            cube_area_test = self.z_val_raw
            rospy.logerr("x_1: %s, y_1: %s, z_1: %s", x_values_test, y_values_test, cube_area_test)
            if cube_area_test[0] == 0.0:
                num_of_positions -= 1
            if cube_area_test[1] == 0.0:
                num_of_positions -= 1
            if cube_area_test[2] == 0.0:
                num_of_positions -= 1
            if num_of_positions > biggest_num_of_detected_cubes:
                biggest_num_of_detected_cubes = num_of_positions
                x_values = [x_values_test[0], x_values_test[1], x_values_test[2]]
                y_values = [y_values_test[0], y_values_test[1], y_values_test[2]]
                cube_area = [cube_area_test[0], cube_area_test[1], cube_area_test[2]]
                for j in range(3 - biggest_num_of_detected_cubes):
                    x_values.pop(-1)
                    y_values.pop(-1)
                    cube_area.pop(-1)
                    cube_det[-(j+1)] = False
            time.sleep(0.1)

        # now I have a list with the x and y values of the detected cubes and the area of the cubes
        pos1 = []
        if biggest_num_of_detected_cubes == 0:
            return
        if  max(cube_area) > 1800:
            pb = cube_area.index(max(cube_area)) # index of big cube
            self.pos_of_big_cube = 1
            # y_values[pb] = y_values[pb] - 0.3
            pos1.append([x_values[pb]*1000-x_hardware_A, y_values[pb]*1000-y_hardware_A])
        else:
            pb = 4 # just some number which is not in the range
            self.pos_of_big_cube = 4

        for i in range(biggest_num_of_detected_cubes):
            if i != pb:
                pos1.append([x_values[i]*1000-x_hardware_A, y_values[i]*1000-y_hardware_A])

        self.cubes_detected = cube_det
        # rospy.logerr("X: %s", self.pos_of_big_cube)

        for i in range(len(pos1)):
            if i == 0:
                self.pos_onTur1 = [pos1[i][1], -pos1[i][0]]
            if i == 1:
                self.pos_onTur2 = [pos1[i][1], -pos1[i][0]]
            if i == 2:
                self.pos_onTur3 = [pos1[i][1], -pos1[i][0]]

        return True


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
                self.wait_until_var("/conveyor_moved")
                self.move_uarm3(self.pos_cube3_on_conv)
                rospy.set_param('/conveyor_moved', False)
                
                #time.sleep(1)
                self.grip_uarm3(False)
                #time.sleep(1)
                self.move_uarm3([self.pos_cube3_on_conv[0], self.pos_cube3_on_conv[1], self.pos_cube3_on_conv[2]+50, self.pos_cube3_on_conv[3]])
                msg = conv_cube()
                msg.length = -250
                msg.size = 1
                self.conveyor_pub.publish(msg)
                time.sleep(15) # without that it doesnt work:after a specific time the conveyor stops
            else:
                # move uarm to turtelbot and take the stone and put it on the conveyor
                pass
        return True

    def handle_move_uarm3_real_hardware(self, order):
        time.sleep(4)
        self.callback_cargo_pos()
        storage = [order.storage1, order.storage2, order.storage3]
        # positions = [[169, -83, 124, 90], [150, -126, 140, 90], [187, -126, 140, 90]]
        positions = [self.pos_onTur1, self.pos_onTur2, self.pos_onTur3]
        pos_of_biggest_cube = self.pos_of_big_cube
        detected_cubes_kinect = self.cubes_detected
        for i in range(3):
            if i + 1 == pos_of_biggest_cube:
                positions[i].append(self.z_tur_b)
                positions[i].append(90)
            else:
                positions[i].append(self.z_tur_s)
                positions[i].append(90)

        # positions = [[self.pos_onTur1[0], self.pos_onTur1[1],124, 90], [self.pos_onTur2[0], self.pos_onTur2[1], 140, 90],[self.pos_onTur3[0], self.pos_onTur3[1], 140, 90]]
        pos_cube_on_conv = []
        # for cube in range(3):
            # if storage[cube]:
        rospy.logerr("X: %s", detected_cubes_kinect)
        rospy.logerr("X: %s", pos_of_biggest_cube)

        pos = positions[0]
        for cube in range(3):
            if storage[cube] and detected_cubes_kinect[cube]:
                rospy.loginfo("Getting Cube Nr. %s out of 3", cube)
                pos = positions[cube]
                self.move_uarm3([pos[0]-40, pos[1]+70, pos[2]+10, pos[3]])
                self.move_uarm3([pos[0], pos[1], pos[2]+10, pos[3]])
                self.move_uarm3([pos[0], pos[1], pos[2], pos[3]])
                time.sleep(1)
                self.grip_uarm3(True)
                time.sleep(1)

                if self.move_uarm3([pos[0], pos[1], pos[2]+15, pos[3]]):
                    pass
                else: self.move_uarm3([pos[0], pos[1], pos[2]+10, pos[3]])

                self.move_uarm3([pos[0], pos[1]+ 50, pos[2]+15, pos[3]])
                self.move_uarm3([pos[0]+30, pos[1]+ 80, pos[2]+15, pos[3]])
                #time.sleep(1)
                self.move_uarm3([200, -20, 140, 90])
                self.move_uarm3([self.pos_cube3_on_conv_1[0], self.pos_cube3_on_conv_1[1], self.pos_cube3_on_conv_1[2]+45, self.pos_cube3_on_conv_1[3]])
                self.wait_until_var("/conveyor_moved")
                rospy.set_param('/conveyor_moved', False)
                msg = conv_cube()
                msg.length = 250
                if cube + 1 == pos_of_biggest_cube:
                    self.move_uarm3(self.pos_cube3_on_conv_1)
                    msg.size = 1
                else:
                    self.move_uarm3(self.pos_cube3_on_conv_2)
                    msg.size = 2
                #time.sleep(1)
                self.grip_uarm3(False)
                #time.sleep(1)
                self.move_uarm3([self.pos_cube3_on_conv_1[0], self.pos_cube3_on_conv_1[1], self.pos_cube3_on_conv_1[2]+90, self.pos_cube3_on_conv_1[3]])

                self.conveyor_pub.publish(msg)
                time.sleep(2) # without that it doesnt work:after a specific time the conveyor stops

        
        goal_move = String()
        goal_move.data = "parking"
        self.navigation_pub.publish(goal_move)
        return True

    def wait_until_var(self, var):
        while rospy.has_param(var) and not rospy.get_param(var):
            pass

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

    def check_for_emergency(self):
        while True:
            if rospy.get_param("/group6/emergency") == True:
                time.sleep(1)
            else:
                break


if __name__ == '__main__':
    # Starts a new node
    rospy.init_node('handle_uarm3_server', anonymous=True)
    rospy.loginfo("I heard TEst")
    # create new Service server
    uarm3 = get_turtle_pos()
    rospy.spin()