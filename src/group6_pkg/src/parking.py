#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import Twist, Point, Pose, Quaternion, PoseStamped
from nav_msgs.msg import Odometry
import math
import time

# max linear speed is 0.22 m/s
# max angular speed is 2.84rad/sec
angular_z = 0.1
stop = False
stop1 = False
stop2 = False
z_info = 0
# def stop():
#     global cmd
#     cmd = Twist()
#     rospy.loginfo("STOP.")
#     cmd.linear.x = 0.0
#     cmd.angular.z = 0.0  


def log_pub(c):
    pub.publish(c)


def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
     
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
     
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
     
    return roll_x, pitch_y, yaw_z # in radians


def charuco_detector_callback(park: PoseStamped):
    # pass
    global z_info
    if stop2 == False:
        global cmd
        global angular_z, stop, stop1
        cmd = Twist()
        x_rad, y_rad, z_rad = euler_from_quaternion(park.pose.orientation.x, park.pose.orientation.y, park.pose.orientation.z, park.pose.orientation.w)
        x_degree = math.degrees(x_rad)
        # print(x_degree)

        if park.pose.position.x < 0.01 and park.pose.position.z > 0.15:
            rospy.loginfo("too far to right")
            cmd.linear.x = 0.00
            cmd.angular.z = 0.3


        if park.pose.position.x > 0.04 and park.pose.position.z > 0.15:
            rospy.loginfo("too far to left")
            cmd.linear.x = 0.00
            cmd.angular.z = -0.3

        # if park.pose.position.x < 0.01:
        #     rospy.loginfo("too far to right")
        #     cmd.linear.x = -0.05
        #     cmd.angular.z = 0.3      

        


        # if park.pose.position.x > 0.04 and park.pose.position.z > 0.15:
        #     rospy.loginfo("too far to left")
        #     cmd.linear.x = -0.05
        #     cmd.angular.z = -0.3
        
        

        if park.pose.position.x > 0.01 and park.pose.position.x < 0.04 and park.pose.position.z > 0.15:
            rospy.loginfo("happy path towards marker.")
            cmd.linear.x = -0.05
            cmd.angular.z = 0.0
        # elif park.pose.position.z < 0.15:
        #     cmd.linear.x = -0.0005
        #     angular_z = angular_z*(-1)
        #     cmd.angular.z = angular_z

        if park.pose.position.z < 0.15 and stop1 == False:
            print("BINGO")
            stop = True
            stop1 = True
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        
        
        z_info = park.pose.position.z 
        # print(park.pose.position.z)  
        log_pub(cmd)

# def odom_callback(data: Odometry):
#     global cmd

#     cmd = Twist()

#     x_rad, y_rad, z_rad = euler_from_quaternion(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
#     z_degree = math.degrees(z_rad)
#     # z_sign = math.degrees(z_rad)

#     # print(z_degree)
#     # print(data.pose.pose.position.x)

#     if z_degree < -90 and z_degree > -180 and data.pose.pose.position.x < 1.15:
#         if z_degree < -90 and z_degree > -175:
#             cmd.linear.x = 0.0
#             cmd.angular.z = -0.3
#         else:
#             rospy.loginfo("First condition achieved.")
#             cmd.linear.x = -0.08
#             cmd.angular.z = 0.0
#     # if data.pose.pose.position.x < 1.15 and z_degree < 180 and z_degree > 175:
#     #     rospy.loginfo("First condition achieved.")
#     #     cmd.linear.x = -0.08
#     #     cmd.angular.z = 0.0
#     # elif z_degree < 0 and z_degree > 0:

#     pub.publish(cmd)

# def odom_callback(data: Odometry):
#     global cmd

#     cmd = Twist()

#     x_rad, y_rad, z_rad = euler_from_quaternion(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
#     z_degree = abs(math.degrees(z_rad))

#     print(z_degree)

    # if data.pose.pose.position.x < 1.15 and z_degree < 185 and z_degree > 175:
    #     rospy.loginfo("First condition achieved.")
    #     cmd.linear.x = -0.08
    #     cmd.angular.z = 0.0

    # pub.publish(cmd)

if __name__ == '__main__':
    rospy.init_node("turtlebot_controller")
    pub = rospy.Publisher("/turtlebot1/cmd_vel", Twist, queue_size=10)
    # sub = rospy.Subscriber("/turtlebot1/odom", Odometry,callback=odom_callback)
    sub1 = rospy.Subscriber("/turtlebot1/camera/image_charuco_pose", PoseStamped,callback=charuco_detector_callback)
    rospy.loginfo("Node has been started.")
    cmd = Twist()


    while(True):
        if stop == True: 
            time.sleep(3)
            z_stop = z_info-0.08
            time_stop = z_stop/0.01
            stop2 = True
            print("ACCESS")
            print(time_stop)
            cmd.linear.x= -0.01
            cmd.angular.z = 0.0
            pub.publish(cmd)
            time.sleep(time_stop-0.5)
            cmd.linear.x = 0.0
            pub.publish(cmd)
            print("BINGO")
            stop = False