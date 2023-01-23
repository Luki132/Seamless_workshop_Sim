#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import Twist, Point, Pose, Quaternion, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import math
import time


angular_z = 0.1
stop = False
stop1 = False
stop2 = False
z_info = 0
callback_denied = False

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
     
    return roll_x, pitch_y, yaw_z


def log_pub(c):
    pub.publish(c)

#def amcl_callback(msg):
    #rospy.sleep(1)

def charuco_detector_callback(park: PoseStamped):
    global z_info, cmd, angular_z, stop, stop1, stop2, x_ang, access, callback_denied
    print(park.pose.position.z)
    x_ang, y_ang, z_ang = euler_from_quaternion(park.pose.orientation.x, park.pose.orientation.y, park.pose.orientation.z, park.pose.orientation.w)
    x_ang = math.degrees(x_ang)
    # if stop2 == False:
    #     global cmd
    #     global angular_z, stop, stop1
    #     cmd = Twist()

    # if x_ang > 140 and x_ang < 175:
    #     rospy.loginfo("too far to left")
    #     cmd.linear.x = 0.00
    #     cmd.angular.z = -0.035
    # elif x_ang < -140 and x_ang > -175:
    #     rospy.loginfo("too far to right")
    #     cmd.linear.x = 0.00
    #     cmd.angular.z = 0.035

    # x_ang < -175 and x_ang > 175 and 
    if callback_denied == False:

        if park.pose.position.x < 0.01 and park.pose.position.z > 0.15:
            rospy.loginfo("too far to right")
            cmd.linear.x = 0.00
            cmd.angular.z = 0.015


        if park.pose.position.x > 0.04 and park.pose.position.z > 0.15:
            rospy.loginfo("too far to left")
            cmd.linear.x = 0.00
            cmd.angular.z = -0.015
        if abs(x_ang) > 175 and park.pose.position.z > 0.15:
            print("Hello")
            rospy.loginfo("happy path towards marker.")
            cmd.linear.x = -0.01
            cmd.angular.z = 0.0
        
        if park.pose.position.z < 0.15 and stop1 == False:
            print("BINGO")
            stop = True
            stop1 = True
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        
        
    z_info = park.pose.position.z 
    log_pub(cmd)
    access = True
    # print("Executing")
    # print(x_ang)
    # print(park.pose.position.z)

def park_and_stop(event):
    global z_info, stop, stop1,stop2, cmd, access, callback_denied
    print("BAD BOY!")
    callback_denied = True
    if stop == True and access==True:
        time.sleep(4)
        z_stop = z_info-0.08
        time_stop = z_stop/0.01
        stop2 = True
        print("ACCESS")
        print(time_stop - 0.5)
        if time_stop > 0:

            cmd.linear.x= -0.01
            cmd.angular.z = 0.0
            pub.publish(cmd)
            time.sleep(time_stop)
            cmd.linear.x = 0.0
            pub.publish(cmd)
            print("BINGO")
            stop = False
            stop1 = True
            stop2 = True
    elif access == True:
        time.sleep(4)
        print(abs(x_ang))
        time_stop1 = (180-abs(x_ang)-5)/math.degrees(0.1)
        print("Rotate")
        print(time_stop1)
        if time_stop1 > 0:

            cmd.linear.x = 0.0
            cmd.angular.z = 0.1
            pub.publish(cmd)
            time.sleep(time_stop1)
            cmd.angular.z = 0.0
            pub.publish(cmd)
            print("Stop")

    # rotate_and_stop()
    access = False
    callback_denied = False
# def rotate_and_stop():
#     global x_ang, cmd

#     time.sleep(4)
#     time_stop = 180 - abs(x_ang)/0.035
#     print("Rotate")
#     print(time_stop)
#     cmd.linear.x = 0.0
#     cmd.angular.z = 0.035
#     pub.publish(cmd)
#     time.sleep(time_stop)
#     cmd.angular.z = 0.0
#     pub.publish(cmd)
#     print("Stop")


if __name__ == '__main__':
    rospy.init_node("turtlebot_controller")
    pub = rospy.Publisher("/turtlebot1/cmd_vel", Twist, queue_size=10)
    # sub = rospy.Subscriber("/turtlebot1/odom", Odometry,callback=odom_callback)
    sub1 = rospy.Subscriber("/turtlebot1/camera/image_charuco_pose", PoseStamped,callback=charuco_detector_callback)
    #sub1 = rospy.Subscriber("/turtlebot1/amcl_pose", PoseWithCovarianceStamped,callback=amcl_callback)
    rospy.loginfo("Node has been started.")
    cmd = Twist()
    print("Start")
    rospy.Timer(rospy.Duration(5), park_and_stop)

    # rospy.Timer(rospy.Duration(4), rotate_and_stop)

    rospy.spin()

    # while(True):
    #     if stop == True: 
    #         time.sleep(4)
    #         z_stop = z_info-0.08
    #         time_stop = z_stop/0.01
    #         stop2 = True
    #         print("ACCESS")
    #         print(time_stop)
    #         cmd.linear.x= -0.01
    #         cmd.angular.z = 0.0
    #         pub.publish(cmd)
    #         time.sleep(time_stop)
    #         cmd.linear.x = 0.0
    #         pub.publish(cmd)
    #         print("BINGO")
    #         stop = False
    #         stop1 = True
    #         stop2 = True
    #         break