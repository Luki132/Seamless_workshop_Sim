#!/usr/bin/env python3
import rospy
import tf
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Point, Pose, Quaternion, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import math
import time


angular_z = 0.1
stop = False
stop2 = False
z_info = 0
callback_denied = False
access = False
wait_for_charuco = True
do_parking = False


def reinit():
    global angular_z, stop, stop2, z_info, callback_denied, access, wait_for_charuco, do_parking
    angular_z = 0.1
    stop = False
    stop2 = False
    z_info = 0
    callback_denied = False
    access = False
    wait_for_charuco = True
    do_parking = False




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


def charuco_detector_callback(park:PoseStamped):
    global z_info, angular_z, stop, stop2, x_ang, access, callback_denied, wait_for_charuco, do_parking
    print(park.pose.position.z)
    x_ang, y_ang, z_ang = euler_from_quaternion(park.pose.orientation.x, park.pose.orientation.y, park.pose.orientation.z, park.pose.orientation.w)
    x_ang = math.degrees(x_ang)
    
    cmd = Twist()
    if do_parking:# and callback_denied == False:

        if park.pose.position.z > 0.15:
            if park.pose.position.x < 0.005:
                rospy.loginfo("too far to right")
                cmd.linear.x = 0.00
                cmd.angular.z = 0.015


            elif park.pose.position.x > 0.045:
                rospy.loginfo("too far to left")
                cmd.linear.x = 0.00
                cmd.angular.z = -0.015

            elif abs(x_ang) > 175:
                rospy.loginfo("happy path towards marker.")
                cmd.linear.x = -0.015
                cmd.angular.z = 0.0
            pub_cmd_vel.publish(cmd)
        else:
            if not access:
                cmd = Twist()
                pub_cmd_vel.publish(cmd)
            access = True
            stop = True
        
    z_info = park.pose.position.z 
    wait_for_charuco = False
   

def park_and_stop(event=None):
    global z_info, stop,stop2, access, callback_denied, do_parking
    
    if not do_parking:
        return
    callback_denied = True

    cmd = Twist()

    if access==True:
        print("BAD BOY!")
        if stop == True:
            time.sleep(4)
            time_stop = 1.0
            while time_stop > 0:
                z_stop = z_info-0.084
                time_stop = z_stop/0.01
                print("ACCESS")
                print(time_stop - 0.5)
                if time_stop > 0:
                    #time.sleep(4)
                    cmd.linear.x= -0.01
                    cmd.angular.z = 0.0
                    pub_cmd_vel.publish(cmd)
                    time.sleep(time_stop)
                    cmd.linear.x = 0.0
                    pub_cmd_vel.publish(cmd)
                    time.sleep(4)
            print("BINGO")
            stop = False
            stop2 = True
        else:
            print(abs(x_ang))
            time.sleep(4)
            time_stop1 = (180-abs(x_ang)-7)/math.degrees(0.1)
            print("Rotate")
            print(time_stop1)
            if time_stop1 > 0:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.1
                pub_cmd_vel.publish(cmd)
                time.sleep(time_stop1)
                cmd.angular.z = 0.0
                pub_cmd_vel.publish(cmd)
                print("Stop")

    access = False
    callback_denied = False


def cb_nav(msg: String):
    global do_parking
    if msg.data == "reached:parking":
        status = Bool()
        status.data = False
        pub_state.publish(status)
        do_parking = True
        while wait_for_charuco:
            rospy.sleep(0.05)
        while not stop2:
            park_and_stop()
        status.data = True
        pub_state.publish(status)
        reinit()


if __name__ == '__main__':
    rospy.init_node("turtlebot_controller")
    pub_cmd_vel = rospy.Publisher("/turtlebot1/cmd_vel", Twist, queue_size=10)
    pub_state  = rospy.Publisher("/group6/parking_state", Bool, queue_size=10)
    # sub = rospy.Subscriber("/turtlebot1/odom", Odometry,callback=odom_callback)
    sub1 = rospy.Subscriber("/turtlebot1/camera/image_charuco_pose", PoseStamped,callback=charuco_detector_callback)
    sub_nav = rospy.Subscriber("/group6/nav_state", data_class=String, callback=cb_nav)
    #sub1 = rospy.Subscriber("/turtlebot1/amcl_pose", PoseWithCovarianceStamped,callback=amcl_callback)
    rospy.loginfo("Node has been started.")
    print("Start")
    #rospy.Timer(rospy.Duration(5), park_and_stop)
    rospy.spin()

