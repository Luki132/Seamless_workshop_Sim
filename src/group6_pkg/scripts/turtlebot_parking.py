#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import Twist, Point, Pose, Quaternion, PoseStamped
from nav_msgs.msg import Odometry
import math

# max linear speed is 0.22 m/s
# max angular speed is 2.84rad/sec
angular_z = 0.1

def stop():
    cmd = Twist()
    rospy.loginfo("STOP.")
    cmd.linear.x = 0.0
    cmd.angular.z = 0.0  

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
    global cmd
    global angular_z
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
        cmd.linear.x = -0.15
        cmd.angular.z = 0.0
    elif park.pose.position.z < 0.15:
        cmd.linear.x = -0.15
        angular_z = angular_z*(-1)
        cmd.angular.z = angular_z

    if park.pose.position.z < 0.085:
       stop()
            
    # print(park.pose.position.z)  
    pub.publish(cmd)

def odom_callback(data: Odometry):
    global cmd

    cmd = Twist()

    x_rad, y_rad, z_rad = euler_from_quaternion(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    z_degree = math.degrees(z_rad)
    # z_sign = math.degrees(z_rad)

    # print(z_degree)
    # print(data.pose.pose.position.x)

    if z_degree < -90 and z_degree > -180 and data.pose.pose.position.x < 1.15:
        if z_degree < -90 and z_degree > -175:
            cmd.linear.x = 0.0
            cmd.angular.z = -0.3
        else:
            rospy.loginfo("First condition achieved.")
            cmd.linear.x = -0.08
            cmd.angular.z = 0.0
    # if data.pose.pose.position.x < 1.15 and z_degree < 180 and z_degree > 175:
    #     rospy.loginfo("First condition achieved.")
    #     cmd.linear.x = -0.08
    #     cmd.angular.z = 0.0
    # elif z_degree < 0 and z_degree > 0:

    pub.publish(cmd)

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
    sub = rospy.Subscriber("/turtlebot1/odom", Odometry,callback=odom_callback)
    sub1 = rospy.Subscriber("/turtlebot1/camera/image_charuco_pose", PoseStamped,callback=charuco_detector_callback)
    rospy.loginfo("Node has been started.")

    rospy.spin()