#!/usr/bin/env python3
import math
import rospy
import tf
from geometry_msgs.msg import Twist, Point, Pose, Quaternion
from nav_msgs.msg import Odometry

angle_condition = False
final_condition = False

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

def odom_callback(odom: Odometry):

    global angle_condition, final_condition

    cmd = Twist()
    x_rad, y_rad, z_rad = euler_from_quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
    z_degree = math.degrees(z_rad)
    
    xc = odom.pose.pose.position.x
    yc = odom.pose.pose.position.y
    
    print("current Θ =", z_degree)
    print("current x =", xc)
    print("current y =", yc)

    xt = 0.5
    yt = 0.45

    if angle_condition == False:
        angle = math.degrees(math.atan((yc - yt)/(xc - xt)))
    else:
        angle = 0

    print("alignment angle =", angle)
    
    if (yc - yt) > 0.01 and final_condition == False:
        print("Case 1: Y too high.")
        if (z_degree - angle) > 0.5:
            print("Too left. Turning right.")
            cmd.linear.x = 0.0
            cmd.angular.z = -0.2

        elif (z_degree - angle) < -0.5:
            print("Too right. Turning left.")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.2

        elif (z_degree - angle) > -0.5 and (z_degree - angle) < 0.5 and (xc - xt) > 0.01:
            print("Moving towards target.")
            cmd.angular.z = 0.0
            cmd.linear.x = -0.1

        elif (xc - xt) < 0.01 and z_degree > -0.5:
            print("Final orientation.")
            angle_condition = True
            cmd.linear.x = 0.0
            cmd.angular.z = 0.05

        else:
            print("Reached target.")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            final_condition = True
            
            
    elif (yc - yt) < -0.01 and final_condition == False:
        print("Case 2: Y too low.")
        if (z_degree - angle) > 0.5:
            print("Too left. Turning right.")
            cmd.linear.x = 0.0
            cmd.angular.z = -0.2

        elif (z_degree - angle) < -0.5:
            print("Too right. Turning left.")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.2

        elif (z_degree - angle) > -0.5 and (z_degree - angle) < 0.5 and (xc - xt) > 0.01:
            print("Moving towards target.")
            cmd.angular.z = 0.0
            cmd.linear.x = -0.1

        elif (xc - xt) < 0.01 and z_degree < 0.5:
            print("Final orientation.")
            angle_condition = True
            cmd.linear.x = 0.0
            cmd.angular.z = -0.05

        else:
            print("Reached target.")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            final_condition = True
            

    elif (yc - yt) < 0.01 and (yc - yt) > -0.01 and final_condition == False:
        print("Case 3: Y aligned.")
        angle_condition = True
        if (xc - xt) > 0.001 and (z_degree - angle) > 0.5:
            print("Too left. Turning right.")
            cmd.linear.x = 0.0
            cmd.angular.z = -0.2

        elif (xc - xt) > 0.01 and (z_degree - angle) < -0.5:
            print("Too right. Turning left.")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.2

        elif (z_degree - angle) > -0.5 and (z_degree - angle) < 0.5 and (xc - xt) > 0.01:
            print("Moving towards target.")
            cmd.angular.z = 0.0
            cmd.linear.x = -0.1

        else:
            print("Reached target.")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            final_condition = True


    elif (yc - yt) < 0.05 and (yc - yt) > -0.05 and (xc - xt) < 0.05 and (xc - xt) > -0.05:
        print("Case 4: X and Y both aligned.")
        angle_condition = True
        if (z_degree - angle) > 0.5:
            print("Too left. Turning right.")
            cmd.linear.x = 0.0
            cmd.angular.z = -0.2
        
        elif (z_degree - angle) < -0.5:
            print("Too right. Turning left.")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.2

        else:
            print("Reached target.")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            final_condition = True


    else:
        print("Out of defined coordinates.")
        
    print("-------------------------------")

    pub.publish(cmd)

if __name__ == '__main__':
    rospy.init_node("simulation_localization")
    pub = rospy.Publisher("/turtlebot1/cmd_vel", Twist, queue_size=10)
    sub = rospy.Subscriber("/turtlebot1/odom", Odometry,
                           callback=odom_callback)
    rospy.loginfo("Node has been started.")
    rate = rospy.Rate(50)
    rospy.spin()

