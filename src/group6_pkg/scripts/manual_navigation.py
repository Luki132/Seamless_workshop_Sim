#!/usr/bin/env python3
import math
import rospy
import tf
from geometry_msgs.msg import Twist, Point, Pose, Quaternion
from nav_msgs.msg import Odometry

C1 = False
C2 = False
C3 = False
C4 = False
C5 = False
C6 = False
C7 = False
C8 = False
C9 = False
C10 = False
C11 = False
C12 = False
C13 = False
C14 = False
C15 = False
C16 = False
C17 = False
C18 = False

forward_speed = -0.1

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
    global C1
    global C2
    global C3
    global C4
    global C5
    global C6
    global C7
    global C8
    global C9
    global C10
    global C11
    global C12
    global C13
    global C14
    global C15
    global C16
    global C17
    global C18
    global forward_speed

    cmd = Twist()
    x_rad, y_rad, z_rad = euler_from_quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
    z_degree = math.degrees(z_rad)
    print(z_degree)
    
    if odom.pose.pose.position.x > 1.15 and odom.pose.pose.position.x < 1.3 and C1 == False:
        rospy.loginfo("START.")
        cmd.linear.x = 0.1
        cmd.angular.z = 0.0

    elif odom.pose.pose.position.x > 1.0 and odom.pose.pose.position.x < 1.16 and z_degree < -90 and C2 == False:
        rospy.loginfo("First condition achieved.")
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5
        C1 = True

    elif z_degree > -90 and C1 == True and C3 == False:
        rospy.loginfo("Second condition achieved.")
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        C2 = True
        C3 = True
        #rospy.sleep(0.5)

    elif odom.pose.pose.position.y > 0.45 and odom.pose.pose.position.y < 0.8 and C3 == True and C4 == False:
        rospy.loginfo("Third condition achieved.")
        cmd.linear.x = 0.1
        cmd.angular.z = 0.0

    elif odom.pose.pose.position.y > 0.4 and z_degree < 0 and C5 == False:
        rospy.loginfo("Fourth condition achieved.")
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5
        C4 = True

    elif z_degree > 0 and C4 == True and C6 == False:
        rospy.loginfo("Fifth condition achieved.")
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        C5 = True
        C6 = True
        #rospy.sleep(0.5)

    elif odom.pose.pose.position.x > 0.51 and C6 == True and C7 == False:
        rospy.loginfo("Sixth condition achieved.")
        if odom.pose.pose.position.x > 0.7 and odom.pose.pose.position.x < 1.15 and forward_speed > -0.18:
            forward_speed = forward_speed -0.005
            print('current speed', forward_speed)
        elif odom.pose.pose.position.x < 0.7:
            forward_speed = -0.1
        cmd.linear.x = forward_speed        
        # cmd.linear.x = -0.1
        cmd.angular.z = 0.0

    elif odom.pose.pose.position.x < 0.51 and C7 == False:
        rospy.loginfo("Seventh condition achieved.")
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        C7 = True

    elif odom.pose.pose.position.x < 0.51 and C7 == True and C10 == False:
        rospy.loginfo("Eighth condition achieved. Waiting 10s.")
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        C10 = True
        print("X = ", odom.pose.pose.position.x)
        print("Y = ", odom.pose.pose.position.y)
        print("Z_deg = ", z_degree)
        rospy.sleep(10)

    elif odom.pose.pose.position.x < 1.15 and C10 == True and C12 == False:
        rospy.loginfo("Ninth condition achieved.")
        cmd.linear.x = 0.18
        cmd.angular.z = 0.0
        C11 = True

    elif odom.pose.pose.position.x > 1.15 and C11 == True and C12 == False:
        rospy.loginfo("Tenth condition achieved.")
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        C12 = True

    elif z_degree > -90 and C12 == True and C14 == False:
        rospy.loginfo("Eleventh condition achieved.")
        cmd.linear.x = 0.0
        cmd.angular.z = -0.3
        C13 = True

    elif z_degree < -90 and C13 == True and C15 == False:
        rospy.loginfo("Twelfth condition achieved.")
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        C14 = True
        C15 = True
        #rospy.sleep(0.5)

    elif odom.pose.pose.position.y < 0.78 and C15 == True and C17 == False:
        rospy.loginfo("Thirteenth condition achieved.")
        cmd.linear.x = -0.05
        cmd.angular.z = 0.0
        C16 = True

    elif odom.pose.pose.position.y > 0.75 and z_degree > -179 and C16 == True and C18 == False:
        rospy.loginfo("Fourteenth condition achieved.")
        cmd.linear.x = 0.0
        cmd.angular.z = -0.3
        C17 = True

    elif (z_degree < -179 or z_degree > 179) and C17 == True:
        rospy.loginfo("STOP.")
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        C18 = True
        #rospy.sleep(0.5)

    else:
        rospy.loginfo("OUT OF DEFINED COORDINATES.")
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        

    pub.publish(cmd)


if __name__ == '__main__':
    rospy.init_node("turtlebot_controller")
    pub = rospy.Publisher("/turtlebot1/cmd_vel", Twist, queue_size=10)
    sub = rospy.Subscriber("/turtlebot1/odom", Odometry,
                           callback=odom_callback)
    rospy.loginfo("Node has been started.")
    rate = rospy.Rate(50)
    rospy.spin()

