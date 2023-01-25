#!/usr/bin/env python3
import math
import rospy
import tf
from geometry_msgs.msg import Twist, Point, Pose, Quaternion
from sensor_msgs.msg import LaserScan

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

def scan_callback(msg):
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

    cmd = Twist()
    
    f = msg.ranges[0]
    l = msg.ranges[90]
    b = msg.ranges[180]
    r = msg.ranges[270]

    # Check invalid lidar data
    if f == 0 or b == 0:
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

    # Set linear velocity in the x-axis: Linear 1
    #if f > 0.24 and C1 == False:
    if (b < 0.8 and f > 0.24) and C1 == False:
        rospy.loginfo("Linear 1")
        cmd.linear.x = 0.1
        cmd.angular.z = 0.0

    #elif f < 0.24 and C2 == False:
    elif (b > 0.8 or f < 0.24) and C2 == False:
        rospy.loginfo("Stop Linear 1")
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        C1 = True
        C2 = True

    # rotate the turtlebot: Angular 1
    elif C2 == True and C3 == False:
        print("Angular 1")
        cmd.linear.x = 0
        cmd.angular.z = 0.5
        t0 = rospy.Time.now().to_sec()
        current_time = 0
        pub.publish(cmd)

        while(current_time < (3.14)):
            t1 = rospy.Time.now().to_sec()
            current_time = t1-t0
            C3 = True

    elif f < 0.73 and C3 == True and C4 == False:                # check condition f, r
        print("Too left. Turning right.")
        cmd.linear.x = 0.0
        cmd.angular.z = -0.1

    elif f > 0.8 and C3 == True and C4 == False:             # check condition f, r
        print("Too right. Turning left.")
        cmd.linear.x = 0.0
        cmd.angular.z = 0.1

    elif f > 0.73 and f < 0.8 and C3 == True and C4 == False:        # check condition f, r
        print("Oriented correct.")
        cmd.angular.z = 0.0
        cmd.linear.x = 0.0
        C4 = True

    # Set linear velocity in the x-axis: Linear 2
    #elif f > 0.45 and C4 == True and C5 == False:
    elif b < 0.54 and C4 == True and C5 == False:
        rospy.loginfo("Linear 2")
        cmd.linear.x = 0.1
        cmd.angular.z = 0.0

    #elif f < 0.45 and C4 == True and C5 == False:
    elif b > 0.54 and C4 == True and C5 == False:
        rospy.loginfo("Stop Linear 2")
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        C5 = True

    # rotate the turtlebot: Angular 2
    elif C5 == True and C6 == False:
        print("Angular 2")
        cmd.linear.x = 0
        cmd.angular.z = 0.5
        t0 = rospy.Time.now().to_sec()
        current_time = 0
        pub.publish(cmd)

        while(current_time < (3.14)):
            t1 = rospy.Time.now().to_sec()
            current_time = t1-t0
            C6 = True

    elif b > 1.05 and C6 == True and C7 == False:                # check condition f, r
        print("Too left. Turning right.")
        cmd.linear.x = 0.0
        cmd.angular.z = -0.1

    elif b < 0.955 and C6 == True and C7 == False:             # check condition f, r
        print("Too right. Turning left.")
        cmd.linear.x = 0.0
        cmd.angular.z = 0.1

    elif b > 0.955 and b < 1.05 and C6 == True and C7 == False:        # check condition f, r
        print("Oriented correct.")
        cmd.angular.z = 0.0
        cmd.linear.x = 0.0
        C7 = True

    # Set linear velocity in the x-axis: Linear 3
    #elif f < 1.45 and C7 == True and C8 == False:
    elif (b > 0.55 and f < 1.45) and C7 == True and C8 == False:
        rospy.loginfo("Linear 3")
        cmd.linear.x = -0.1
        cmd.angular.z = 0.0

    #elif f > 1.45 and C7 == True and C8 == False:
    elif (b < 0.55 or f > 1.45) and C7 == True and C8 == False:
        rospy.loginfo("Stop Linear 3")
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        C8 = True

    elif C8 == True and C9 == False:
        print("UNLOAD")
        rospy.sleep(5)
        C9 = True

    # Set linear velocity in the x-axis: Linear 4
    #elif f > 0.95 and C9 == True and C10 == False:
    elif b < 0.95 and C9 == True and C10 == False:
        rospy.loginfo("Linear 4")
        cmd.linear.x = 0.1
        cmd.angular.z = 0.0

    #elif f < 0.95 and C9 == True and C10 == False:
    elif b > 0.95 and C9 == True and C10 == False:
        rospy.loginfo("Stop Linear 4")
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        C10 = True
       
    # rotate the turtlebot: Angular 3
    elif C10 == True and C11 == False:
        print("Angular 3")
        cmd.linear.x = 0
        cmd.angular.z = 0.5
        t0 = rospy.Time.now().to_sec()
        current_time = 0
        pub.publish(cmd)

        while(current_time < (3.14)):
            t1 = rospy.Time.now().to_sec()
            current_time = t1-t0
            C11 = True

    elif f > 0.7 and C11 == True and C12 == False:                # check condition f, r
        print("Too left. Turning right.")
        cmd.linear.x = 0.0
        cmd.angular.z = -0.1

    elif f < 0.6 and C11 == True and C12 == False:             # check condition f, r
        print("Too right. Turning left.")
        cmd.linear.x = 0.0
        cmd.angular.z = 0.1

    elif f < 0.7 and f > 0.6 and C11 == True and C12 == False:        # check condition f, r
        print("Oriented correct.")
        cmd.angular.z = 0.0
        cmd.linear.x = 0.0
        C12 = True

    # Set linear velocity in the x-axis: Linear 5
    #elif f > 0.275 and C12 == True and C13 == False:
    elif b < 0.68 and C12 == True and C13 == False:
        rospy.loginfo("Linear 5")
        cmd.linear.x = 0.1
        cmd.angular.z = 0.0

    #elif f < 0.275 and C12 == True and C13 == False:
    elif b > 0.68 and C12 == True and C13 == False:
        rospy.loginfo("Stop Linear 5")
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        C13 = True
       
    # rotate the turtlebot: Angular 4
    elif C13 == True and C14 == False:
        print("Angular 4")
        cmd.linear.x = 0
        cmd.angular.z = 0.5
        t0 = rospy.Time.now().to_sec()
        current_time = 0
        pub.publish(cmd)

        while(current_time < (3.14)):
            t1 = rospy.Time.now().to_sec()
            current_time = t1-t0
            C14 = True

    elif f > 0.295 and C14 == True and C15 == False:                # check condition f, r
        print("Too left. Turning right.")
        cmd.linear.x = 0.0
        cmd.angular.z = -0.1

    elif f < 0.275 and C14 == True and C15 == False:             # check condition f, r
        print("Too right. Turning left.")
        cmd.linear.x = 0.0
        cmd.angular.z = 0.1

    elif f < 0.295 and f > 0.275 and C14 == True and C15 == False:        # check condition f, r
        print("Oriented correct.")
        cmd.angular.z = 0.0
        cmd.linear.x = 0.0
        C15 = True

    # Set linear velocity in the x-axis: Linear 6
    #elif f < 0.39 and C15 == True and C16 == False:
    elif (b > 0.71 or f < 0.39) and C15 == True and C16 == False:
        rospy.loginfo("Linear 6")
        cmd.linear.x = -0.1
        cmd.angular.z = 0.0

    #elif f > 0.39 and C15 == True and C16 == False:
    elif (b < 0.71 or f > 0.39) and C15 == True and C16 == False:
        rospy.loginfo("Stop Linear 6")
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        C16 = True

    elif C16 == True:
        print("Nearly Parked!")

    else:
        rospy.loginfo("OUT OF DEFINED COORDINATES.")
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        

    pub.publish(cmd)


if __name__ == '__main__':
    rospy.init_node("laser_navigation")
    pub = rospy.Publisher("/turtlebot1/cmd_vel", Twist, queue_size=10)
    sub = rospy.Subscriber('/turtlebot1/scan', LaserScan, scan_callback)
    rospy.loginfo("Node has been started.")
    rospy.spin()

