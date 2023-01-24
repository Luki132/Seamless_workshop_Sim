#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def navigate():
    rospy.init_node('blind_navigation', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtlebot1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    # Set linear velocity in the x-axis
    print("Linear 1")
    vel_msg.linear.x = 0.1
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0

    # Set angular velocity in the z-axis
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    # Set the time to run the robot
    t0 = rospy.Time.now().to_sec()
    current_time = 0

    while(current_time < 2.5):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_time = t1-t0

    # rotate the turtlebot
    print("Angular 1")
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0.5
    t0 = rospy.Time.now().to_sec()
    current_time = 0

    while(current_time < (3.14)):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_time = t1-t0

    # move the turtlebot
    print("Linear 2")
    vel_msg.linear.x = 0.1
    vel_msg.angular.z = 0
    t0 = rospy.Time.now().to_sec()
    current_time = 0

    while(current_time < 4):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_time = t1-t0

    # rotate the turtlebot
    print("Angular 2")
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0.5
    t0 = rospy.Time.now().to_sec()
    current_time = 0

    while(current_time < (3.14)):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_time = t1-t0

    # move the turtlebot
    print("Linear 3")
    vel_msg.linear.x = -0.1
    vel_msg.angular.z = 0
    t0 = rospy.Time.now().to_sec()
    current_time = 0

    while(current_time < 5.7):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_time = t1-t0

    # stop the turtlebot for unloading
    print("UNLOAD")
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.0
    t0 = rospy.Time.now().to_sec()
    current_time = 0

    while(current_time < 5):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_time = t1-t0

    # move the turtlebot
    print("Linear 4")
    vel_msg.linear.x = 0.1
    vel_msg.angular.z = 0
    t0 = rospy.Time.now().to_sec()
    current_time = 0

    while(current_time < 5.75):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_time = t1-t0

    # rotate the turtlebot
    print("Angular 3")
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0.5
    t0 = rospy.Time.now().to_sec()
    current_time = 0

    while(current_time < (3.14)):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_time = t1-t0

    # move the turtlebot
    print("Linear 5")
    vel_msg.linear.x = 0.1
    vel_msg.angular.z = 0
    t0 = rospy.Time.now().to_sec()
    current_time = 0

    while(current_time < 3.9):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_time = t1-t0

    # rotate the turtlebot
    print("Angular 4")
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0.5
    t0 = rospy.Time.now().to_sec()
    current_time = 0

    while(current_time < (3.14)+0.1):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_time = t1-t0

    # stop the turtlebot
    print("STOP")
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.0
    t0 = rospy.Time.now().to_sec()
    current_time = 0

    while(current_time < 2):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_time = t1-t0

if __name__ == '__main__':
    try:
        navigate()
    except rospy.ROSInterruptException:
        pass

