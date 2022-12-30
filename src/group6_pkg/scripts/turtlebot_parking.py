#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import Twist, Point, Pose, Quaternion, PoseStamped
from nav_msgs.msg import Odometry

# max linear speed is 0.22 m/s
# max angular speed is 2.84rad/sec

def stop():
    cmd = Twist()
    rospy.loginfo("STOP.")
    cmd.linear.x = 0.0
    cmd.angular.z = 0.0  


def charuco_detector_callback(park: PoseStamped):

    cmd = Twist()

    if park.pose.position.x < 0.01:
        rospy.loginfo("too far to right")
        cmd.linear.x = -0.05
        cmd.angular.z = 0.3      

    


    if park.pose.position.x > 0.04:
        rospy.loginfo("too far to left")
        cmd.linear.x = -0.05
        cmd.angular.z = -0.3
    
    

    if park.pose.position.x > 0.01 and park.pose.position.x < 0.04:
        rospy.loginfo("First condition achieved.")
        cmd.linear.x = -0.15
        cmd.angular.z = 0.0

    if park.pose.position.z < 0.085:
       stop()
            
    print(park.pose.position.z)  
    pub.publish(cmd)


if __name__ == '__main__':
    rospy.init_node("turtlebot_controller")
    pub = rospy.Publisher("/turtlebot1/cmd_vel", Twist, queue_size=10)
    # sub = rospy.Subscriber("/turtlebot1/odom", Odometry,callback=odom_callback)
    sub1 = rospy.Subscriber("/turtlebot1/camera/image_charuco_pose", PoseStamped,callback=charuco_detector_callback)
    rospy.loginfo("Node has been started.")

    rospy.spin()