import rospy
import tf
from geometry_msgs.msg import Twist, Point, Pose, Quaternion
from nav_msgs.msg import Odometry

import math
first_condition = False
second_condition = False
third_condition = False
fourth_condition = False
fifth_condition = False
sixth_condition = False

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
    global third_condition
    global first_condition
    global second_condition
    global fourth_condition
    global fifth_condition
    global sixth_condition


    # quarternion = [odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,\
    #             odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
    #(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
    #self.pose.theta = yaw
    #self.pose.x = odom.pose.pose.position.x
    #self.pose.y = odom.pose.pose.position.y
    #self.theta.z = odom.pose.pose.orientation.z
    #self.theta.w = odom.pose.pose.orientation.w

    cmd = Twist()
    x_rad, y_rad, z_rad = euler_from_quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
    z_degree = math.degrees(z_rad)
    print(z_degree)
    if odom.pose.pose.position.x > 1.15 and odom.pose.pose.position.x < 1.3 and first_condition == False:
        rospy.loginfo("First condition achieved.")
        cmd.linear.x = 0.22
        cmd.angular.z = 0.0
    elif odom.pose.pose.position.x > 1.0 and odom.pose.pose.position.x < 1.16 and z_degree < -90 and second_condition == False:
        rospy.loginfo("Second condition achieved.")
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5
        first_condition = True
    elif z_degree > -90 and third_condition == False:
        rospy.loginfo("Third condition achieved.")
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        rospy.sleep(2)
        second_condition = True
        third_condition = True
        print(third_condition)
    elif odom.pose.pose.position.y > 0.5 and odom.pose.pose.position.y < 0.8 and z_degree > -90 and fourth_condition == False:
        rospy.loginfo("Fourth condition achieved.")
        cmd.linear.x = 0.1
        cmd.angular.z = 0.0
    elif odom.pose.pose.position.y > 0.4 and odom.pose.pose.position.y < 0.51 and z_degree < 0 and z_degree > -90 and fifth_condition == False:
        rospy.loginfo("Fifth condition achieved.")
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5
        fourth_condition = True
    elif odom.pose.pose.orientation.z < 0.07 and odom.pose.pose.orientation.z > -0.071 and odom.pose.pose.orientation.w > 0.99 and odom.pose.pose.position.x > 0.5 and sixth_condition == False:
        rospy.loginfo("Sixth condition achieved.")
        cmd.linear.x = -0.22
        cmd.angular.z = 0.0
        fifth_condition = True
    else:
        rospy.loginfo("STOP.")
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        sixth_condition = True

    pub.publish(cmd)


if __name__ == '__main__':
    rospy.init_node("turtlebot_controller")
    pub = rospy.Publisher("/turtlebot1/cmd_vel", Twist, queue_size=10)
    sub = rospy.Subscriber("/turtlebot1/odom", Odometry,
                           callback=odom_callback)
    rospy.loginfo("Node has been started.")
    rate = rospy.Rate(50)
    rospy.spin()

