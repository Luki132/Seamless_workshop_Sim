from cv_bridge import CvBridge
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
bridge = CvBridge()

camera_info = CameraInfo()

def callback():
    pass

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/kinect1/rgb_camera/camera_info", Image, callback)
    rospy.spin()

if __name__=='__main__':
    listener()