from cv_bridge import CvBridge
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
bridge = CvBridge()


def callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    # blurredFrame = cv2.GaussianBlur(cv_image, (11,11), 0)

    # hsvFrame = cv2.cvtColor(blurredFrame, cv2.COLOR_BGR2HSV)

    #cv2.imshow('graycsale image',cv_image)
    cv2.imshow('Kinect Camera', cv_image)


    # print(cv_image.shape) # [0] = 280, [1]=720
    cv2.waitKey(1)
    cv2.imwrite('/home/robis/cv_image2.jpg', cv_image)
    print("Bingo!")

def listener():
    rospy.init_node('capture', anonymous=True)
    rospy.Subscriber("/kinect/rgb_camera/image_raw", Image, callback)
    rospy.spin()

if __name__=='__main__':
    listener()