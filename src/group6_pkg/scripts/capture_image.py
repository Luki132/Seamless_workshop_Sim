from cv_bridge import CvBridge
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
bridge = CvBridge()

counter = 0

def callback(data):
    global counter
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    # hsvFrame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    # hsvFrame[:,2,2] = 255
    # cv_image = cv2.cvtColor(hsvFrame, cv2.COLOR_HSV2BGR)
    # blurredFrame = cv2.GaussianBlur(cv_image, (11,11), 0)

    # hsvFrame = cv2.cvtColor(blurredFrame, cv2.COLOR_BGR2HSV)

    #cv2.imshow('graycsale image',cv_image)
    cv2.imshow('Kinect Camera', cv_image)

    str_counter = str(counter)
    # print(cv_image.shape) # [0] = 280, [1]=720
    cv2.waitKey(1)
    cv2.imwrite('/home/robis/image_23012023/cv_img_'+ str_counter+ '.jpg', cv_image)
    counter = counter + 1
    print("Bingo!")

def listener():
    rospy.init_node('capture', anonymous=True)
    rospy.Subscriber("/kinect/rgb_camera/image_raw", Image, callback)
    rospy.spin()

if __name__=='__main__':
    listener()
