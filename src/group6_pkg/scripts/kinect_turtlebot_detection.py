from cv_bridge import CvBridge
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
bridge = CvBridge()

methods = [cv2.TM_CCOEFF, cv2.TM_CCOEFF_NORMED, cv2.TM_CCORR,
            cv2.TM_CCORR_NORMED, cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]

def callback(data):
    rospy.loginfo('I heard data')
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    template = cv2.imread('turtlebot.png', 0)
    h, w = template.shape

    # for method in methods:
    #     img2 = resize_image.copy()
    #     result = cv2.matchTemplate(img2, template, method)
    #     min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
    #     if method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
    #         location = min_loc
    #     else:
    #         location = max_loc

    #     bottom_right = (location[0] + w, location[1] + h)    
    #     cv2.rectangle(img2, location, bottom_right, 255, 5)

    #######

    img2 = gray_image.copy()
    result = cv2.matchTemplate(img2, template, cv2.TM_CCOEFF_NORMED)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
    print(max_val)
    location = max_loc
    print(location)
    print(cv_image.shape)
    bottom_right = (location[0] + w, location[1] + h)    
    cv2.rectangle(img2, location, bottom_right, 255, 5)

    #######

    #cv2.imshow('graycsale image',cv_image)
    cv2.imshow('Match', cv_image)
    cv2.waitKey(1)
    # print(cv2.__version__)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/kinect1/rgb_camera/image_raw", Image, callback)
    rospy.spin()

if __name__=='__main__':
    listener()