from cv_bridge import CvBridge
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
bridge = CvBridge()

def callback(data):
    rospy.loginfo('I heard data')
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    blurred_image = cv2.GaussianBlur(cv_image, (11,11), 0)

    hsvFrame = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2HSV)
    blue_lower = np.array([100, 100 , 100], np.uint8)
    blue_upper = np.array([125, 255 , 255], np.uint8)
    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)

    kernel = np.ones((3, 3), "uint8")

    blue_mask = cv2.dilate(blue_mask, kernel)
    blue_mask = cv2.erode(blue_mask, kernel)

    res_blue = cv2.bitwise_and(blurred_image, blurred_image, mask = blue_mask)

    contours, hierarchy = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        print(area)        
        if(area > 300):
            x, y, w, h = cv2.boundingRect(contour)
            cv_image = cv2.rectangle(cv_image, (x,y), (x+w, y+h), (255, 0 , 0), 2)

            cv2.putText(cv_image, "Blue Colour", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0))

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


