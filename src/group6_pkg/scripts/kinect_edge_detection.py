from cv_bridge import CvBridge
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
bridge = CvBridge()

test_green = np.empty((1,2))



def callback(data):
    global test_green
    overall_candidate = np.zeros((9,6))
    # rospy.loginfo('I heard data')
    num_of_objects = 0
    kernel = np.ones((5, 5), np.uint8)
    # cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    cv_image = cv2.imread('/home/robis/cv_image4.jpg')
    blurredFrame = cv2.GaussianBlur(cv_image, (11,11), 0)
    grayFrame = cv2.cvtColor(blurredFrame, cv2.COLOR_BGR2GRAY)

    edges = cv2.Canny(grayFrame, 100, 200)

    ret, thresh = cv2.threshold(grayFrame, 100, 255, cv2.THRESH_BINARY)
    erodeFrame = cv2.erode(thresh, kernel, iterations=1)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for count, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        print(count, area)
        if(area > 20000 and area < 40000):
            # print("Bingo")
            x, y, w, h = cv2.boundingRect(contour)
            info_green = np.array([[x, y, w, h, area, 2]])
            # print("Hello", candidate_green)
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            # print(np.column_stack((box, np.array([[1],[0],[0],[0]]))))
            # if box[0]
            print(box)
            rectangle = np.int0(box)
            cv2.drawContours(cv_image,[rectangle],0,(0,0,0),2)
            test_green = np.append(test_green, box, axis = 0)
    test_green = np.delete(test_green, 0, 0)
    # print(test_green)

    cv2.circle(cv_image, (850, 280), 10, 10)
    cv2.circle(cv_image, (500, 62), 10, 10)
    cv2.circle(cv_image, (600, 205), 10, 10)


    

    cv2.imshow('Kinect Camera', cv_image)
    cv2.imshow('Binary', thresh)
    cv2.imshow('Erode', erodeFrame)


    print("Bingo")
    print(edges.shape)


    print(cv_image.shape) # [0] = 280, [1]=720
    cv2.waitKey(1)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/kinect/rgb_camera/image_raw", Image, callback)
    rospy.spin()

if __name__=='__main__':
    listener()
