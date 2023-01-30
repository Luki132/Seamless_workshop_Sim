from cv_bridge import CvBridge
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
bridge = CvBridge()

x_offset = 0.275
cube_offset = 0.015
x_to_world = [None, None, None, None, None, None, None, None, None]
y_to_world = [None, None, None, None, None, None, None, None, None]
x_prev = 0
x_start = False
test_green = np.empty((1,2))
test_red = np.empty((1,2))
test_blue = np.empty((1,2))

# K_matrix = np.array([[395.33, 0.0, 360.5], [0.0, 395.33, 140.5], [0.0, 0.0, 1.0]])
# Rt_matrix = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]])
# P_matrix = np.matmul(K_matrix, Rt_matrix)
# P_matrix = np.array([[359.9986776485914, 0.0, 360.5, -25.1999074354014], [0.0, 359.9986776485914, 140.5, 0.0], [0.0, 0.0, 1.0, 0.0]])
# P_matrix = np.array([[395.33, 0.0, 290.5, -180.9815], [0.0, 395.33, 140.5, -87.5315], [0.0, 0.0, 1.0, -0.623]])
# P_matrix = np.array([[360.0, 0.0, 360.5, -25.2], [0.0, 360.0, 140.5, 0], [0.0, 0.0, 1.0, -0.623]])

# Matrix to contain possible candidates
candidate_blue = np.empty((1,6))
candidate_red = np.empty((1,6))
candidate_green = np.empty((1,6))


# HSV Color for color detection
blue_lower = np.array([100, 50 , 150], np.uint8)
blue_upper = np.array([120, 114 , 255], np.uint8)

red_lower1 = np.array([0, 50 , 40], np.uint8)
red_upper1 = np.array([5, 255 , 255], np.uint8)
red_lower2 = np.array([140, 50 , 40], np.uint8)
red_upper2 = np.array([180, 255 , 255], np.uint8)

green_lower = np.array([35, 80 , 40], np.uint8)
green_upper = np.array([100, 255 , 255], np.uint8)


def detect_blue_object(hsv_frame, blurred_image):
    global candidate_blue
    global test_blue
    global blue_mask
    global erodeFrame
    candidate_blue = np.empty((1,6))
    kernel = np.ones((5, 5), np.uint8)

    # blue_mask = cv2.inRange(hsv_frame, blue_lower, blue_upper)
    # dilateFrame = cv2.dilate(blue_mask, kernel, iterations=2)
    # erodeFrame = cv2.erode(dilateFrame, kernel, iterations=1)

    edges = cv2.Canny(cv_image, threshold1= 255, threshold2=255)

    contours, hierarchy = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    test_blue = np.empty((1,2))
    for count, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        print(count, area)
        if(area > 100 and area < 4000):
            # print("Bingo")
            x, y, w, h = cv2.boundingRect(contour)
            info_blue = np.array([[x, y, w, h, area, 2]])
            # print("Hello", candidate_blue)
            candidate_blue = np.append(candidate_blue, info_blue, axis=0)
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            # print(np.column_stack((box, np.array([[1],[0],[0],[0]]))))
            test_blue = np.append(test_blue, box, axis = 0)
    test_blue = np.delete(test_blue, 0, 0)
    candidate_blue = np.delete(candidate_blue, 0, 0)



if __name__=='__main__':
    while(True):
        overall_candidate = np.zeros((9,6))
        # rospy.loginfo('I heard data')
        num_of_objects = 0
        num_of_red = 0
        cv_image = cv2.imread('/home/robis/cv_image9.jpg')
        blurredFrame = cv2.GaussianBlur(cv_image, (3,3), 0)

        hsvFrame = cv2.cvtColor(blurredFrame, cv2.COLOR_BGR2HSV)
        
        detect_blue_object(hsvFrame, blurredFrame)
        

        for i in range(int(test_blue.shape[0]/4)):
            num_of_objects = num_of_objects + 1
            rectangle = test_blue[i*4:i*4+4]
            rectangle = np.int0(rectangle)
            cv2.drawContours(cv_image,[rectangle],0,(0,0,0),2)
            x_pos = int((rectangle[0][0] + rectangle[1][0] + rectangle[2][0] + rectangle[3][0])/4)
            y_pos = int((rectangle[0][1] + rectangle[1][1] + rectangle[2][1] + rectangle[3][1])/4)
            cv2.circle(cv_image, (x_pos, y_pos), 2, 2)

        cv2.imshow('Kinect Camera', cv_image)
        cv2.imshow('Tray', erodeFrame)


        print(cv_image.shape) # [0] = 280, [1]=720
        cv2.waitKey(1)