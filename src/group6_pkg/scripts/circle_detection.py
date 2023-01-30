from cv_bridge import CvBridge
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
bridge = CvBridge()

test_green = np.empty((1,2))
bias = 50

# HSV Color for color detection
white_lower = np.array([0, 0 , 200], np.uint8)
white_upper = np.array([255, 255 , 255], np.uint8)

blue_lower = np.array([90, 100 , 90], np.uint8)
blue_upper = np.array([110, 255 , 255], np.uint8)

red_lower1 = np.array([0, 50 , 40], np.uint8)
red_upper1 = np.array([5, 255 , 255], np.uint8)
red_lower2 = np.array([140, 50 , 40], np.uint8)
red_upper2 = np.array([180, 255 , 255], np.uint8)

green_lower = np.array([35, 80 , 40], np.uint8)
green_upper = np.array([100, 255 , 255], np.uint8)

tray_found = False
num_of_objects = 0
kernel = np.ones((5, 5), np.uint8)

if __name__=='__main__':
    while(True):
        test_green = np.empty((1,2))

        overall_candidate = np.zeros((9,6))
        # cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        cv_image = cv2.imread('/home/robis/cv_image4.jpg')
        hsv_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        gray_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        edges = cv2.Canny(cv_image, threshold1= 100, threshold2=255)

        white_mask = cv2.inRange(hsv_frame, white_lower, white_upper)
        blue_mask = cv2.inRange(hsv_frame, blue_lower, blue_upper)
        red_mask1 = cv2.inRange(hsv_frame, red_lower1, red_upper1)
        red_mask2 = cv2.inRange(hsv_frame, red_lower2, red_upper2)
        green_mask = cv2.inRange(hsv_frame, green_lower, green_upper)
        all_mask = white_mask + blue_mask + red_mask1 + red_mask2 + green_mask

        ret, out = cv2.threshold(all_mask, 190, 255, cv2.THRESH_BINARY)

        # detect circles in the image
        circles = cv2.HoughCircles(gray_frame, cv2.HOUGH_GRADIENT, 1.2, 10, param1=50,param2=30,minRadius=1)
        # ensure at least some circles were found
        if circles is not None:
	        # convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")
	        # loop over the (x, y) coordinates and radius of the circles
            for (x, y, r) in circles:
                # draw the circle in the output image, then draw a rectangle
                # corresponding to the center of the circle
                cv2.circle(cv_image, (x, y), r, (0, 255, 0), 4)
                cv2.rectangle(cv_image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

        contours, hierarchy = cv2.findContours(out, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(cv_image, contours,-1,(0,0,255),1)

        for count, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            # print(count, area)
            if(area > 4000 and area < 15000):
                print("Bingo", area)
                x, y, w, h = cv2.boundingRect(contour)
                if x < 850:
                    info_green = np.array([[x, y, w, h, area, 2]])
                    rect = cv2.minAreaRect(contour)
                    box = cv2.boxPoints(rect)

                    rectangle = np.int0(box)
                    cv2.drawContours(cv_image,[rectangle],0,(0,0,0),2)
                    test_green = np.append(test_green, box, axis = 0)
                    tray_found = True
        test_green = np.delete(test_green, 0, 0)
        # print(test_green
        

        cv2.imshow('Kinect Camera', cv_image)

        cv2.waitKey(1)