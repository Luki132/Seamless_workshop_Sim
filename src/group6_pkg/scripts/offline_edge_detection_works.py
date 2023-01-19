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
        cv_image = cv2.imread('/home/robis/cv_image2.jpg')
        hsv_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        edges = cv2.Canny(cv_image, threshold1= 100, threshold2=255)

        white_mask = cv2.inRange(hsv_frame, white_lower, white_upper)
        blue_mask = cv2.inRange(hsv_frame, blue_lower, blue_upper)
        red_mask1 = cv2.inRange(hsv_frame, red_lower1, red_upper1)
        red_mask2 = cv2.inRange(hsv_frame, red_lower2, red_upper2)
        green_mask = cv2.inRange(hsv_frame, green_lower, green_upper)
        all_mask = white_mask + blue_mask + red_mask1 + red_mask2 + green_mask

        ret, out = cv2.threshold(all_mask, 190, 255, cv2.THRESH_BINARY)

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
                    u_pos = int((rectangle[0][0] + rectangle[1][0] + rectangle[2][0] + rectangle[3][0])/4)
                    v_pos = int((rectangle[0][1] + rectangle[1][1] + rectangle[2][1] + rectangle[3][1])/4)
                    cv2.circle(cv_image, (u_pos, v_pos), 4, 2)
                    cv2.drawContours(cv_image,[rectangle],0,(0,0,0),2)
                    test_green = np.append(test_green, box, axis = 0)
                    tray_found = True
        test_green = np.delete(test_green, 0, 0)
        # print(test_green)

        if tray_found == False:
            print("Back-up Plan")
            test_green = np.empty((1,2))

            overall_candidate = np.zeros((9,6))
            # cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image = cv2.imread('/home/robis/cv_image2jpg')
            hsv_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            edges = cv2.Canny(cv_image, threshold1= 100, threshold2=255)
            edges = cv2.dilate(edges, kernel)
            grayFrame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            ret, thresh = cv2.threshold(grayFrame, 180, 255, cv2.THRESH_BINARY)
            white_mask = cv2.inRange(hsv_frame, white_lower, white_upper)
            blue_mask = cv2.inRange(hsv_frame, blue_lower, blue_upper)
            red_mask1 = cv2.inRange(hsv_frame, red_lower1, red_upper1)
            red_mask2 = cv2.inRange(hsv_frame, red_lower2, red_upper2)
            green_mask = cv2.inRange(hsv_frame, green_lower, green_upper)
            all_mask = cv2.bitwise_or(white_mask, red_mask1)
            all_mask = cv2.bitwise_or(all_mask, red_mask2)
            all_mask = cv2.bitwise_or(all_mask, blue_mask)
            all_mask = cv2.bitwise_or(all_mask, green_mask)


            
            # white_mask + blue_mask + red_mask1 + red_mask2 + green_mask
            # all_mask = cv2.bitwise_or(thresh, all_mask)
            all_mask = cv2.bitwise_not(all_mask)
            out2 = cv2.bitwise_or(all_mask, edges)
            out = cv2.bitwise_not(out2)

            # ret, out = cv2.threshold(out, 190, 255, cv2.THRESH_BINARY)

            contours, hierarchy = cv2.findContours(out, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
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
                        u_pos = int((rectangle[0][0] + rectangle[1][0] + rectangle[2][0] + rectangle[3][0])/4)
                        v_pos = int((rectangle[0][1] + rectangle[1][1] + rectangle[2][1] + rectangle[3][1])/4)
                        cv2.circle(cv_image, (u_pos, v_pos), 4, 2)

                        cv2.drawContours(cv_image,[rectangle],0,(0,0,0),2)
                        test_green = np.append(test_green, box, axis = 0)
                        tray_found = True
            test_green = np.delete(test_green, 0, 0)
        
        print(test_green)

        

        cv2.imshow('Kinect Camera', cv_image)
        # cv2.imshow('Binary', out2)
        cv2.imshow('Canny', edges)
        cv2.imshow('HSV', all_mask)
        cv2.imshow('Output', out)


        # print("Bingo")
        # print(edges.shape)

        tray_found = False

        # print(cv_image.shape) # [0] = 280, [1]=720
        cv2.waitKey(1)