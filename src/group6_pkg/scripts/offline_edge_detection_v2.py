from cv_bridge import CvBridge
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
bridge = CvBridge()

test_green = np.empty((1,2))

if __name__=='__main__':
    while(True):
        test_green = np.empty((1,2))

        overall_candidate = np.zeros((9,6))
        # rospy.loginfo('I heard data')
        num_of_objects = 0
        kernel = np.ones((3, 3), np.uint8)
        # cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        cv_image = cv2.imread('/home/robis/cv_image.jpg')
        # blurredFrame = cv2.GaussianBlur(cv_image, (11,11), 0)
        grayFrame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        edges = cv2.Canny(cv_image, threshold1= 100, threshold2=255)


        ret, thresh = cv2.threshold(grayFrame, 220, 255, cv2.THRESH_BINARY)
        # th3 = cv2.adaptiveThreshold(grayFrame,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
        #     cv2.THRESH_BINARY,75,2)
        # dilateFrame = cv2.dilate(thresh, kernel, iterations=2)
        # erodeFrame = cv2.erode(dilateFrame, kernel, iterations=1)
        mask = np.zeros(grayFrame.shape, dtype="uint8")
        contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
        # print("Look at me", hierarchy)
        # print(hierarchy[0][0][2])
        
        # # detect circles in the image
        # circles = cv2.HoughCircles(edges,cv2.HOUGH_GRADIENT,1,20,
        #                         param1=50,param2=30,minRadius=0)
        # # ensure at least some circles were found
        # if circles is not None:
        #     print("CIRCLE")
        #     # convert the (x, y) coordinates and radius of the circles to integers
        #     circles = np.round(circles[0, :]).astype("int")
        #     # loop over the (x, y) coordinates and radius of the circles
        #     print(circles)
        #     for (x, y, r) in circles:
        # 	# draw the circle in the output image, then draw a rectangle
        # 	# corresponding to the center of the circle
        #         cv2.circle(cv_image, (x, y), r, (0, 255, 0), 4)
        #         cv2.rectangle(cv_image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
        mask = np.zeros(cv_image.shape, dtype="uint8")
        counter = 0
        for count, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            print(count, area)
            if(hierarchy[0][count][2]== -1):
                counter = counter +1
                print("No.",counter)
                cv2.fillPoly(mask, pts =contours, color=(255,255,255))
            else:
                pass
            # print(count, area)
            # if(area > 200 and area < 400000):
            #     print("Bingo")
            #     x, y, w, h = cv2.boundingRect(contour)
            #     info_green = np.array([[x, y, w, h, area, 2]])
            #     # print("Hello", candidate_green)
            #     rect = cv2.minAreaRect(contour)
            #     box = cv2.boxPoints(rect)
            #     # print(np.column_stack((box, np.array([[1],[0],[0],[0]]))))
            #     # if box[0]
            #     # print(box)
            #     rectangle = np.int0(box)
            #     cv2.drawContours(cv_image,[rectangle],0,(0,0,0),2)
            #     test_green = np.append(test_green, box, axis = 0)
        # test_green = np.delete(test_green, 0, 0)
        # print(test_green)


        

        cv2.imshow('Kinect Camera', cv_image)
        cv2.imshow('Binary', thresh)
        cv2.imshow('Canny', edges)
        cv2.imshow('Test', mask)
        cv2.imshow('gray', grayFrame)


        print("Bingo")
        print(edges.shape)


        print(cv_image.shape) # [0] = 280, [1]=720
        cv2.waitKey(1)