import numpy as np
import cv2

webcam = cv2.VideoCapture(0)

while(1): 

    _, imageFrame = webcam.read()
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
    blue_lower = np.array([100, 100 , 100], np.uint8)
    blue_upper = np.array([125, 255 , 255], np.uint8)
    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)

    kernel = np.ones((3, 3), "uint8")

    blue_mask = cv2.dilate(blue_mask, kernel)
    blue_mask = cv2.erode(blue_mask, kernel)

    res_blue = cv2.bitwise_and(imageFrame, imageFrame, mask = blue_mask)

    contours, hierarchy = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        print(area)        
        if(area > 300):
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x,y), (x+w, y+h), (255, 0 , 0), 2)

            cv2.putText(imageFrame, "Blue Colour", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0))

    cv2.imshow("Blue Colour in Real-Time", blue_mask)
    # cv2.imshow("Real-Time", imageFrame)
    cv2.waitKey(1)