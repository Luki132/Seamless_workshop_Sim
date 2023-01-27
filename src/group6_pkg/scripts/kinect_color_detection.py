from cv_bridge import CvBridge
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
bridge = CvBridge()

x_offset = 0.24
cube_offset = 0.015
x_to_world = [None, None, None]
y_to_world = [None, None, None]
x_prev = 0
x_start = False

# Matrix to contain possible candidates
candidate_blue = np.zeros((3,6))
candidate_red = np.zeros((3,6))
candidate_green = np.zeros((3,6))


# HSV Color for color detection
blue_lower = np.array([100, 200 , 100], np.uint8)
blue_upper = np.array([125, 255 , 255], np.uint8)

red_lower = np.array([0, 200 , 100], np.uint8)
red_upper = np.array([10, 255 , 255], np.uint8)

green_lower = np.array([45, 100 , 200], np.uint8)
green_upper = np.array([75, 255 , 255], np.uint8)

def x_position(x):
    y = 1.23/720*x - 1.23/720*70
    # print(y)
    return(y + cube_offset)

def y_position(x):
    y = 0.48/280*x 
    # print(y + x_offset)
    return(y + x_offset + cube_offset)

def detect_blue_object(hsv_frame, blurred_image):
    global candidate_blue
    candidate_blue = np.zeros((3,6))
    blue_mask = cv2.inRange(hsv_frame, blue_lower, blue_upper)

    kernel = np.ones((5, 5), "uint8")

    # blue_mask = cv2.dilate(blue_mask, kernel)
    # blue_mask = cv2.erode(blue_mask, kernel)

    res_blue = cv2.bitwise_and(blurred_image, blurred_image, mask = blue_mask)

    contours, hierarchy = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # print(contours)
    for count, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        # print(area)        
        # if(area > 300 and area < 1000):
        x, y, w, h = cv2.boundingRect(contour)
            # cv_image = cv2.rectangle(cv_image, (x,y), (x+w, y+h), (255, 0 , 0), 2)
            # x_to_world = str(x_position(x))
            # y_to_world = str(y_position(y))
            # print(str(round((x_position(x)),3)))
            # print(str(round((y_position(y)),3))) 
            # x_to_world = str(round((y_position(y)),3))
            # y_to_world = str(round((x_position(x)),3))    
            # cv2.putText(cv_image, "Blue Colour (" + x_to_world + "," + y_to_world + ")", (10, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0))
            # cv2.circle(cv_image, (650,140), 5, 10)
            # print(w)
            # print(h)
        candidate_blue[count] = [x, y, w, h, area, 0]
    # print(candidate_blue)

def detect_red_object(hsv_frame, blurred_image):
    global candidate_red
    candidate_red = np.zeros((3,6))
    red_mask = cv2.inRange(hsv_frame, red_lower, red_upper)

    contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # print(contours)
    for count, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        print("Area:",area)    
        print("Count", count)    
        if(area > 300 and area < 1000):
            x, y, w, h = cv2.boundingRect(contour)
            candidate_red[count] = [x, y, w, h, area, 1]


def detect_green_object(hsv_frame, blurred_image):
    global candidate_green
    candidate_green = np.zeros((3,6))
    green_mask = cv2.inRange(hsv_frame, green_lower, green_upper)

    contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # print(contours)
    for count, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        print(area)    
        print("count_green", count)    
        if(area > 300 and area < 1000):
            x, y, w, h = cv2.boundingRect(contour)
            candidate_green[count] = [x, y, w, h, area, 2]

def callback(data):
    overall_candidate = np.zeros((9,6))
    rospy.loginfo('I heard data')
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    blurredFrame = cv2.GaussianBlur(cv_image, (11,11), 0)

    hsvFrame = cv2.cvtColor(blurredFrame, cv2.COLOR_BGR2HSV)
    
    detect_blue_object(hsvFrame, blurredFrame)
    detect_red_object(hsvFrame, blurredFrame)
    detect_green_object(hsvFrame, blurredFrame)

    #Check if the candidates are valid candidates
    for i in range(3):
        if(candidate_blue[i][4] > 300 and candidate_blue[i][4] < 1000):
            overall_candidate[i] = candidate_blue[i] 
    for i in range(3):
        if(candidate_red[i][4] > 300 and candidate_red[i][4] < 1000):
            overall_candidate[i+3] = candidate_red[i] 
    for i in range(3):
        if(candidate_green[i][4] > 300 and candidate_green[i][4] < 1000):
            overall_candidate[i+6] = candidate_green[i] 
    print(overall_candidate)
    overall_candidate = overall_candidate[overall_candidate[:,4].argsort()]
    print(overall_candidate)    

    for i in range(3):
        x_to_world[i] = str(round(y_position(float(overall_candidate[6+i][1])), 3))
        y_to_world[i] = str(round(x_position(float(overall_candidate[6+i][0])), 3))
    
    for i in range(3):
        x = int(overall_candidate[6+i][0])
        y = int(overall_candidate[6+i][1])
        w = int(overall_candidate[6+i][2])
        h = int(overall_candidate[6+i][3])
        if(overall_candidate[6+i][5] == 0):
            colour = (255,0,0)
            colour_str = "blue"
        elif(overall_candidate[6+i][5]== 1):
            colour = (0, 0, 255)
            colour_str = "red"
        else:
            colour = (0, 255, 0)
            colour_str = "green"
        cv_image = cv2.rectangle(cv_image, (x,y), (x+w, y+h), colour, 2)
        cv2.putText(cv_image, colour_str + "(" + x_to_world[i] + "," + y_to_world[i] + ")", (10, 250 - 20*i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour)

    if(overall_candidate[8][1] > overall_candidate[7][1]):
        print(overall_candidate[8][1])
        print("bigger object is below")
    else:
        print("bigger object is above")

    # for i in range(9):


    #cv2.imshow('graycsale image',cv_image)
    cv2.imshow('Match', cv_image)

    print(cv_image.shape) # [0] = 280, [1]=720
    cv2.waitKey(1)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/kinect1/rgb_camera/image_raw", Image, callback)
    rospy.spin()

if __name__=='__main__':
    listener()

