from cv_bridge import CvBridge
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
import math   
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import Pose, PoseArray, Point 
bridge = CvBridge()

x_offset = 0.53 # Test Area A
y_offset = 0.48 # Test Area A 
z_diff = 0.6104 - 0.158

counter = 0
test_green = np.empty((1,2))

x_to_world_float = [None, None, None]
y_to_world_float = [None, None, None]

# HSV Color for color detection
white_lower = np.array([0, 0 , 210], np.uint8)
white_upper = np.array([255, 255 , 255], np.uint8)

black_lower = np.array([90, 0 , 0], np.uint8)
black_upper = np.array([120, 255 , 140], np.uint8)

blue_lower = np.array([90, 100 , 90], np.uint8)
blue_upper = np.array([110, 255 , 255], np.uint8)

red_lower1 = np.array([0, 100 , 100], np.uint8)
red_upper1 = np.array([5, 255 , 255], np.uint8)
red_lower2 = np.array([130, 50 , 100], np.uint8)
red_upper2 = np.array([180, 255 , 255], np.uint8)

green_lower = np.array([35, 80 , 40], np.uint8)
green_upper = np.array([100, 255 , 255], np.uint8)

num_of_objects = 0
kernel = np.ones((3, 3), np.uint8)

turtlebot_found = False
tray_found = True
turtlebot_pos = np.array([[0], [0]])
turtlebot_pos_world = np.array([[0], [0]])

tray_pos = np.array([[0], [0]])
second_method = False
test_green = np.empty((1,2))
angle = 0.0
turtlebot_ang = None
# def find_orientation(arr1: np.array, arr2: np.array):
#     """ arr1: tray, arr2: lidar"""
#     global angle
#     x = abs(arr1[0] - arr2[0])
#     y = abs(arr1[1] - arr2[1])
#     print("x",x)
#     print("y",y)
#     if arr1[1] < arr2[1]:
#         if arr1[0] > arr2[0]:
#             angle = 90 - math.degrees(math.atan(y/x))
#         else:
#             angle = (90 - math.degrees(math.atan(y/x)))*(-1)
#     else:
#         if arr1[0] > arr2[0]:
#             angle = 180 - math.degrees(math.atan(x/y))
#         else:
#             angle = (180 - math.degrees(math.atan(x/y)))*(-1)
#     print(angle)
#     return angle

def find_orientation(arr1: np.array, arr2: np.array, ang):
    """ arr1: tray, arr2: lidar"""
    global angle
    ang = ang*(-1)
    x = abs(arr1[0] - arr2[0])
    y = abs(arr1[1] - arr2[1])
    print("x",x)
    # print("y",y)
    print(arr1[1])
    print(arr2[1])

    if arr1[1] < arr2[1]: ## Tray is on top of turtlebot
        if arr1[0] > arr2[0] or ang > 88: ## Tray: Right, TurtleBot: Left
            print("Done")
            angle = 90 - ang
        else:
            print("Done done")
            angle = ang*(-1)
    else:
        if arr1[0] > arr2[0] or ang < 3:
            angle = 180 - ang
        else:
            angle = (90 + ang)*(-1)
    print(ang)
    print("Something happen",angle)
    return angle

def circle_detector(img):
    global turtlebot_found,turtlebot_pos
    # detect circles in the image
    circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1.2, 100, minRadius = 20, maxRadius = 70)
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
            # print(r)
            turtlebot_pos = np.array([[x], [y]])
            turtlebot_found = True
            # print("found")

def turtlebot_detector(img1,img2):
    global turtlebot_found,turtlebot_pos, turtlebot_pos_world
    counter = 0
    img1=  cv2.bitwise_not(img1)
    img1 = cv2.bitwise_or(img1, img2)
    img1 = cv2.bitwise_not(img1)
    img1 = cv2.morphologyEx(img1, cv2.MORPH_CLOSE, kernel)

    contours, hierarchy = cv2.findContours(img1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(cv_image, contours,-1,(0,0,255),1)

    for count, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        # print(count, area)
        if(area > 10000):
            # print("Bingo", area)
            x, y, w, h = cv2.boundingRect(contour)
            if x < 850 and x > 150 and y > 100 and w < 250 and h < 250:
            # if x > 0:
                print("Area:",area)
                print("width:", w)
                info_green = np.array([[x, y, w, h, area, 2]])
                rect = cv2.minAreaRect(contour)
                angle = rect[-1]
                angle = (90 + angle) % 180 - 90
                # print("Rotation:", angle)
                box = cv2.boxPoints(rect)

                rectangle = np.int0(box)
                u_pos = int((rectangle[0][0] + rectangle[1][0] + rectangle[2][0] + rectangle[3][0])/4)
                v_pos = int((rectangle[0][1] + rectangle[1][1] + rectangle[2][1] + rectangle[3][1])/4)
                turtlebot_pos = np.array([[u_pos], [v_pos]])
                y_old, x_old, z_old = camera.projectPixelTo3dRay((u_pos,v_pos)) ## Keep in mind that the pixel here is flip
                x_to_world_float[counter] = round(z_diff/z_old*x_old + x_offset, 3)
                y_to_world_float[counter] = round(z_diff/z_old*y_old + y_offset, 3)
                turtlebot_pos_world = np.array([[x_to_world_float[counter]], [y_to_world_float[counter]]])
                turtlebot_found = True
                cv2.circle(cv_image, (u_pos, v_pos), 4, 2)
                cv2.drawContours(cv_image,[rectangle],0,(0,0,0),2)
                counter = counter + 1

    return img1

def callback(data):    
    global tray_found, turtlebot_found,test_green, tray_pos, turtlebot_pos, cv_image, edges, all_mask, out, second_method,counter, angle
    global coordinate_publisher, black_mask, turtlebot_ang
    angle = 0.0
    coordinates = PoseArray()

    test_green = np.empty((1,2))

    overall_candidate = np.zeros((9,6))

    if second_method == False:
        second_method = True
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        # cv_image = cv2.imread('/home/robis/image_23012023/cv_img_'+ str(counter) + '.jpg')
        # cv_image = cv2.imread('/home/robis/cv_image9.jpg')
        # cv_image = cv2.imread('/home/robis/image_23012023/cv_img_106.jpg')


        if counter < 280:
            counter = counter + 1
        else:
            counter = 0
        hsv_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        grayFrame = cv2.normalize(cv_image,None, 150, 255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)

        grayFrame = cv2.cvtColor(grayFrame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(cv_image, threshold1= 100, threshold2=255)

        white_mask = cv2.inRange(hsv_frame, white_lower, white_upper)
        black_mask = cv2.inRange(hsv_frame, black_lower, black_upper)
        blue_mask = cv2.inRange(hsv_frame, blue_lower, blue_upper)
        red_mask1 = cv2.inRange(hsv_frame, red_lower1, red_upper1)
        red_mask2 = cv2.inRange(hsv_frame, red_lower2, red_upper2)
        green_mask = cv2.inRange(hsv_frame, green_lower, green_upper)
        all_mask = white_mask + blue_mask + red_mask1 + red_mask2 + green_mask

        ret, out = cv2.threshold(all_mask, 190, 255, cv2.THRESH_BINARY)
        out = cv2.dilate(out, kernel)

        edges = cv2.Canny(grayFrame, threshold1= 0, threshold2=255)
        edges = cv2.dilate(edges, kernel)

        circle_detector(grayFrame)
        if turtlebot_found == False:
            black_mask = turtlebot_detector(black_mask, edges)

        contours, hierarchy = cv2.findContours(out, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(cv_image, contours,-1,(0,0,255),1)

        for count, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area > 8000 and area < 15000):
                x, y, w, h = cv2.boundingRect(contour)
                if x < 740 and x > 0 and w < 170 and h < 170 and y < 600:
                # if x > 0:
                    print("Bingo1", area)
                    info_green = np.array([[x, y, w, h, area, 2]])
                    rect = cv2.minAreaRect(contour)
                    angle = rect[-1]
                    print("Rotation:", angle)
                    box = cv2.boxPoints(rect)

                    rectangle = np.int0(box)
                    u_pos = int((rectangle[0][0] + rectangle[1][0] + rectangle[2][0] + rectangle[3][0])/4)
                    v_pos = int((rectangle[0][1] + rectangle[1][1] + rectangle[2][1] + rectangle[3][1])/4)
                    cv2.circle(cv_image, (u_pos, v_pos), 4, 2)
                    cv2.drawContours(cv_image,[rectangle],0,(0,0,0),2)
                    test_green = np.append(test_green, box, axis = 0)
                    tray_found = True
                    tray_pos = np.array([[u_pos],[v_pos]])

        test_green = np.delete(test_green, 0, 0)
        if tray_found == True and turtlebot_found == True:
            second_method = False
        else:
            second_method = True
    else:
        second_method = False
        turtlebot_found = False
        tray_found = False
        print("Back-up Plan")
        test_green = np.empty((1,2))

        overall_candidate = np.zeros((9,6))
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        # cv_image = cv2.imread('/home/robis/cv_image8.jpg')
        hsv_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        grayFrame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)


        edges = cv2.Canny(grayFrame, threshold1= 0, threshold2=255)
        # edges = cv2.dilate(edges, kernel)
        # edges = cv2.dilate(edges, kernel)

        black_mask = cv2.inRange(hsv_frame, black_lower, black_upper)

        circle_detector(grayFrame)
        if turtlebot_found == False:
            black_mask = turtlebot_detector(black_mask, edges)

        ret, thresh = cv2.threshold(grayFrame, 180, 255, cv2.THRESH_BINARY)
        white_mask = cv2.inRange(hsv_frame, white_lower, white_upper)
        blue_mask = cv2.inRange(hsv_frame, blue_lower, blue_upper)
        red_mask1 = cv2.inRange(hsv_frame, red_lower1, red_upper1)
        red_mask2 = cv2.inRange(hsv_frame, red_lower2, red_upper2)
        green_mask = cv2.inRange(hsv_frame, green_lower, green_upper)
        # all_mask = cv2.bitwise_or(white_mask, red_mask1)
        # all_mask = cv2.bitwise_or(all_mask, red_mask2)
        all_mask = cv2.bitwise_or(red_mask1, red_mask2)

        all_mask = cv2.bitwise_or(all_mask, blue_mask)
        all_mask = cv2.bitwise_or(all_mask, green_mask)


            
        # white_mask + blue_mask + red_mask1 + red_mask2 + green_mask
        # all_mask = cv2.bitwise_or(thresh, all_mask)
        out2 = cv2.bitwise_not(all_mask)
        out2 = cv2.bitwise_or(out2, edges)
        out = cv2.bitwise_not(out2)
        out = cv2.dilate(out, kernel)
        out = cv2.dilate(out, kernel)

        out = cv2.morphologyEx(out, cv2.MORPH_CLOSE, kernel)
        out = cv2.dilate(out, kernel)

        # ret, out = cv2.threshold(out, 190, 255, cv2.THRESH_BINARY)

        contours, hierarchy = cv2.findContours(out, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(cv_image, contours,-1,(0,0,255),1)

        for count, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            # print(count, area)
            if(area > 8000 and area < 15000):
            # if area > 0:
                print("Bingo", area)
                x, y, w, h = cv2.boundingRect(contour)
                if x < 740 and x > 0 and w < 150 and h < 150:
                    info_green = np.array([[x, y, w, h, area, 2]])
                    rect = cv2.minAreaRect(contour)
                    box = cv2.boxPoints(rect)
                    angle = rect[-1]
                    rectangle = np.int0(box)
                    u_pos = int((rectangle[0][0] + rectangle[1][0] + rectangle[2][0] + rectangle[3][0])/4)
                    v_pos = int((rectangle[0][1] + rectangle[1][1] + rectangle[2][1] + rectangle[3][1])/4)
                    cv2.circle(cv_image, (u_pos, v_pos), 4, 2)

                    cv2.drawContours(cv_image,[rectangle],0,(0,0,0),2)
                    test_green = np.append(test_green, box, axis = 0)
                    tray_found = True
                    tray_pos = np.array([[u_pos],[v_pos]])

        test_green = np.delete(test_green, 0, 0)
        if tray_found == True and turtlebot_found == True:
            # print(tray_pos)
            # print(turtlebot_pos)
            second_method = True
        else:
            second_method = False





    if turtlebot_pos[0] != 0 and tray_pos[0] != 0 and  tray_found == True and turtlebot_found == True:
        turtlebot_ang = round(find_orientation(tray_pos, turtlebot_pos, angle),2)
        # print(angle)
        cv2.putText(cv_image, "TurtleBot Orientation is: " + str(turtlebot_ang), (10, 650), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0))

    cv2.putText(cv_image, "Position of TurtleBot:" "(" + str(turtlebot_pos_world[0]) + "," + str(turtlebot_pos_world[1]) +")", (10, 700), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0))
    tray_found = False
    turtlebot_found = False

    coord1 = Pose(position=Point(x=x_to_world_float[0], y=y_to_world_float[0], z=turtlebot_ang))
    coordinates.poses.append(coord1)
    coordinate_publisher.publish(coordinates)
    # cv2.circle(cv_image, (0, 300), 4, 2)

    cv2.imshow('Kinect Camera', cv_image)
    # cv2.imshow('Binary', out2)
    cv2.imshow('Black mask', black_mask)
    # cv2.imshow('Canny', edges)
    cv2.imshow('Output', out)


    # print(cv_image.shape) 
    cv2.waitKey(1)

def listener():
    global coordinate_publisher
    rospy.init_node('turtlebot_position_node', anonymous=True)
    rospy.Subscriber("/kinect/rgb_camera/image_raw", Image, callback)
    coordinate_publisher = rospy.Publisher("/turtlebot_position", PoseArray, queue_size=10 )
    camera_info_msg = rospy.wait_for_message("/rgb/camera_info", CameraInfo)
    camera = PinholeCameraModel()
    camera.fromCameraInfo(camera_info_msg)
    rospy.spin()

if __name__=='__main__':
    # while(True):
    #     callback()
    # listener()
    rospy.init_node('turtlebot_position_node', anonymous=True)
    rospy.Subscriber("/kinect/rgb_camera/image_raw", Image, callback)
    coordinate_publisher = rospy.Publisher("/turtlebot_position", PoseArray, queue_size=10 )
    camera_info_msg = rospy.wait_for_message("/rgb/camera_info", CameraInfo)
    camera = PinholeCameraModel()
    camera.fromCameraInfo(camera_info_msg)
    rospy.spin()