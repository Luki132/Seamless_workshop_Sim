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

# x_offset = 0.54 # Test Area C
# y_offset = 0.49 # Test Area C
counter = 0
test_green = np.empty((1,2))

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

circle_found = False
tray_found = True
circle_pos = np.array([[0], [0]])
tray_pos = np.array([[0], [0]])
second_method = False
test_green = np.empty((1,2))
angle = 0.0

z_diff = 0.6104 - 0.158

x_to_world = [None, None, None]
y_to_world = [None, None, None]
x_to_world_float = [None, None, None]
y_to_world_float = [None, None, None]
area_float = [None, None, None]
counter_img = 0

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
    angle = 0.0
    ang = ang*(-1)
    x = abs(arr1[0] - arr2[0])
    y = abs(arr1[1] - arr2[1])
    # print("x",x)
    # print("y",y)
    if arr1[1] < arr2[1]:
        if arr1[0] > arr2[0] and x > 10: ##
            angle = 90 - ang
        else:
            angle = ang*(-1)
    else:
        if arr1[0] > arr2[0] and x > 10:
            angle = 180 - ang
        else:
            angle = (90 + ang)*(-1)
    print(angle)
    return angle

def circle_detector(img):
    global circle_found,circle_pos
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
            circle_pos = np.array([[x], [y]])
            circle_found = True
            # print("found")


def callback(data):   
    global camera
    coordinates = PoseArray()
 
    global tray_found, circle_found,test_green, tray_pos, circle_pos, cv_image, edges, all_mask, out, second_method,counter, angle, area_float, counter_img

    test_green = np.empty((1,2))
    x_to_world_float = [None, None, None]
    y_to_world_float = [None, None, None]
    area_float = [None, None, None]

    overall_candidate = np.zeros((9,6))

    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    # cv_image = cv2.imread('/home/robis/image_23012023/cv_img_'+ str(counter_img) + '.jpg')
    # cv_image = cv2.imread('/home/robis/image_23012023/cv_img_14.jpg')
    # cv_image1 = cv2.imread('/home/robis/image_23012023/cv_img_14.jpg')

        # cv_image = cv2.imread('/home/robis/cv_image4.jpg')

    if counter_img < 280:
        counter_img = counter_img + 1
    else:
        counter_img = 0
    hsv_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    grayFrame = cv2.normalize(cv_image,None, 150, 255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
    blurFrame= cv2.GaussianBlur(cv_image, (5,5), 0)

    grayFrame = cv2.cvtColor(grayFrame, cv2.COLOR_BGR2GRAY)
    # edges = cv2.Canny(cv_image, threshold1= 100, threshold2=255)

    white_mask = cv2.inRange(hsv_frame, white_lower, white_upper)
    black_mask = cv2.inRange(hsv_frame, black_lower, black_upper)
    blue_mask = cv2.inRange(hsv_frame, blue_lower, blue_upper)
    red_mask1 = cv2.inRange(hsv_frame, red_lower1, red_upper1)
    red_mask2 = cv2.inRange(hsv_frame, red_lower2, red_upper2)
    green_mask = cv2.inRange(hsv_frame, green_lower, green_upper)
    all_mask = white_mask + blue_mask + red_mask1 + red_mask2 + green_mask

    # ret, out = cv2.threshold(all_mask, 190, 255, cv2.THRESH_BINARY)
    # out = cv2.dilate(out, kernel)

    edges = cv2.Canny(blurFrame, threshold1= 100, threshold2=255)
    edges = cv2.dilate(edges, kernel, iterations = 4)
    edges = cv2.erode(edges, kernel, iterations = 3)

        # black_mask = cv2.bitwise_not(black_mask)
        # black_mask = cv2.bitwise_or(black_mask, edges)
        # black_mask = cv2.bitwise_not(black_mask)
        # black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel)

        # contours, hierarchy = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # cv2.drawContours(cv_image, contours,-1,(0,0,255),1)

        # for count, contour in enumerate(contours):
        #     area = cv2.contourArea(contour)
        #     # print(count, area)
        #     if(area > 0):
        #         # print("Bingo", area)
        #         x, y, w, h = cv2.boundingRect(contour)
        #         # if x < 850 and x > 150 and w < 130 and h < 130:
        #         if x > 0:
        #             info_green = np.array([[x, y, w, h, area, 2]])
        #             rect = cv2.minAreaRect(contour)
        #             angle = rect[-1]
        #             angle = (90 + angle) % 180 - 90
        #             # print("Rotation:", angle)
        #             box = cv2.boxPoints(rect)

        #             rectangle = np.int0(box)
        #             u_pos = int((rectangle[0][0] + rectangle[1][0] + rectangle[2][0] + rectangle[3][0])/4)
        #             v_pos = int((rectangle[0][1] + rectangle[1][1] + rectangle[2][1] + rectangle[3][1])/4)
        #             cv2.circle(cv_image, (u_pos, v_pos), 4, 2)
        #             cv2.drawContours(cv_image,[rectangle],0,(0,0,0),2)
        #             test_green = np.append(test_green, box, axis = 0)

        # # detect circles in the image
        # circles = cv2.HoughCircles(grayFrame, cv2.HOUGH_GRADIENT, 1.2, 100, minRadius = 20, maxRadius = 70)
        # # ensure at least some circles were found
        # if circles is not None:
        #     # convert the (x, y) coordinates and radius of the circles to integers
        #     circles = np.round(circles[0, :]).astype("int")
        #     # loop over the (x, y) coordinates and radius of the circles
        #     for (x, y, r) in circles:
        #         # draw the circle in the output image, then draw a rectangle
        #         # corresponding to the center of the circle
        #         cv2.circle(cv_image, (x, y), r, (0, 255, 0), 4)
        #         cv2.rectangle(cv_image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
        #         # print(r)
        #         circle_pos = np.array([[x], [y]])
        #         circle_found = True
        #         print("found")


    hsv_mask = cv2.bitwise_or(red_mask1, red_mask2)
    hsv_mask = cv2.bitwise_or(hsv_mask, red_mask2)
    hsv_mask = cv2.bitwise_or(hsv_mask, blue_mask)
    hsv_mask = cv2.bitwise_or(hsv_mask, green_mask)
    hsv_mask_inv = cv2.bitwise_not(hsv_mask)
    out_inv = cv2.bitwise_or(hsv_mask_inv,edges)
    out = cv2.bitwise_not(out_inv)


    contours, hierarchy = cv2.findContours(out, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    counter = 0
    for count, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > 750 and area < 3500 and counter < 3):
            x, y, w, h = cv2.boundingRect(contour)
            if x < 850 and x > 150 and w < 100 and h < 100 and w > 30 and h > 30:
                # if x > 0:
                print("For count:{}, area: {}, width: {}, heigth:{}".format(count, area, w, h))
                info_green = np.array([[x, y, w, h, area, 2]])
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)

                rectangle = np.int0(box)
                u_pos = int((rectangle[0][0] + rectangle[1][0] + rectangle[2][0] + rectangle[3][0])/4)
                v_pos = int((rectangle[0][1] + rectangle[1][1] + rectangle[2][1] + rectangle[3][1])/4)
                # u_pos = 640
                # v_pos = 360
                y_old, x_old, z_old = camera.projectPixelTo3dRay((u_pos,v_pos)) ## Keep in mind that the pixel here is flip
                x_to_world_float[counter] = round(z_diff/z_old*x_old + x_offset, 3)
                y_to_world_float[counter] = round(z_diff/z_old*y_old + y_offset, 3)
                area_float[counter] = area
                cv2.circle(cv_image, (u_pos, v_pos), 4, 2)
                cv2.drawContours(cv_image,[rectangle],0,(0,0,0),2)
                test_green = np.append(test_green, box, axis = 0)
                tray_found = True
                tray_pos = np.array([[u_pos],[v_pos]])
                print(counter)
                counter = counter + 1
    # print(test_green)

    coord1 = Pose(position=Point(x=x_to_world_float[0], y=y_to_world_float[0], z=area_float[0]))
    coord2 = Pose(position=Point(x=x_to_world_float[1], y=y_to_world_float[1], z=area_float[1]))
    coord3 = Pose(position=Point(x=x_to_world_float[2], y=y_to_world_float[2], z=area_float[2]))

    coordinates.poses.append(coord1)
    coordinates.poses.append(coord2)
    coordinates.poses.append(coord3)

    coordinate_publisher.publish(coordinates)

    tray_found = False
    circle_found = False
    # cv2.circle(cv_image, (0, 300), 4, 2)

    cv2.imshow('Kinect Camera', cv_image)
    cv2.imshow('Out_Inv', out_inv)
    cv2.imshow('HSV_Inv', hsv_mask_inv)
    cv2.imshow('Canny', edges)
    cv2.imshow('HSV', hsv_mask)
    cv2.imshow('Output', out)
    cv2.imshow('Blue', blue_mask)
    cv2.imshow('Greeen',  blurFrame)


    # print("Bingo")
    # print(edges.shape)

    # tray_found = False

    # print(cv_image.shape) # [0] = 280, [1]=720
    cv2.waitKey(1)

def listener():
    global camera

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/rgb/camera_info", Image, callback)
    coordinate_publisher = rospy.Publisher("/cargo_position", PoseArray, queue_size=10 )

    camera_info_msg = rospy.wait_for_message("/kinect1/rgb_camera/camera_info", CameraInfo)
    camera = PinholeCameraModel()
    camera.fromCameraInfo(camera_info_msg)
    rospy.spin()

if __name__=='__main__':
    # while(True):
    #     callback()
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/kinect/rgb_camera/image_raw", Image, callback)
    coordinate_publisher = rospy.Publisher("/cargo_position", PoseArray, queue_size=10 )

    camera_info_msg = rospy.wait_for_message("/rgb/camera_info", CameraInfo)
    camera = PinholeCameraModel()
    camera.fromCameraInfo(camera_info_msg)
    rospy.spin()
