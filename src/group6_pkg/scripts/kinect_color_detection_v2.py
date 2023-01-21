from cv_bridge import CvBridge
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
bridge = CvBridge()
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import Pose, PoseArray, Point


x_offset = 0.275
cube_offset = 0.015
x_to_world = [None, None, None]
y_to_world = [None, None, None]
x_to_world_float = [None, None, None]
y_to_world_float = [None, None, None]
x_prev = 0
x_start = False
test_green = np.empty((1,2))
test_red = np.empty((1,2))
test_blue = np.empty((1,2))

z_diff = 0.6104 - 0.158

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
blue_lower = np.array([100, 200 , 100], np.uint8)
blue_upper = np.array([125, 255 , 255], np.uint8)

red_lower = np.array([0, 200 , 200], np.uint8)
red_upper = np.array([10, 255 , 255], np.uint8)

green_lower = np.array([45, 100 , 200], np.uint8)
green_upper = np.array([75, 255 , 255], np.uint8)



def x_position(x):
    # y = 1.23/720*x - 1.23/720*70
    y = 1.02/720*x
    # print(y)
    return(y)

def y_position(x):
    # y = 0.48/280*x 
    y = 0.375345/280*x
    # print(y + x_offset)
    return(y + x_offset)

def detect_blue_object(hsv_frame, blurred_image):
    global candidate_blue
    global test_blue
    global blue_mask
    candidate_blue = np.empty((1,6))
    blue_mask = cv2.inRange(hsv_frame, blue_lower, blue_upper)

    contours, hierarchy = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    test_blue = np.empty((1,2))
    for count, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        # print(count, area)
        if(area > 300 and area < 1500):
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

def detect_red_object(hsv_frame, blurred_image):
    global candidate_red
    global test_red
    global red_mask

    candidate_red = np.empty((1,6))
    red_mask = cv2.inRange(hsv_frame, red_lower, red_upper)

    contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    test_red = np.empty((1,2))
    for count, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        # print(count, area)
        if(area > 300 and area < 1500):
            # print("Bingo")
            x, y, w, h = cv2.boundingRect(contour)
            info_red = np.array([[x, y, w, h, area, 2]])
            # print("Hello", candidate_red)
            candidate_red = np.append(candidate_red, info_red, axis=0)
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            # print(np.column_stack((box, np.array([[1],[0],[0],[0]]))))
            test_red = np.append(test_red, box, axis = 0)
    test_red = np.delete(test_red, 0, 0)
    candidate_red = np.delete(candidate_red, 0, 0)




def detect_green_object(hsv_frame, blurred_image):
    global candidate_green
    global test_green
    global green_mask
    candidate_green = np.empty((1,6))
    green_mask = cv2.inRange(hsv_frame, green_lower, green_upper)

    contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    test_green = np.empty((1,2))
    for count, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        # print(count, area)
        if(area > 300 and area < 1500):
            # print("Bingo")
            x, y, w, h = cv2.boundingRect(contour)
            info_green = np.array([[x, y, w, h, area, 2]])
            # print("Hello", candidate_green)
            candidate_green = np.append(candidate_green, info_green, axis=0)
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            # print(np.column_stack((box, np.array([[1],[0],[0],[0]]))))
            test_green = np.append(test_green, box, axis = 0)
    test_green = np.delete(test_green, 0, 0)
    candidate_green = np.delete(candidate_green, 0, 0)

    # print(test_green)
    # print(candidate_green)
    # print(test_green[0:3])


def callback(data):
    coordinates = PoseArray()
    overall_candidate = np.zeros((9,6))
    # rospy.loginfo('I heard data')
    num_of_objects = 0
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    blurredFrame = cv2.GaussianBlur(cv_image, (11,11), 0)

    hsvFrame = cv2.cvtColor(blurredFrame, cv2.COLOR_BGR2HSV)
    
    detect_blue_object(hsvFrame, blurredFrame)
    detect_red_object(hsvFrame, blurredFrame)
    detect_green_object(hsvFrame, blurredFrame)

    # print("blue", test_blue.shape[0]/4)
    # print("red", test_red.shape[0]/4)
    # print("green", test_green.shape[0]/4)

    for i in range(int(test_green.shape[0]/4)):
        num_of_objects = num_of_objects + 1
        # print(i*4, i*4+4)
        rectangle = test_green[i*4:i*4+4]
        rectangle = np.int0(rectangle)
        # print(rectangle)
        cv2.drawContours(cv_image,[rectangle],0,(0,0,0),2)
        x_pos = int((rectangle[0][0] + rectangle[1][0] + rectangle[2][0] + rectangle[3][0])/4)
        y_pos = int((rectangle[0][1] + rectangle[1][1] + rectangle[2][1] + rectangle[3][1])/4)
        cv2.circle(cv_image, (x_pos, y_pos), 2, 2)
        # x_to_world[i] = str(round(y_position(y_pos), 3))
        # y_to_world[i] = str(round(x_position(x_pos), 3))
        y_old, x_old, z_old = camera.projectPixelTo3dRay((x_pos,y_pos)) ## Keep in mind that the pixel here is flip
        x_to_world_float[i] = round(z_diff/z_old*x_old + 0.465, 3)
        y_to_world_float[i] = round(z_diff/z_old*y_old + 0.5, 3)

        x_to_world[i] = str(round(z_diff/z_old*x_old + 0.465, 3))
        y_to_world[i] = str(round(z_diff/z_old*y_old + 0.5, 3))
        cv2.putText(cv_image, "green" + "(" + x_to_world[i] + "," + y_to_world[i] + ")", (10, 250 - 20*(num_of_objects-1)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))
    

    for i in range(int(test_red.shape[0]/4)):
        num_of_objects = num_of_objects + 1
        rectangle = test_red[i*4:i*4+4]
        rectangle = np.int0(rectangle)
        cv2.drawContours(cv_image,[rectangle],0,(0,0,0),2)
        x_pos = int((rectangle[0][0] + rectangle[1][0] + rectangle[2][0] + rectangle[3][0])/4)
        y_pos = int((rectangle[0][1] + rectangle[1][1] + rectangle[2][1] + rectangle[3][1])/4)
        cv2.circle(cv_image, (x_pos, y_pos), 2, 2)
        # x_to_world[i] = str(round(y_position(y_pos), 3))
        # y_to_world[i] = str(round(x_position(x_pos), 3))

        y_old, x_old, z_old = camera.projectPixelTo3dRay((x_pos,y_pos)) ## Keep in mind that the pixel here is flip
        x_to_world_float[i] = round(z_diff/z_old*x_old + 0.465, 3)
        y_to_world_float[i] = round(z_diff/z_old*y_old + 0.5, 3)

        x_to_world[i] = str(round(z_diff/z_old*x_old + 0.465, 3))
        y_to_world[i] = str(round(z_diff/z_old*y_old + 0.5, 3))
        cv2.putText(cv_image, "red" + "(" + x_to_world[i] + "," + y_to_world[i] + ")", (10, 250 - 20*(num_of_objects-1)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255))

    for i in range(int(test_blue.shape[0]/4)):
        num_of_objects = num_of_objects + 1
        rectangle = test_blue[i*4:i*4+4]
        rectangle = np.int0(rectangle)
        cv2.drawContours(cv_image,[rectangle],0,(0,0,0),2)
        x_pos = int((rectangle[0][0] + rectangle[1][0] + rectangle[2][0] + rectangle[3][0])/4)
        y_pos = int((rectangle[0][1] + rectangle[1][1] + rectangle[2][1] + rectangle[3][1])/4)
        cv2.circle(cv_image, (x_pos, y_pos), 2, 2)
        # x_to_world[i] = str(round(y_position(y_pos), 3))
        # y_to_world[i] = str(round(x_position(x_pos), 3))
        y_old, x_old, z_old = camera.projectPixelTo3dRay((x_pos,y_pos)) ## Keep in mind that the pixel here is flip
        x_to_world_float[i] = round(z_diff/z_old*x_old + 0.465, 3)
        y_to_world_float[i] = round(z_diff/z_old*y_old + 0.5, 3)

        x_to_world[i] = str(round(z_diff/z_old*x_old + 0.465, 3))
        y_to_world[i] = str(round(z_diff/z_old*y_old + 0.5, 3))
        cv2.putText(cv_image, "blue" + "(" + x_to_world[i] + "," + y_to_world[i] + ")", (10, 250 - 20*(num_of_objects-1)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0))

    print("number of objects detected:", num_of_objects)
    print("number of blue objects:", test_blue.shape[0]/4)
    print("number of red objects:", test_red.shape[0]/4)
    print("number of green objetcs:", test_green.shape[0]/4)


    # #Check if the candidates are valid candidates
    # for i in range(3):
    #     if(candidate_blue[i][4] > 300 and candidate_blue[i][4] < 1000):
    #         overall_candidate[i] = candidate_blue[i] 
    # for i in range(3):
    #     if(candidate_red[i][4] > 300 and candidate_red[i][4] < 1000):
    #         overall_candidate[i+3] = candidate_red[i] 
    # for i in range(3):
    #     if(candidate_green[i][4] > 300 and candidate_green[i][4] < 1000):
    #         overall_candidate[i+6] = candidate_green[i] 
    # print(overall_candidate)
    # overall_candidate = overall_candidate[overall_candidate[:,4].argsort()]
    # print(overall_candidate)    

    # for i in range(3):
    #     x_to_world[i] = str(round(y_position(float(overall_candidate[6+i][1])), 3))
    #     y_to_world[i] = str(round(x_position(float(overall_candidate[6+i][0])), 3))
    
    # for i in range(3):
    #     x = int(overall_candidate[6+i][0])
    #     y = int(overall_candidate[6+i][1])
    #     w = int(overall_candidate[6+i][2])
    #     h = int(overall_candidate[6+i][3])
    #     if(overall_candidate[6+i][5] == 0):
    #         colour = (255,0,0)
    #         colour_str = "blue"
    #     elif(overall_candidate[6+i][5]== 1):
    #         colour = (0, 0, 255)
    #         colour_str = "red"
    #     else:
    #         colour = (0, 255, 0)
    #         colour_str = "green"
    #     cv_image = cv2.rectangle(cv_image, (x,y), (x+w, y+h), colour, 2)
    #     cv2.putText(cv_image, colour_str + "(" + x_to_world[i] + "," + y_to_world[i] + ")", (10, 250 - 20*i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour)

    # if(overall_candidate[8][1] > overall_candidate[7][1]):
    #     print(overall_candidate[8][1])
    #     print("bigger object is below")
    # else:
    #     print("bigger object is above")

    # for i in range(9):
    # real_coordinates = np.array([0.5 , 0.215, 0.00, 1.0]) # x and y is swapped
    # print(P_matrix)
    # print(np.matmul(P_matrix, real_coordinates))
    # u, v, w = np.matmul(P_matrix, real_coordinates)
    # print(u, v)

    # cv2.circle(cv_image, (round(u)+70, round(v)), 10, 10)

    coord1 = Pose(position=Point(x=x_to_world_float[0], y=y_to_world_float[0], z=0.158))
    coord2 = Pose(position=Point(x=x_to_world_float[1], y=y_to_world_float[1], z=0.158))
    coord3 = Pose(position=Point(x=x_to_world_float[2], y=y_to_world_float[2], z=0.158))

    coordinates.poses.append(coord1)
    coordinates.poses.append(coord2)
    coordinates.poses.append(coord3)

    coordinate_publisher.publish(coordinates)

    cv2.putText(red_mask, "RED", (300, 60), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (255,0,0))
    cv2.putText(blue_mask, "BLUE", (280, 60), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (255,0,0))
    cv2.putText(green_mask, "GREEN", (260, 60), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (255,0,0))
    cv2.rectangle(red_mask,(0,0),(720,280),(255,0,0),3)
    cv2.rectangle(blue_mask,(0,0),(720,280),(255,0,0),3)
    cv2.rectangle(green_mask,(0,0),(720,280),(255,0,0),3)

    #cv2.imshow('graycsale image',cv_image)
    cv2.imshow('Kinect Camera', cv_image)
    cv2.imshow('Red', red_mask)
    cv2.imshow('Blue', blue_mask)
    cv2.imshow('Green',  green_mask)


    print(cv_image.shape) # [0] = 280, [1]=720
    cv2.waitKey(1)

# def listener():
#     global camera
#     rospy.init_node('listener', anonymous=True)
#     camera_info_msg = rospy.wait_for_message("/kinect1/rgb_camera/camera_info", CameraInfo)
#     camera = PinholeCameraModel()
#     camera.fromCameraInfo(camera_info_msg)
#     # x_old, y_old, z_old = camera.projectPixelTo3dRay((1280/2,720/2))
#     # x_old, y_old, z_old = camera.projectPixelTo3dRay((1280/2,720/2))


#     # y_new = 0.518/z_old*x_old + 0.54
#     # x_new = 0.518/z_old*y_old + 0.48
#     rospy.Subscriber("/kinect1/rgb_camera/image_raw", Image, callback)
#     coordinate_publisher = rospy.Publisher("/cargo_position", PoseArray, queue_size=10 )
#     rospy.spin()

if __name__=='__main__':
    global camera
    rospy.init_node('listener', anonymous=True)
    camera_info_msg = rospy.wait_for_message("/kinect1/rgb_camera/camera_info", CameraInfo)
    camera = PinholeCameraModel()
    camera.fromCameraInfo(camera_info_msg)
    # x_old, y_old, z_old = camera.projectPixelTo3dRay((1280/2,720/2))
    # x_old, y_old, z_old = camera.projectPixelTo3dRay((1280/2,720/2))


    # y_new = 0.518/z_old*x_old + 0.54
    # x_new = 0.518/z_old*y_old + 0.48
    rospy.Subscriber("/kinect1/rgb_camera/image_raw", Image, callback)
    coordinate_publisher = rospy.Publisher("/cargo_position", PoseArray, queue_size=10 )
    rospy.spin()
