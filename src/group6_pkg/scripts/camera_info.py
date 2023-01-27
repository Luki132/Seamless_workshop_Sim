from cv_bridge import CvBridge
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from image_geometry import PinholeCameraModel

bridge = CvBridge()
# camera_info_= None

def make_camera_msg(cam):
    global camera_info_msg
    camera_info_msg = CameraInfo()
    width, height = cam[0], cam[1]
    fx, fy = cam[2], cam[3]
    cx, cy = cam[4], cam[5]
    camera_info_msg.width = width
    camera_info_msg.height = height
    camera_info_msg.K = [fx, 0, cx,
                         0, fy, cy,
                         0, 0, 1]
                         
    camera_info_msg.D = [0, 0, 0, 0]
    
    camera_info_msg.P = [fx, 0, cx, 0,
                         0, fy, cy, 0,
                         0, 0, 1, 0]
    print(camera_info_msg)
    return camera_info_msg 



def callback():
    pass

def listener():
    rospy.init_node('listener', anonymous=True)
    # rospy.Subscriber("/rgb/camera_info", CameraInfo, make_camera_msg)
    camera_info_msg = rospy.wait_for_message("/rgb/camera_info", CameraInfo)
    camera = PinholeCameraModel()
    camera.fromCameraInfo(camera_info_msg)
    # x_old, y_old, z_old = camera.projectPixelTo3dRay((1280/2,720/2))
    x_old, y_old, z_old = camera.projectPixelTo3dRay((1280/2,720/2))


    y_new = 0.518/z_old*x_old + 0.54
    x_new = 0.518/z_old*y_old + 0.48
    print(y_new - 0.5)
    print(x_new,y_new)
    rospy.spin()

if __name__=='__main__':
    listener()