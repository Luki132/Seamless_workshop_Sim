import rospy
from geometry_msgs.msg import PoseArray

rospy.init_node("coordinate_subscriber")

# def coordinate_callback(coordinates):
#     # Do something with the received coordinates
#     print(coordinates)

# def coordinate_callback(coordinates):
#     for i, coord in enumerate(coordinates.poses):
#         print("Coordinate {}: x = {}, y = {}, z = {}".format(i+1, coord.position.x, coord.position.y, coord.position.z))

def coordinate_callback(coordinates):
    print("first coordinate: x = {}, y = {}, z = {}".format(coordinates.poses[0].position.x, coordinates.poses[0].position.y,coordinates.poses[0].position.z))
    print("second coordinate: x = {}, y = {}, z = {}".format(coordinates.poses[1].position.x, coordinates.poses[1].position.y,coordinates.poses[1].position.z))
    print("third coordinate: x = {}, y = {}, z = {}".format(coordinates.poses[2].position.x, coordinates.poses[2].position.y,coordinates.poses[2].position.z))

#### Lukas, read this please ######
""" Position of the first cargo: coordinates.poses[0].position.x, coordinates.poses[0].position.y, coordinates.poses[0].position.z 
    Position of the second cargo:coordinates.poses[1].position.x, coordinates.poses[1].position.y, coordinates.poses[1].position.z 
    Position of the third cargo:coordinates.poses[2].position.x, coordinates.poses[2].position.y, coordinates.poses[2].position.z 
"""



coordinate_subscriber = rospy.Subscriber("/cargo_position", PoseArray, coordinate_callback)
rospy.spin()
