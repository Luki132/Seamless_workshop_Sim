import rospy
from std_msgs.msg import String
import json


def cb_nav_state(msg: String):
    pub = rospy.Publisher("/group6/conveyor/incoming_cubes", data_class=String, queue_size=10)

    incoming_cubes = {
        "tray1": "red",
        "tray2": "red",
        "tray3": "blue",
    }

    msg = String()

    msg.data = json.dumps(incoming_cubes)

    pub.publish(msg)


def example_code_subscriber(msg: String):
    incoming_cubes = json.loads(msg.data)
    print(incoming_cubes)


def main():
    rospy.init_node("order_manager")

    sub_nav = rospy.Subscriber("/group6/nav_state", data_class=String, callback=cb_nav_state)
    sub_cubes = rospy.Subscriber("/group6/conveyor/incoming_cubes", data_class=String, callback=example_code_subscriber)

    while not rospy.is_shutdown():
        pass


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

