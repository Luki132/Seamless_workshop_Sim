#!/usr/bin/env python


import roslaunch
import rospy
import nav_optimizer


def main():
    # straight from the roslaunch API documentation
    rospy.init_node('en_Mapping', anonymous=True)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/robis/group6_ws/src/common/seamless_environment/se_world.launch"])
    launch.start()
    rospy.loginfo("started")

    rospy.sleep(3)
    # 3 seconds later
    launch.shutdown()


if __name__ == '__main__':
    main()