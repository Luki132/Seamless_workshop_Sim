import rospy
from rosnode import rosnode_listnodes
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from robis_messages.msg import MoveActionGoal
import subprocess
import multiprocessing as mp
from string import whitespace
import asyncio


# Name of the node of this script. Search for its usage to find out when and
# why it's needed.
NODE_NAME = "emergency_shutdown"


# Wrapper for executing ros related commands like you do in a normal terminal.
# Sourcing ~/.bashrc doesn't work because bashrc can only be sourced from an
# interactive terminal (see "If not running interactively, don't do anything"
# in .bashrc)
# Therefore the sourcing of the relevant stuff must be made manually before
# executing the actual command
# This is kinda hacky and specific to the SE22 setup:
#   * Source the ros setup.bash file (hardcoded here)
#   * Find the SE22 workshop params (like IP addresses etc) and run those
#     commands as well.
async def bash_run(cmd):
    init_cmds = ['source /opt/ros/noetic/setup.bash']
    with open("/home/robis/.bashrc") as f:
        bashrc = f.read()

    # Find SE22 config, discard everything before, then discard the
    # se22 config comment.
    se22_start = "#=== SE22 Workspace/Simulation Configurations ===#"
    if se22_start in bashrc:
        bashrc = bashrc[bashrc.index(se22_start):].splitlines()[1:]
        init_cmds += bashrc
    init_cmds = " && ".join([c for c in init_cmds if c.strip(whitespace)])
    if type(cmd) is not str:
        raise TypeError(f"cmd must be of type str, got {type(cmd)}: {cmd}")
    final_cmd = f"bash -c '{init_cmds} && {cmd}'"
    print("executing:", cmd)

    proc = await asyncio.create_subprocess_shell(
        final_cmd,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE,
    )
    stdout, stderr = await proc.communicate()
    return stdout, stderr


# rospy doesn't provide a function to get the publishers or subscribers of a topic.
# Therefore, they must be gathered from the console output of rostopic info.
#
# Example output of rostopic info /rosout:
#   Type: rosgraph_msgs/Log
#
#   Publishers:
#    * /pycon1 (http://robis-ubuntu20-usb:42955/)
#    * /pycon2 (http://robis-ubuntu20-usb:42956/)
#
#   Subscribers:
#    * /rosout (http://robis-ubuntu20-usb:39559/)
async def get_publishers(topic):
    stdout, stderr = await bash_run(f"rostopic info {topic}")
    if stderr:
        raise Exception(stderr)
    stdout = stdout.decode("utf8")
    publishers = []
    if "Publishers" in stdout:
        stdout = stdout[stdout.index("Publishers"):].splitlines()[1:]
        for p in stdout:
            start, end = " * ", " ("
            if start not in p or end not in p:
                break
            start = p.index(start) + len(start)
            end   = p.index(end)
            p = p[start:end]
            publishers.append(p)
    return publishers


async def kill_node(node):
    # rospy doesn't provide a function to kill nodes. Therefore this
    # wrapper function relies on the CLI command "rosnode kill /node".
    # It also adds basic error handling.
    stdout, stderr = await bash_run(f"rosnode kill {node}")
    if stderr:
        raise Exception(stderr)
    stdout = stdout.decode("utf8")

    # Return values could be used for more robust code.
    if "killed" in stdout:
        return True
    else:
        print(f"Couldn't kill node {node}. stdout: {stdout}")
        return False


class Robot:
    def __init__(self, name, control_topic, stop_msg):
        self.name = name
        self.control_topic = control_topic
        self.stop_msg = stop_msg
        self.pub = rospy.Publisher(self.control_topic, data_class=type(self.stop_msg), queue_size=10)
        self.shell_init = ""

    def log(self, msg):
        print(f"{self.name}: {msg}")

    async def stop(self):
        self.log(f"Getting subscribers of {self.control_topic}")
        nodes_to_kill = await get_publishers(self.control_topic)
        # Exclude this script from the list of nodes that shall be killed.
        nodes_to_kill = [n for n in nodes_to_kill if not n.endswith(NODE_NAME)]
        self.log(f"Nodes to kill: {nodes_to_kill}")
        for node in nodes_to_kill:
            await kill_node(node)
        self.log("All nodes killed.")
        self.log("Stopping robot...")
        self.pub.publish(self.stop_msg)
        self.log("Robot stopped. ")


async def emergency_stop():
    # Set up all the robots that need to be stopped, and specify how they're stopped.
    # Assumes turtlebot-like interfaces (a.k.a. likely to need adjustment for uarms).
    turtlebot = Robot(name="turtlebot", control_topic="/turtlebot1/cmd_vel", stop_msg=Twist())
    uarm1     = Robot(name="uarm1",     control_topic="/uarm1/goal",         stop_msg=MoveActionGoal())
    uarm2     = Robot(name="uarm2",     control_topic="/uarm2/goal",         stop_msg=MoveActionGoal())
    uarm3     = Robot(name="uarm3",     control_topic="/uarm3/goal",         stop_msg=MoveActionGoal())

    robots = [turtlebot, uarm1, uarm2, uarm3]

    # Run the stop functions for each robot. await and async ensure that if one stop action
    # is waiting for something (f.ex. a node shutting down), the other stop actions are not
    # delayed. Not the same as parallelization but close.
    await asyncio.gather(*(robot.stop() for robot in robots))


def callback(msg):
    # This is how you run an async function. Check out the comments in emergency_stop()
    # for details.
    # This likely can be simplified a bit, but I'm not sure how. 
    asyncio.run(emergency_stop())


def main():
    rospy.init_node(NODE_NAME)
    rospy.Subscriber("/group6/emergency_shutdown", data_class=Bool, callback=callback)

    while not rospy.is_shutdown():
        # Prevent excessive CPU load
        rospy.sleep(0.05)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
