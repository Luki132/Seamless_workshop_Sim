import asyncio
from string import whitespace
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID


# Name of the node of this script. Search for its usage to find out when and
# why it's needed.
NODE_NAME = "emergency_shutdown"


# There is or isn't an emergency
emergency = False


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


# Track nodes that are being killed. This is necessary to prevent them from
# being killed multiple times, resulting in errors. Those errors could be
# ignored, however, this could ignore actual errors as well.
# Cleaner ways of doing this likely exist, yet none are known to me atm.
node_kill_list = []


async def kill_node(node):
    global node_kill_list

    # Node has already been killed.
    if node in node_kill_list:
        return True

    node_kill_list.append(node)

    # rospy doesn't provide a function to kill nodes. Therefore, this
    # wrapper function relies on the CLI command "rosnode kill /node".
    # It also adds basic error handling.
    stdout, stderr = await bash_run(f"rosnode kill {node}")

    if stderr:
        # if stderr.startswith(b"ERROR: Unknown node(s):"):
        #     # Node has already been stopped.
        #     return True
        # else:
        #     raise Exception(stderr)
        raise Exception(stderr)

    # Return value could be used for more robust code.
    if b"killed" in stdout:
        return True

    print(f"Couldn't kill node {node}. stdout: {stdout}")
    return False


class Robot:
    log_enable = True

    def __init__(self, name, control_topic, stop_topic, stop_msg):
        self.name = name
        self.control_topic = control_topic
        self.stop_topic = stop_topic
        self.stop_msg = stop_msg
        self.pub = rospy.Publisher(self.stop_topic, data_class=type(self.stop_msg), queue_size=10)
        self.shell_init = ""

    def log(self, msg):
        if Robot.log_enable:
            print(f"{self.name}: {msg}")

    async def disable(self):
        self.log(f"Getting subscribers of {self.control_topic}")
        nodes_to_kill = await get_publishers(self.control_topic)
        # Exclude this script from the list of nodes that shall be killed.
        nodes_to_kill = [n for n in nodes_to_kill if not n.endswith(NODE_NAME)]
        self.log(f"Nodes to kill: {nodes_to_kill}")
        for node in nodes_to_kill:
            await kill_node(node)
        self.log("All nodes killed.")

    def stop(self):
        self.log("Stopping robot...")
        self.pub.publish(self.stop_msg)
        self.log("Robot stopped. ")


# Set up all the robots that need to be stopped, and specify how they're stopped.
# Assumes turtlebot-like interfaces (a.k.a. likely to need adjustment for uarms).
turtlebot = Robot(
    name="turtlebot",
    control_topic="/turtlebot1/cmd_vel",
    stop_topic="/turtlebot1/cmd_vel",
    stop_msg=Twist(),
)
slider = Robot(
    name="slider1",
    control_topic="/slider1/move/goal",
    stop_topic="/slider1/move/cancel",
    stop_msg=GoalID(stamp=rospy.Time.from_sec(0.0), id=""),
)
uarm1 = Robot(
    name="uarm1",
    control_topic="/uarm1/move/goal",
    stop_topic="/uarm1/move/cancel",
    stop_msg=GoalID(stamp=rospy.Time.from_sec(0.0), id=""),
)
uarm2 = Robot(
    name="uarm2",
    control_topic="/uarm2/move/goal",
    stop_topic="/uarm2/move/cancel",
    stop_msg=GoalID(stamp=rospy.Time.from_sec(0.0), id=""),
)
uarm3 = Robot(
    name="uarm3",
    control_topic="/uarm3/move/goal",
    stop_topic="/uarm3/move/cancel",
    stop_msg=GoalID(stamp=rospy.Time.from_sec(0.0), id=""),
)

robots = [turtlebot, slider, uarm1, uarm2, uarm3]


# Run the stop functions for each robot. await and async ensure that if one stop action
# is waiting for something (f.ex. a node shutting down), the other stop actions are not
# delayed. Not the same as parallelization but close.
async def disable_all_robots():
    global robots, node_kill_list

    # reset kill list because nodes could have been respawned
    node_kill_list = []

    await asyncio.gather(*(robot.disable() for robot in robots))


def stop_all_robots():
    global robots
    for robot in robots:
        robot.stop()


def cb_timer_enforce_stop(timer_info: rospy.timer.TimerEvent = None):
    global emergency
    if emergency:
        old = Robot.log_enable
        Robot.log_enable = False
        stop_all_robots()
        Robot.log_enable = old


def cb_set_emergency(msg: Bool):
    global emergency

    emergency = msg.data

    if emergency:
        # This is how you run an async function. Check out the comments in emergency_stop()
        # for details.
        # This likely can be simplified a bit, but I'm not sure how.
        asyncio.run(disable_all_robots())

        # Immediately stop the robots, then continue to enforce it using a timer (see main).
        stop_all_robots()
    else:
        print("Disabling emergency stop enforcement.")


def main():
    rospy.init_node(NODE_NAME)

    rospy.Subscriber(name="/group6/emergency_shutdown", data_class=Bool, callback=cb_set_emergency)
    rospy.Timer(period=rospy.Duration(0.1), callback=cb_timer_enforce_stop)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
