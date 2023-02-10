import asyncio
from string import whitespace
import paramiko


def get_init_cmds():
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
    return init_cmds


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
async def bash_run(cmd, exception_on_error=True):
    init_cmds = get_init_cmds()
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
    if exception_on_error:
        if stderr:
            raise Exception(f"Error running {cmd}. stderr: {stderr}")
    return stdout, stderr


async def ssh_run(name, pw, cmds, exception_on_error=True):
    init_cmds = get_init_cmds()
    init_ssh = f"ssh -o BatchMode=Yes root@{name}"
    final_cmd = f"bash -c '{init_cmds} && {init_ssh}'"
    print("executing:", init_ssh)

    proc = await asyncio.create_subprocess_shell(
        final_cmd,
        stdin=asyncio.subprocess.PIPE,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE,
    )
    print(await proc.communicate())

    # Strip newlines and whitespaces, add them later to ensure all commands are the same
    cmds = [cmd.strip(whitespace) for cmd in cmds]

    # Add password as first command
    cmds = [pw] + cmds

    # Add exit command to the end of the ssh command list if not already there
    if cmds[-1] != "exit":
        cmds.append("exit")

    cmds = [cmd + "\n" for cmd in cmds]

    stdout, stderr = b"", b""
    for i, cmd in enumerate(cmds):
        if type(cmd) is str:
            cmd = cmd.encode("utf8")

        #if not i:
        #    print("ssh exec: [login using given pw]")
        #else:
        print("ssh exec:", cmd)
        o, e = await proc.communicate(input=cmd)
        print("o:", o)
        print("e:", e)
        stdout += o
        stderr += e

        if exception_on_error:
            if stderr:
                raise Exception(f"Error running {cmds}. stderr: {stderr}")

    return stdout, stderr


async def ssh_sync(host, name, pw, cmd):
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    if type(host) is not str:
        host = ".".join([str(x) for x in host])

    print(f"ssh login: {host}")
    ssh.connect(hostname=host, username=name, password=pw)

    # source bashrc manually.
    stdin, stdout, stderr = ssh.exec_command("cat ~/.bashrc")
    bashrc = stdout.read().decode("utf8")
    bashrc = bashrc.rsplit("\nfi\n", 1)[-1].splitlines()
    bashrc = [b.strip(whitespace) for b in bashrc]
    bashrc = " && ".join([b for b in bashrc if b and '"catkin" not in b'])
    bashrc = bashrc.replace("'", "\"")

    # for cmd in bashrc:
    #     print("ssh exec bashrc:", cmd)
    #     stdin, stdout, stderr = ssh.exec_command("bash -c ")
    #     stdout = stdout.read()
    #     stderr = stderr.read()
    #     if stdout:
    #         print(f"ssh stdout: {stdout}")
    #     if stderr:
    #         print(f"ssh stderr: {stderr}")

    print("ssh {name}@{host} exec:", cmd)
    stdin, stdout, stderr = ssh.exec_command(f"bash -c '{bashrc} && {cmd}'")
    if name == "npi":
        for line in stdout:
            print("tb stdout:", line)
    # for line in stdout:
    #     if line:
    #         print(f"ssh {name}@{host} stdout: {line}")
    #     await asyncio.sleep(0.001)
    # for line in stderr:
    #     if line:
    #         print(f"ssh {name}@{host} stderr: {line}")
    #     await asyncio.sleep(0.001)


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

    # Return value could be used for more robust code.
    if b"killed" in stdout:
        return True

    print(f"Couldn't kill node {node}. stdout: {stdout}")
    return False
