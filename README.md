# Seamless Engineering 2022 Workspace 🟩 Group A
Project Workspace for the lecture "Seamless Engineering", winter term 2022/23.

Feel free to alter this readme to help your team members to run your code.

## Getting Started
Please follow all steps in order.

### Add an SSH Key for Your Local Device.
Before you can download and update this repository with the following commands, you need to create a ssh key.
To allow git to access repositories from the remote, add the ssh key to your gitlab account. You can find a short guide [here](https://medium.com/devops-with-valentine/2021-how-to-your-ssh-key-for-gitlab-on-linux-1b94e2a3a49a).

### Clone the Repo
To use your repository, clone it to your local device

```bash
cd ~
```
```bash
git clone git@git.scc.kit.edu:seamless-engineering/se22/students/se22_a_ws.git
``` 
Go to the root folder of your workspace (se22_a_ws).
```bash
cd ~/se22_a_ws
```

We provide you with additional packages in the `src/common` directory.
These additional packages are linked as submodules to this workspace.
In order to retrieve these, you have to run an additional command before you continue.

```bash
git submodule update --init --recursive --remote
```
This will clone all additional packages.

### Install ROS and other Dependencies
⚠️ *Note: When using one of the Seamless OS sticks, this step has to be skipped.* ⚠️


The `common` directory contains utility shell scripts. To use them, you have to make them executable:
```bash
cd ~/se22_a_ws/src/common/scripts
```
```bash
chmod +x edit_bashrc.sh install_se_pkgs.sh
```

You can install the necessary packages via the `install_se_pkgs.sh` script, or manually. These packages are already
preinstalled on the provided OS sticks. To install via the script enter
```bash
cd ~/se22_a_ws/src/common/scripts
```
```bash
sudo ./install_se_pkgs.sh
```
Manual install instructions are listed below.

### Configure ROS
To check if all packages were downloaded correctly, try to build the whole workspace and observe if all 
common packages are compiled:
```bash
cd ~/se22_a_ws/
```
```bash
catkin_make
```

Next , we'll tell ROS where to look for resources. 

Then just run the script `edit_bashrc.sh`:

```bash
cd ~/se22_a_ws/src/common/scripts
```
```bash
chmod +x edit_bashrc.sh
```
```bash
./edit_bashrc.sh
```

And you are good to go!

## Usage

### Simulation
All models and provided nodes can be started using launch files. A good place to start is the full Seamless Engineering
world with all models loaded. Just launch it via:
```bash
    roslaunch seamless_environment se_world.launch
```
This also includes visualization with Rviz.
Feel free to copy this launch file and modify it according to your personal task,
by commenting out unnecessary models.
Turn off navigation in this launch file to get an exact position of the turtlebot.

### Where do I put My Own Code?
This repository is your catkin workspace. Therefore, all ROS packages you develop need to be placed in the `src` folder.
There you can create your own packages. Just be careful not to alter code in the `common` folder, as this could create issues 
when downloading new versions of the common packages.

## Tutorials
Example script: `src/common/robis_uarm/scripts/very_simple_action_client_example.py`

### Useful Links
- [Git Submodules](https://www.vogella.com/tutorials/GitSubmodules/article.html)
- ROS:
  - [rqt](http://wiki.ros.org/rqt), [rqt plugins](http://wiki.ros.org/rqt/Plugins): Topic Monitor, Easy Message Publisher, TF-Tree, Node-Graph
  - [RViz](http://wiki.ros.org/rviz)
  - [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)


## Manual ROS Installation:

### ROS
Install ROS noetic desktop full!
(Instructions can be found [here](http://wiki.ros.org/noetic/Installation/Ubuntu)).

don't forget:
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```
and
```bash
source ~/.bashrc    
```

### Turtlebot3 Packages
```bash
sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
ros-noetic-rgbd-launch ros-noetic-depthimage-to-laserscan \
ros-noetic-rosserial-arduino ros-noetic-rosserial-python \
ros-noetic-rosserial-server ros-noetic-rosserial-client \
ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
ros-noetic-compressed-image-transport ros-noetic-rqt* \
ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers \
ros-noetic-dynamixel-sdk ros-noetic-turtlebot3-msgs ros-noetic-turtlebot3 \
ros-noetic-effort-controllers
```

### Required Python Packages
```bash
pip install robust_serial squaternion shapely nptyping
```
