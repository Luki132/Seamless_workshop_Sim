# Seamless Engineering 2022 Workspace 🟩 Group A



## Getting Started
Please follow all steps in order.

For more tips concerning tutorials and working with our Robis packages
take a look in the common folder's readme as well!

### Add an SSH Key for Your Local Device.
Before you can download and update this repository with the following commands, you need to create a ssh key.
To allow git to access repositories from the remote, add the ssh key to your gitlab account. You can find a short guide [to create and add a key on your pc](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) and [to add a key in github to your account](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account).

### Clone the Repo
To use your repository, clone it to your local device

```bash
git clone git@git.scc.kit.edu:seamless-engineering/se22/students/se22_a_ws.git
``` 
Go to the root folder of your workspace (se22_a_ws).
We will refer to that as YOUR_WS
```bash
cd ~/se22_a_ws
```


We provide you with additional packages in the `src/commons` directory.
These additional packages are linked as submodules to this workspace.
In order to retrieve these, you have to run an addiional command before you continue.

```bash
git submodule update --init --recursive --remote
```
This will clone all additional packages.

Your repository is also the catkin workspace for your project. To see if you installed everything correctly, try to build your workspace:
Make sure you are in the root folder of your workspace first:
```bash
cd ~/se22_a_ws
```
And then build (compile) your workspace

```bash
catkin_make
```
In the end you have to make shure, ROS knows where to look for your files by sourcing your workspace.

You can use the tools in ~/YOUR_WS/src/commons to do that.
