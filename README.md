# Seamless Engineering 2022 Workspace 🟩 Group A



## Getting Started
Please follow all steps in order.

For more tips concerning tutorials and working with our Robis packages
take a look in the common folder's readme as well!

### Add an SSH Key for Your Local Device.
To allow git to access repositories from the remote, add an ssh key to your gitlab account. You can find a short guide [here](https://www.theserverside.com/blog/Coffee-Talk-Java-News-Stories-and-Opinions/How-to-configure-GitLab-SSH-keys-for-secure-Git-connections).

### Clone the Repo
To use your repository, clone it to your local device

```bash
git clone git@git.scc.kit.edu:seamless-engineering/se22/students/se22_a_ws.git
cd se22_a_ws
``` 
We provide you with additional packages in the `src/commons` directory. In order to retrieve these, you have to run an addiional command before you continue

```bash
git submodule update --init --recursive --remote
```
This will clone all additional packages.

Your repository is also the catkin workspace for your project. To see if you installed everything correctly, try to build your workspace:

```bash
catkin_make
```
