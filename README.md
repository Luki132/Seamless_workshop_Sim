# Seamless Engineering 2022 Workspace 🟩 Group A



## Getting started
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