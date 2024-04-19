# Happy Walker

## ROS1 Noetic Installation
### Apple Silicon MacOS Installation 

Please use Mambaforge instead of Anaconda.
If you are using Anaconda now, please remove the conda init line from the .bashrc or .zshrc file. Then install Mambaforge.
You can download Mambaforge from the following link:

```
# For chinese users
https://mirrors.tuna.tsinghua.edu.cn/github-release/conda-forge/miniforge/LatestRelease/Mambaforge-24.3.0-0-MacOSX-arm64.sh
```
```
# For other users
https://github.com/conda-forge/miniforge/releases/download/24.3.0-0/Mambaforge-24.3.0-0-MacOSX-arm64.sh
```

After installing Mambaforge, you can create a new environment with the following command:
```
mamba create -n ros_env python=3.9.18
mamba activate ros_env

# this adds the conda-forge channel to the new created environment configuration 
conda config --env --add channels conda-forge
# and the robostack channel
conda config --env --add channels robostack-staging
# remove the defaults channel just in case, this might return an error if it is not in the list which is ok
conda config --env --remove channels defaults

# Install ros-noetic into the environment (ROS1)
mamba install ros-noetic-desktop-full

# Initialize dependencies
mamba install ros-noetic-gazebo-ros
mamba install ros-noetic-cv-bridge
mamba install ros-noetic-turtlebot3
mamba install ros-noetic-turtlebot3-simulations
mamba install ros-noetic-turtlebot3-msgs
```


Reactivate the environment to initialize the ros env
```
mamba deactivate
mamba activate ros_env
```

### Testing installation
First terminal:
```
mamba activate ros_env
roscore
```
Second terminal:
```
mamba activate ros_env
rviz
```

### Installation(Linux)
Please use Ubuntu 20.04, then follow https://wiki.ros.org/noetic/Installation/Ubuntu  to install ROS Noetic
After ROS Noetic is installed, you can install the following packages:
```bash
sudo apt-get -y install ros-noetic-gazebo-ros ros-noetic-cv-bridge ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations ros-noetic-turtlebot3-msgs
```

## Get Started
### Install Project Package
```bash
mamba activate ros_env
mkdir -p ~/catkin_ws/src
cp happy_walker ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Run ROS Core
Open a new terminal and run the following commands:
```bash
source /opt/ros/noetic/setup.bash
roscore
```

### Run Turtlebot3 in Gazbo
Open a new terminal and run the following commands:
```bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_house.launch
```
Wait until you see the turtlebot3 in the gazebo world.

### Run Control Node
Open a new terminal and run the following commands:
```bash
cd ~/catkin_ws
source devel/setup.bash
rosrun happy_walker talker.py
```

### Run Image Sender to send a demo image
```bash
cd ~/catkin_ws
source devel/setup.bash
rosrun happy_walker image_sender.py
```
