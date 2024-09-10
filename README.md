# dev_ws
## Instalação do ROS Noetic


sudo apt install curl # if you haven't already installed curl

curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -

sudo apt update

sudo apt install ros-noetic-desktop-full

sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers

sudo apt install ros-noetic-core

source /opt/ros/noetic/setup.bash

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

sudo apt install python3-rosdep

## Criando e configurando repositório 

source /opt/ros/noetic/setup.bash
mkdir -p dev_ws/src
cd dev_ws
catkin_make
source devel/setup.bash
echo $ROS_PACKAGE_PATH
/home/youruser/catkin_ws/src:/opt/ros/kinetic/share

ls
cd src
catkin_create_pkg cubesat_1u std_msgs rospy roscpp

cd ~/dev_ws
catkin_make
. ~/dev_ws/devel/setup.bash

mkdir -p dev_ws/src/cubesat_1u/description
mkdir -p dev_ws/src/cubesat_1u/launch
mkdir -p dev_ws/src/cubesat_1u/worlds
mkdir -p dev_ws/src/cubesat_1u/config