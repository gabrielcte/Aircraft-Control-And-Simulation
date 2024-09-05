# CubeSat tutorial

catkin_make
roscore
roslaunch cubesat_pkg cubesat_rviz.launch
source devel/setup.bash

rostopic list

ros topic echo /imu
ros run joint_state_publisher_gui joint_state_publisher_gui

rosparam load $(find your_package)/config/gazebo_ros_control_params.yaml

