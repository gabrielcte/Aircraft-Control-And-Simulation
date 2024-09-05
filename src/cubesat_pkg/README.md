# CubeSat tutorial

catkin_make
source devel/setup.bash
roslaunch cubesat_pkg cubesat_rviz.launch


rostopic list

rostopic echo /imu
rosrun joint_state_publisher_gui joint_state_publisher_gui

rosparam load $(find your_package)/config/gazebo_ros_control_params.yaml

