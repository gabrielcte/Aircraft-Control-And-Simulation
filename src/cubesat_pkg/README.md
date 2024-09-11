# CubeSat1u tutorial

Para executar a simulação

cd ~/dev_ws

catkin_make
source devel/setup.bash
roslaunch cubesat_pkg cubesat_world.launch


Para verificar os sensores 
rostopic list
rostopic echo /imu
rosrun joint_state_publisher_gui joint_state_publisher_gui

rosparam load $(find your_package)/config/gazebo_ros_control_params.yaml

