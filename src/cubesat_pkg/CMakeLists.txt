cmake_minimum_required(VERSION 2.8.3)
project(cubesat_pkg)

## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	rospy
	std_msgs
	message_generation
)

 
catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  
install(DIRECTORY rqt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  
install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Declare a C++ executable
add_executable(hello_cpp_e src/hello_cpp_f.cpp)
