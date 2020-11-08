cmake_minimum_required(VERSION 2.8.3)

project(dog_sim)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  roslaunch
)
include_directories(include ${catkin_INCLUDE_DIRS})
catkin_package()
add_executable(controller src/control.cpp src/pid.cpp src/sensors_information.cpp src/base_controller.cpp)
target_link_libraries(controller ${catkin_LIBRARIES})


foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
