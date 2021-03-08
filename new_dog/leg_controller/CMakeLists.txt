cmake_minimum_required(VERSION 2.8.3)

project(leg_controller)


find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  roslaunch
)


include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})
catkin_package()

add_executable(leg_controller src/main.cpp src/leg_controller.cpp src/leg_dynamic.cpp)
target_link_libraries(leg_controller ${catkin_LIBRARIES})



foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
