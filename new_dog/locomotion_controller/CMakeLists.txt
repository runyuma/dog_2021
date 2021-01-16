cmake_minimum_required(VERSION 2.8.3)

project(locomotion_controller)


find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  roslaunch
  Eigen3
)


include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})
catkin_package()




foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
