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


)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_definitions(${EIGEN_DEFINITIONS})
catkin_package()

add_executable(locomotion src/locomotion.cpp src/locomotion_controller.cpp src/dog_controller.cpp src/qp_solver.cpp src/gait_schedular)
target_link_libraries(locomotion
  ${catkin_LIBRARIES}
)

target_link_libraries(locomotion OsqpEigen::OsqpEigen osqp::osqp Eigen3::Eigen)
foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
