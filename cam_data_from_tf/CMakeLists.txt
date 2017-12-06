cmake_minimum_required(VERSION 2.8.3)
project(cam_data_from_tf)
add_compile_options(-std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS             	
            	pluginlib
             	cmake_modules
             	geometric_shapes
	     	kinova_msgs
		roscpp
   		rospy
   		std_msgs
		actionlib
   		message_generation
		genmsg
		actionlib_msgs
		tf
)


find_package(Eigen3 REQUIRED)

find_package(kinova_driver REQUIRED)

catkin_package(
  
  CATKIN_DEPENDS
    	kinova_msgs
	message_runtime
	actionlib
	#roscpp
  DEPENDS
    #orocos_kdl
)

###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS})

include_directories(${kinova_driver_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})

add_library(cam_data_from_tf
  src/receive_camera_data.cpp   
)

add_executable(receive_camera_data src/receive_camera_data.cpp)

add_dependencies(cam_data_from_tf ${catkin_EXPORTED_TARGETS} cam_data_from_tf_generate_messages_cpp)
add_dependencies(receive_camera_data ${catkin_EXPORTED_TARGETS} cam_data_from_tf_generate_messages_cpp)


target_link_libraries(cam_data_from_tf ${catkin_LIBRARIES} dl)
target_link_libraries(receive_camera_data ${catkin_LIBRARIES} cam_data_from_tf)





install(TARGETS receive_camera_data DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})







