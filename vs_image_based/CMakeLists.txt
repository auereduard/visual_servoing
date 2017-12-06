cmake_minimum_required(VERSION 2.8.3)
project(vs_image_based)
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
  INCLUDE_DIRS include
  LIBRARIES vs_image_based
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

add_library(vs_image_based
  src/start_vs_image_based.cpp
  src/pid.cpp
  src/vs_image_based.cpp  
  include/vs_image_based.h
  include/pid.h  
)

add_executable(start_vs_image_based src/start_vs_image_based.cpp)

add_dependencies(vs_image_based ${catkin_EXPORTED_TARGETS} vs_image_based_generate_messages_cpp)
add_dependencies(start_vs_image_based ${catkin_EXPORTED_TARGETS} vs_image_based_generate_messages_cpp)


target_link_libraries(vs_image_based ${catkin_LIBRARIES} dl)
target_link_libraries(start_vs_image_based ${catkin_LIBRARIES} vs_image_based)





install(TARGETS start_vs_image_based DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  
#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()
