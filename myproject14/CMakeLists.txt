cmake_minimum_required(VERSION 2.8.3)
project(myproject14)
add_compile_options(-std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
             	actionlib_msgs
		#vs_image_based
            	pluginlib
             	cmake_modules
             	geometric_shapes
	     	kinova_msgs
		roscpp
   		rospy
   		std_msgs
   		message_generation
		genmsg
)


find_package(Eigen3 REQUIRED)

find_package(kinova_driver REQUIRED)

catkin_package(
  INCLUDE_DIRS ../vs_image_based/include
  LIBRARIES vs_image_based
  CATKIN_DEPENDS
    	
    	kinova_msgs
	message_runtime
  DEPENDS
    #orocos_kdl
)

###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS})

include_directories(${kinova_driver_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})

#add_library(myproject14
#  ../vs_image_based/src/visual_servoing_no_moveit.cpp 
#  ../vs_image_based/include/visual_servoing.h
#)

add_executable(start2 src/start2.cpp)

#add_dependencies(vs_image_based ${catkin_LIBRARIES} vs_image_based)

add_dependencies(start2 ${catkin_EXPORTED_TARGETS} vs_image_based_generate_messages_cpp)

target_link_libraries(start2 ${catkin_LIBRARIES} vs_image_based)





install(TARGETS start2 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
