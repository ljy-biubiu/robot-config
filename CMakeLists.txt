cmake_minimum_required(VERSION 2.8.3)
project(robot_config)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	ctilog
	cti_msgs
	tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_config
	CATKIN_DEPENDS roscpp rospy cti_msgs 
#  DEPENDS system_lib
)


SET( CMAKE_BUILD_TYPE "Debug")

FIND_LIBRARY(YAML_CPP_LIBRARIES yaml-cpp)
if(NOT YAML_CPP_LIBRARIES)
	FIND_PACKAGE(yaml-cpp REQUIRED)
	INCLUDE_DIRECTORIES(${YAML_CPP_INCLUDE_DIRS})
endif(NOT YAML_CPP_LIBRARIES)
#find package(PkgConfig REQUIRED)
#pkg_search_module()

include_directories(
# include
	${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/robot_config.cpp )
add_dependencies(${PROJECT_NAME}_node 
#	${${PROJECT_NAME}_EXPORTED_TARGETS} 
	${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} jsoncpp)



install(TARGETS ${PROJECT_NAME}_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

