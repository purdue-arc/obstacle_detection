cmake_minimum_required(VERSION 2.8.3)
project(obstacle_detection)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  sensor_msgs
  std_msgs
  tf
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(
	${OpenCV_INCLUDE_DIRS}
	include
	include/sfm
	/usr/include/eigen3
	${catkin_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
	${sophus_INCLUDE_DIRS}
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES obstacle_detection
#  CATKIN_DEPENDS nav_msgs sensor_msgs std_msgs tf
#  DEPENDS system_lib
	DEPENDS opencv
)

###########
## Build ##
###########

add_library(sfm_config include/sfm/Config.cpp)
target_link_libraries(sfm_config ${catkin_LIBRARIES})

add_library(sfm_frame include/sfm/Frame.cpp)
target_link_libraries(sfm_frame ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} sfm_config)

add_library(sfmlib include/sfm/SFM.cpp)
target_link_libraries(sfmlib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} sfm_config sfm_frame)

add_executable(sfm_node src/sfm_node.cpp)
target_link_libraries(sfm_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${sophus_LIBRARIES} sfmlib)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_obstacle_detection.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
