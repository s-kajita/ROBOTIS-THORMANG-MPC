cmake_minimum_required(VERSION 3.0.2)
project(sample_manager)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

#### C++ 14 required for MIP SDK ####
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  robotis_controller
  thormang3_base_module
  thormang3_action_module
  sensor_module_biped
  motion_module_rleg
	motion_module_lleg
	motion_module_biped
)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    robotis_controller
    thormang3_base_module
    thormang3_action_module
    sensor_module_biped
    motion_module_rleg
		motion_module_lleg
		motion_module_biped
)

###########
## Build ##
###########


## Specify additional locations of header files
## Your package locations should be listed before other locations
target_include_directories(sensor_module_biped
	PUBLIC
  include
  ../sensor_module_biped/mip_sdk
  ../sensor_module_biped/mip_sdk/src
  ${catkin_INCLUDE_DIRS}
)



### original ####
## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(
#  ${catkin_INCLUDE_DIRS}
#)

add_executable(${PROJECT_NAME} src/sample_manager.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} mip)

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
