cmake_minimum_required(VERSION 2.8.3)
project(gps_common CXX)

find_package(Eigen3 REQUIRED)

############
## Catkin ##
############

set(DEPS
  message_filters
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  rospy
  novatel_gps_msgs
  tf
  geometry_msgs
)

find_package(catkin REQUIRED COMPONENTS
             ${DEPS})

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
    GPSStatus.msg
    GPSFix.msg
)

generate_messages(DEPENDENCIES
  nav_msgs
  sensor_msgs
  std_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################

catkin_package(INCLUDE_DIRS include
               CATKIN_DEPENDS
               ${DEPS})

###########
## Build ##
###########
add_compile_options(-std=c++11)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
add_executable(${PROJECT_NAME}/utm_odometry_node src/utm_odometry_node.cpp)
set_target_properties(${PROJECT_NAME}/utm_odometry_node PROPERTIES OUTPUT_NAME "utm_odometry_node")
target_link_libraries(${PROJECT_NAME}/utm_odometry_node ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}/utm_odometry_node
                 ${catkin_EXPORTED_TARGETS}
                 ${${PROJECT_NAME}_EXPORTED_TARGETS})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}/utm_odometry_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

catkin_install_python(PROGRAMS nodes/fix_translator
                      nodes/ins_translator
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
