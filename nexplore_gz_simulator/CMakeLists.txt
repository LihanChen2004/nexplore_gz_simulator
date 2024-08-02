cmake_minimum_required(VERSION 2.8.3)
project(nexplore_gz_simulator)

set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sensor_msgs
  geometry_msgs
  gazebo_msgs
  tf
  pcl_ros
  pcl_conversions
  message_generation
  nexplore
)

find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################

add_service_files(
  FILES
  GetPointCloud.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  roslib
  std_msgs
  sensor_msgs
  pcl_ros
  nexplore
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
)

## Declare executables
add_executable(nexplore_gz_simulator src/nexplore_gz_simulator.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(nexplore_gz_simulator ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS nexplore_gz_simulator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY mesh/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/mesh
)
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)
install(DIRECTORY world/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/world
)
