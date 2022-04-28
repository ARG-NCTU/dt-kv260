cmake_minimum_required(VERSION 2.8.3)
project(obstacle_avoidance)

set(CMAKE_BUILD_TYPE Release)
set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  geometry_msgs
  roscpp
  rospy
  pcl_conversions
  pcl_ros
)

#find_package(OpenCV REQUIRED)
#find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################

#generate_messages(
#  DEPENDENCIES
#  std_msgs  # and/or other packages containing depended messages
#  sensor_msgs
#)
catkin_package(
  CATKIN_DEPENDS
  sensor_msgs
  geometry_msgs
  roscpp
  rospy
  pcl_conversions
  pcl_ros
)






###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
  /usr/local/include # Location when using 'make system_install'
  /usr/include       # More usual location (e.g. when installing using a package)
)

## Specify additional locations for library files
link_directories(
  /usr/local/lib # Location when using 'make system_install'
  /usr/lib       # More usual location (e.g. when installing using a package)
)

## Declare executables
#add_executable(vehicleSimulator src/vehicleSimulator.cpp)

## Specify libraries to link a library or executable target against
#target_link_libraries(vehicleSimulator ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

#install(TARGETS vehicleSimulator
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY log/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/log
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

#############
## Testing ##
#############

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})


add_executable(obs src/obs.cpp)
target_link_libraries(obs ${catkin_LIBRARIES})

add_executable(pc_transform src/pc_transform.cpp)
target_link_libraries(pc_transform ${catkin_LIBRARIES})
