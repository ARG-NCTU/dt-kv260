cmake_minimum_required(VERSION 3.1)
set(PROJECT_NAME dt_kv260_one_node)
project(${PROJECT_NAME})

############################################################################
# Flags

cmake_policy(SET CMP0054 NEW)

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
set(${PROJECT_NAME}_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS "${${PROJECT_NAME}_CXX_FLAGS} -Wall -pedantic -g")

if(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
    set(CMAKE_CXX_FLAGS_RELEASE "-O2 -s -flto") # File size optimization
    message("* Release mode")
endif()

if(CMAKE_BUILD_TYPE MATCHES Debug)
    message("* Debug mode")
    SET(CMAKE_DEBUG_POSTFIX  "d")
else()
    set(CMAKE_CXX_FLAGS_RELEASE "-O2 -s -flto") # File size optimization
    message("* Release mode")
endif()

# Allows to start application by double clicking the icon on Ubuntu
set ( CMAKE_CXX_FLAGS "-no-pie" )

# CPU information
message("Processor: ${CMAKE_SYSTEM_PROCESSOR}")

# Raspberry Pi 4 - OS 64bit
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message("* Optimizing for ${CMAKE_SYSTEM_PROCESSOR}")
  add_compile_options(-mtune=cortex-a72 -mcpu=cortex-a72)
  add_definitions(-DEMBEDDED_ARM)
endif()

# Raspberry Pi 4 - OS 32bit
if(CMAKE_SYSTEM_PROCESSOR MATCHES "armv7l")
  message("* Optimizing for ${CMAKE_SYSTEM_PROCESSOR}")
  # add_compile_options(-mcpu=cortex-a72 -mfloat-abi=hard -mfpu=neon-fp-armv8 -mneon-for-64bits -mtune=cortex-a72)
  add_compile_options(-mfloat-abi=hard -mfpu=neon-fp-armv8 -mneon-for-64bits -mtune=cortex-a72)
  add_definitions(-DEMBEDDED_ARM)
endif()


############################################################################
# Options
option(BUILD_VIDEO      "Build the ZED Open Capture Video Modules (only for Linux)"   ON)
option(BUILD_SENSORS    "Build the ZED Open Capture Sensors Modules"                  ON)
#option(BUILD_EXAMPLES   "Build the ZED Open Capture examples"                         ON)
option(DEBUG_CAM_REG    "Add functions to log the values of the registers of camera"  OFF)

############################################################################
# Sources
set(SRC_VIDEO
    ${PROJECT_SOURCE_DIR}/src/videocapture.cpp
)

set(SRC_SENSORS
    ${PROJECT_SOURCE_DIR}/src/sensorcapture.cpp
)

############################################################################
# Includes
set(HEADERS_VIDEO
    # Base
    ${PROJECT_SOURCE_DIR}/include/videocapture.hpp

    # Defines
    ${PROJECT_SOURCE_DIR}/include/defines.hpp
    ${PROJECT_SOURCE_DIR}/include/videocapture_def.hpp
)

set(HEADERS_SENSORS
    # Base
    ${PROJECT_SOURCE_DIR}/include/sensorcapture.hpp

    # Defines
    ${PROJECT_SOURCE_DIR}/include/defines.hpp
    ${PROJECT_SOURCE_DIR}/include/sensorcapture_def.hpp
)



############################################################################
# Required external libraries

list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake")

############################################################################
# Generate libraries
if(DEBUG_CAM_REG)
    message("* Registers logging available")
    add_definitions(-DSENSOR_LOG_AVAILABLE)
endif()

if(BUILD_SENSORS)
    message("* Sensors module available")
    add_definitions(-DSENSORS_MOD_AVAILABLE)

    find_package(HIDAPI REQUIRED)
    message(STATUS "libhidapi: include dir at ${HIDAPI_INCLUDE_DIRS}")
    message(STATUS "libhidapi: libraries at ${HIDAPI_LIBRARIES}")
    #include_directories(${HIDAPI_INCLUDE_DIRS})

    set(SRC_FULL ${SRC_FULL} ${SRC_SENSORS})
    set(HDR_FULL ${HDR_FULL} ${HEADERS_SENSORS})
    set(DEP_LIBS ${DEP_LIBS}
        ${LibUSB_LIBRARIES}
        ${HIDAPI_LIBRARIES}
        pthread )
endif()

if(BUILD_VIDEO)
    message("* Video module available")
    add_definitions(-DVIDEO_MOD_AVAILABLE)

    find_package(LIBUSB 1 REQUIRED)
    message(STATUS "libusb: include dir at ${LibUSB_INCLUDE_DIRS}")
    message(STATUS "libusb: libraries at ${LibUSB_LIBRARIES}")
    #include_directories(${LibUSB_INCLUDE_DIRS})
    set(SRC_FULL ${SRC_FULL} ${SRC_VIDEO})
    set(HDR_FULL ${HDR_FULL} ${HEADERS_VIDEO})
    set(DEP_LIBS ${DEP_LIBS}
         ${LibUSB_LIBRARIES}
         )

endif()

add_library(${PROJECT_NAME} SHARED ${SRC_FULL} )
target_link_libraries( ${PROJECT_NAME}  ${DEP_LIBS})

############################################################################
# Install rules
set_target_properties(${PROJECT_NAME} PROPERTIES
    PUBLIC_HEADER "${HDR_FULL}"
)
install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_PREFIX}/include/zed-open-capture)


############################################################################
############################################################################
find_package(OpenCV REQUIRED)
find_package(OpenMP)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  message_generation
  tf2
  sensor_msgs
  cv_bridge
  std_msgs
  image_transport
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_msgs  # Or other packages containing msgs
)
catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  pcl_ros
  geometry_msgs
  nav_msgs std_msgs
  tf2
  message_runtime
  sensor_msgs
  cv_bridge
  std_msgs
  image_transport
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${HIDAPI_INCLUDE_DIRS}
  ${LibUSB_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(one_node src/dt_kv260_one_node.cpp src/dt_kv260_one_node.h )
add_dependencies(one_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)
target_link_libraries(one_node ${catkin_LIBRARIES} ${PROJECT_NAME} ${OpenCV_LIBS})
