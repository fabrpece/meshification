cmake_minimum_required(VERSION 2.8)
project(meshification)
add_definitions(-std=c++0x)

set(CMAKE_BUILD_TYPE Release)

if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Debug CACHE STRING
       "Choose the type of build, options are: None Debug Release RelWithDebInfo
MinSizeRel."
       FORCE )
endif()

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
#find_library(RAKNET_LIBRARY RakNetLibStatic)
find_library(VPX_LIBRARY vpx REQUIRED)
find_library(ARUCO_LIBRARY NAMES aruco aruco124 REQUIRED)

set(RAKNET_INCLUDE_PATH /home/fabrizio/Development/Libraries/Raknet/Source)
set(RAKNET_LIB_PATH /home/fabrizio/Development/Libraries/Raknet/build/Lib/LibStatic/)
set(RAKNET_LIBRARY RakNetStatic)

add_subdirectory(3dzip)
include_directories("~/Devel/include/" ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OPENNI_INCLUDE_DIRS} ${RAKNET_INCLUDE_PATH})
link_directories(${RAKNET_LIB_PATH})
add_subdirectory(acquisition)
#add_subdirectory(viewer)
#add_subdirectory(virtual_camera)
add_subdirectory(receiver)
add_subdirectory(test)
add_subdirectory(cloudSimplifier)
#add_subdirectory(calibration)
