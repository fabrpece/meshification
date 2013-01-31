cmake_minimum_required(VERSION 2.8)
project(meshification)
add_definitions(-std=c++0x)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_library(RAKNET_LIBRARY RakNetLibStatic)
find_library(VPX_LIBRARY vpx)
find_library(ARUCO_LIBRARY NAMES aruco aruco124 REQUIRED)
add_subdirectory(3dzip)
include_directories(${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OPENNI_INCLUDE_DIRS})
set_source_files_properties(triangle.cpp PROPERTIES COMPILE_DEFINITIONS "TRILIBRARY;EXTERNAL_TEST;ANSI_DECLARATORS")
add_executable(main main.cpp
	DepthMeshifier.hpp DepthMeshifier.cpp
	Triangulator.hpp Triangulator.cpp
	triangle.h triangle.cpp
	SurfaceReconstruction.hpp SurfaceReconstruction.cpp
	MeshBuilder.hpp MeshBuilder.cpp
	Consumer.hpp Consumer.cpp
	VideoEncoder.hpp VideoEncoder.cpp
	SourceRaw.hpp SourceRaw.cpp
	SourceKinect.hpp SourceKinect.cpp
	SourceOni.hpp SourceOni.cpp
	DepthFilter.hpp DepthFilter.cpp)
target_link_libraries(main ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OPENNI_LIBRARY} 3dzip ${VPX_LIBRARY} ${RAKNET_LIBRARY} ${ARUCO_LIBRARY})
if (WIN32)
	target_link_libraries(main ws2_32)
endif()
add_subdirectory(viewer)
add_subdirectory(virtual_camera)
add_subdirectory(receiver)
add_subdirectory(test)
