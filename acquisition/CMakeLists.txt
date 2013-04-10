set_source_files_properties(triangle.cpp PROPERTIES COMPILE_DEFINITIONS "TRILIBRARY;EXTERNAL_TEST;ANSI_DECLARATORS")
add_executable(aquisition main.cpp
	DepthMeshifier.hpp DepthMeshifier.cpp
	Triangulator.hpp Triangulator.cpp
	triangle.h triangle.cpp
	SurfaceReconstruction.hpp SurfaceReconstruction.cpp
	MeshBuilder.hpp MeshBuilder.cpp
	Consumer.hpp Consumer.cpp
	VideoEncoder.hpp VideoEncoder.cpp
        AsyncWorker.hpp AsyncWorker.cpp
	SourceRaw.hpp SourceRaw.cpp
	SourceKinect.hpp SourceKinect.cpp
	SourceOni.hpp SourceOni.cpp
	DepthFilter.hpp DepthFilter.cpp)
target_link_libraries(aquisition ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OPENNI_LIBRARY} 3dzip ${VPX_LIBRARY} ${RAKNET_LIBRARY} ${ARUCO_LIBRARY})
if (WIN32)
        target_link_libraries(aquisition ws2_32)
endif()