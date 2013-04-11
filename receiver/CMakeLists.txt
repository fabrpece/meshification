find_package(OpenGL REQUIRED)
find_library(VPX_LIBRARY vpx)
find_library(GLEW_LIBRARY NAMES GLEW glew32)

if (WIN32)
    set(RECEIVER_LIBRARY_TYPE SHARED)
endif()

add_library(receiver ${RECEIVER_LIBRARY_TYPE}
xvr_receiver.h xvr_receiver.cpp
Receiver.hpp Receiver.cpp
VideoDecoder.hpp VideoDecoder.cpp
Model.hpp Model.cpp
StaticModel.hpp StaticModel.cpp
../common/AsyncWorker.hpp ../common/AsyncWorker.cpp
Data3d.hpp)

target_link_libraries(receiver ${RAKNET_LIBRARY} ${VPX_LIBRARY} 3dzip ${PCL_LIBRARIES} ${GLEW_LIBRARY} ${OPENGL_gl_LIBRARY})

if (WIN32)
	target_link_libraries(receiver ws2_32)
endif()

install(TARGETS receiver RUNTIME DESTINATION .)
