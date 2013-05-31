add_executable( faceTracker
                main.cpp
                FaceTracker.hpp FaceTracker.cpp)

target_link_libraries(faceTracker ${OpenCV_LIBS} ${RAKNET_LIBRARY} pthread m)
#if (WIN32)
#        target_link_libraries(acquisition ws2_32)
#endif()

install(TARGETS faceTracker RUNTIME DESTINATION .)
