add_executable(calibration calibration.cpp)
target_link_libraries(calibration ${OpenCV_LIBS} opencv_calib3d)
