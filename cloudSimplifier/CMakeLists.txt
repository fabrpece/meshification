find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL QtXml)
include(${QT_USE_FILE})
find_package(PkgConfig)
if (PKG_CONFIG_FOUND)
    pkg_search_module(EIGEN REQUIRED eigen3 eigen2)
    include_directories(${EIGEN_INCLUDE_DIRS})
endif()
QT4_WRAP_CPP(VIEWER_CPP Viewer.hpp)

add_executable(cloudSimplifier ${VIEWER_CPP} main.cpp Viewer.hpp Viewer.cpp)
target_link_libraries(cloudSimplifier ${QT_LIBRARIES} qglviewer-qt4 GLEW GL ${PCL_LIBRARIES} ${OpenCV_LIBS} ${ARUCO_LIBRARY})
