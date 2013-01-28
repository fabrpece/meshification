add_library(receiver
xvr_receiver.h xvr_receiver.cpp
Receiver.hpp Receiver.cpp
VideoDecoder.hpp VideoDecoder.cpp
Model.hpp Model.cpp
Data3d.hpp)

target_link_libraries(receiver ${RAKNET_LIBRARY} vpx 3dzip ${PCL_LIBRARIES})
