#pragma once

#include <MessageIdentifiers.h>

enum VideoMessageType
    {
        ID_USER_PACKET_VIDEO = ID_USER_PACKET_ENUM+1,
        ID_USER_PACKET_CAM = ID_USER_PACKET_VIDEO+1,
        ID_USER_PACKET_FACETRACKER = ID_USER_PACKET_CAM + 1
    };
