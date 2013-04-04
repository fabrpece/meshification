#pragma once

/*
    Copyright (C) 2011-2012 Paolo Simone Gasparello <p.gasparello@sssup.it>

    This file is part of meshificator.

    meshificator is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    meshificator is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with meshificator.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <memory>
#include <vector>

class VideoEncoder;
class AsyncOperation;
namespace RakNet {
class RakPeerInterface;
class SystemAddress;
}

namespace aruco {
class CameraParameters;
class MarkerDetector;
}

class Consumer
{
    std::auto_ptr<AsyncOperation> async_video, async_marker;
    std::auto_ptr<VideoEncoder> encode;
    RakNet::RakPeerInterface* peer;
    std::auto_ptr<RakNet::SystemAddress> address;
    bool is_connected = false;

    std::unique_ptr<aruco::CameraParameters> cam_params;
    std::unique_ptr<aruco::MarkerDetector> marker_detector;
    double modelview[16];

    void connect();

public:
    Consumer(const int w, const int h);
    ~Consumer();
    void operator()(const std::vector<float>& ver, const std::vector<unsigned>& tri, const char* rgb);
};
