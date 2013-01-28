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

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <RakNet/RakPeerInterface.h>
#include <RakNet/RakNetTypes.h>
#include <RakNet/MessageIdentifiers.h>
#include <RakNet/BitStream.h>
#include <aruco/cameraparameters.h>
#include <aruco/markerdetector.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "Consumer.hpp"
#include "VideoEncoder.hpp"
#include "3dzip/3dzip/Writer.hh"

Consumer::Consumer(const int w, const int h) :
    encode(new VideoEncoder(w, h)),
    peer(RakNet::RakPeerInterface::GetInstance()),
    address(new RakNet::SystemAddress),
    cam_params(new aruco::CameraParameters),
    marker_detector(new aruco::MarkerDetector)
{
    auto socket = RakNet::SocketDescriptor();
    peer->Startup(1, &socket, 1);
    connect();
    cam_params->readFromXMLFile("cam.yml");
}

Consumer::~Consumer()
{
    RakNet::RakPeerInterface::DestroyInstance(peer);
}

void Consumer::connect()
{
    peer->Connect("10.100.38.180", 12345, 0, 0);
    //peer->Connect("10.100.32.186", 12345, 0, 0);
}

void Consumer::operator()(const std::vector<float>& ver, const std::vector<unsigned>& tri, const char* rgb)
{
    auto packet_deleter = [this](RakNet::Packet* p) {
        peer->DeallocatePacket(p);
    };

    for (;;) {
        std::unique_ptr<RakNet::Packet, decltype(packet_deleter)> p(peer->Receive(), packet_deleter);
        if (p.get() == 0)
            break;
        const auto id = p->data[0];
        switch (id) {
        case ID_CONNECTION_REQUEST_ACCEPTED:
            if (is_connected == false)
                std::cerr << "Connection established" << std::endl;
            *address = p->systemAddress;
            is_connected = true;
            break;
        case ID_CONNECTION_ATTEMPT_FAILED:
            std::cerr << "Unable to connect to the server" << std::endl;
            is_connected = false;
            connect();
            break;
        case ID_CONNECTION_LOST:
        case ID_DISCONNECTION_NOTIFICATION:
            connect();
            std::cerr << "Connection lost" << std::endl;
            is_connected = false;
            break;
        case ID_NO_FREE_INCOMING_CONNECTIONS:
            connect();
            std::cerr << "The server is full" << std::endl;
            is_connected  = false;
            break;
        default:
            std::cout << "Packet received " << int(id) << std::endl;
        }
    }
    if (is_connected == false)
        return;
    std::vector<aruco::Marker> markers;
    cv::Mat frame(480, 640, CV_8UC3, const_cast<char*>(rgb));
    marker_detector->detect(frame, markers, *cam_params, 0.197, false);
    double modelview[16];
    Eigen::Map<Eigen::Matrix4d> mv_matrix(modelview);
    mv_matrix.setIdentity();
    for (auto& m : markers) {
        if (m.id != 45)
            continue;
        m.draw(frame, cv::Scalar(255, 0, 0));
        cv::Mat rot_src = m.Rvec.clone(), rot;
        rot_src.at<float>(1, 0) *= -1.0f;
        rot_src.at<float>(2, 0) *= -1.0f;
        cv::Rodrigues(rot_src, rot);
        Eigen::Matrix3d r = Eigen::Map<Eigen::Matrix3f>((float*)rot.ptr()).cast<double>();
        Eigen::Vector3d t(-m.Tvec.at<float>(0, 0), m.Tvec.at<float>(1, 0), m.Tvec.at<float>(2, 0));
        Eigen::Affine3d a = Eigen::Affine3d::Identity();
        a.rotate(r).translate(t);
        Eigen::Map<Eigen::Matrix4d> mv_matrix(modelview);
        mv_matrix = a.matrix();
    }
    const bool compression = true;
    std::stringstream tmp(std::ios::in | std::ios::out | std::ios::binary);
    if (compression) {
	tmp.put(1);
	VBE::Writer compress(&tri[0], tri.size() / 3, ver.size() / 3, true);
	compress.addAttrib(&ver[0], ver.size() / 3, 3, "V", 12);
	compress(tmp);
    } else {
	tmp.put(0);
	const int n_vertices = ver.size() / 3, n_triangles = tri.size() / 3;
	tmp.write((const char*)&n_vertices, sizeof(n_vertices));
	tmp.write((const char*)&ver[0], ver.size() * sizeof(float));
	tmp.write((const char*)&n_triangles, sizeof(n_triangles));
	tmp.write((const char*)&tri[0], tri.size() * sizeof(unsigned));
    }
    tmp << std::flush;
    const int model_size = tmp.tellp();
    const std::string& model_string = tmp.str();
    tmp.str("");
    (*encode)(tmp, rgb);
    tmp << std::flush;
    const int video_size = tmp.tellp();
    const std::string& video_string = tmp.str();
    RakNet::BitStream network_stream;
    network_stream.Write(static_cast<RakNet::MessageID>(ID_USER_PACKET_ENUM));
    network_stream.Write(modelview);
    network_stream.Write(model_size);
    network_stream.Write(model_string.c_str(), model_size);
    network_stream.Write(video_size);
    network_stream.Write(video_string.c_str(), video_size);
    peer->Send(&network_stream, HIGH_PRIORITY, UNRELIABLE, 0, *address, false);
    //std::cout << "Model Size: " << model_size * 30 * 8 / 1024.0 <<  "kbps Video Size: " << video_size * 30 * 8 / 1024.0 << "kbps" << std::endl;
}
