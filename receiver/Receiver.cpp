#include <iostream>
#include <sstream>
#include <list>
#include <pcl/range_image/range_image_planar.h>
#include <RakNet/RakPeerInterface.h>
#include <RakNet/MessageIdentifiers.h>
#include <RakNet/BitStream.h>
#include <RakNet/RakSleep.h>
#include <Eigen/Core>
#include "../3dzip/3dzip/Reader.hh"
#include "Receiver.hpp"
#include "VideoDecoder.hpp"
#include "Model.hpp"
#include "Data3d.hpp"

static void compute_texture_coordinates(const int width, const int height, const std::vector<float>& v, std::vector<float>& tex)
{
    const float focal_length = 517;
    //const float focal_length = 1885;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    const int n_points = v.size() / 3;
    cloud.reserve(n_points);
    for (int i = 0; i < n_points; ++i)
	cloud.push_back(pcl::PointXYZ(v[3 * i + 0], v[3 * i + 1], v[3 * i + 2]));
    pcl::RangeImagePlanar range_image;
    Eigen::Affine3f affine;
    affine.setIdentity();
    range_image.createFromPointCloudWithFixedSize(cloud, width, height, width / 2.0f, height / 2.0f, focal_length, focal_length, affine);
    for (int i = 0; i < n_points; ++i) {
	range_image.getImagePoint(v[3 * i], v[3 * i + 1], -v[3 * i + 2], tex[i * 2], tex[i * 2 + 1]);
	tex[i * 2] /= width;
	tex[i * 2 + 1] /= -height;
    }
}

static void compute_normals(Data3d& data)
{
    const auto n_elements = data.tri.size();
    if (n_elements == 0)
        return;
    const int n_vertices = data.ver.size() / 3;
    const int n_triangles = data.tri.size() / 3;
    data.nor.resize(3 * n_vertices);
    Eigen::Map<const Eigen::MatrixXf> v(&data.ver[0], 3, n_vertices);
    Eigen::Map<Eigen::MatrixXf> n(data.nor.data(), 3, n_vertices);
    for (int i = 0; i < n_triangles; ++i) {
        unsigned int idx[3];
        for (int j = 0; j < 3; ++j)
            idx[j] = data.tri[3 * i + j];
        const Eigen::Vector3f v0 = v.col(idx[0]);
        const Eigen::Vector3f v1 = v.col(idx[1]);
        const Eigen::Vector3f v2 = v.col(idx[2]);
        const Eigen::Vector3f normal = -(v1 - v0).cross(v2 - v0);
        for (int j = 0; j < 3; ++j)
            n.col(idx[j]) += normal;
    }
    for (int i = 0; i < n_vertices; ++i)
        n.col(i).normalize();
}

Receiver::Receiver()
{}

Receiver::~Receiver()
{
    stop();
}

void Receiver::init()
{
    Model::init();
}

void Receiver::start()
{
    is_running = true;
    t = std::thread(std::bind(&Receiver::run, this));
}

void Receiver::run()
{
    std::unique_ptr<RakNet::RakPeerInterface, decltype(&RakNet::RakPeerInterface::DestroyInstance)> peer(RakNet::RakPeerInterface::GetInstance(), &RakNet::RakPeerInterface::DestroyInstance);
    RakNet::SocketDescriptor socket(12345, 0);
    peer->Startup(1, &socket, 1);
    peer->SetMaximumIncomingConnections(1);
    peer->SetTimeoutTime(1000, RakNet::UNASSIGNED_SYSTEM_ADDRESS);
    VideoDecoder decode;
    is_running = true;
    auto packet_deleter = [&peer](RakNet::Packet* p) {
        peer->DeallocatePacket(p);
    };
    std::vector<char> buffer;
    while (is_running) {
        RakSleep(30);
        const auto ptr = peer->Receive();
        if (ptr == 0)
            continue;
        std::unique_ptr<RakNet::Packet, decltype(packet_deleter)> p(ptr, packet_deleter);
        switch (p->data[0]) {
        case ID_NEW_INCOMING_CONNECTION: {
            std::unique_lock<std::mutex> l(m);
            new_models.insert(p->guid.g);
            break;
        }
        case ID_CONNECTION_LOST:
        case ID_DISCONNECTION_NOTIFICATION: {
            std::unique_lock<std::mutex> l(m);
            delete_models.insert(p->guid.g);
            break;
        }
        case ID_USER_PACKET_ENUM: {
            const int width = 640, height = 480;
            auto data = std::make_shared<Data3d>(width, height);
            RakNet::BitStream bs(p->data, p->length, false);
            bs.IgnoreBytes(sizeof(RakNet::MessageID));
            bs.Read(data->modelview);
            int size;
            bs.Read(size);
            buffer.resize(size);
            bs.Read(buffer.data(), size);
            std::istringstream in(std::string(buffer.begin(), buffer.end()), std::ios::in | std::ios::binary);
            bs.Read(size);
            buffer.resize(size);
            bs.Read(buffer.data(), size);
            std::istringstream in_video(std::string(buffer.begin(), buffer.end()), std::ios::in | std::ios::binary);
            const bool compression = (in.get() != 0);
            if (compression) {
                VBE::Reader read;
                read(in);
                const int n_tri = read.getNumTri();
                const int n_ver = read.getNumVer();
                data->tri.resize(3 * n_tri);
                data->ver.resize(3 * n_ver);
                data->tex.resize(2 * n_ver);
                read.getTriangles(&data->tri[0]);
                read.getAttrib("V", &data->ver[0]);
            } else {
                int n_vertices, n_triangles;
                in.read((char*)&n_vertices, sizeof(n_vertices));
                data->ver.resize(3 * n_vertices);
                data->tex.resize(2 * n_vertices);
                in.read((char*)&data->ver[0], data->ver.size() * sizeof(float));
                in.read((char*)&n_triangles, sizeof(n_triangles));
                data->tri.resize(3 * n_triangles);
                in.read((char*)&data->tri[0], data->tri.size() * sizeof(unsigned));
            }
            decode(in_video, &data->bgr[0]);
            compute_texture_coordinates(width, height, data->ver, data->tex);
            compute_normals(*data);
            std::unique_lock<std::mutex> l(m);
            updates[p->guid.g] = data;
            break;
        }
        }
    }
}

void Receiver::stop()
{
    is_running = false;
    t.join();
}

void Receiver::draw()
{
    std::list<std::pair<std::uint64_t, std::shared_ptr<Data3d>>> data;
    std::unique_lock<std::mutex> l(m);
    for (auto& g : new_models)
        models[g].reset(new Model);
    new_models.clear();
    for (auto& g : delete_models)
        models.erase(g);
    delete_models.clear();
    for (auto u : updates)
        data.push_back(u);
    updates.clear();
    l.unlock();
    for (const auto& d : data) {
        auto it = models.find(d.first);
        if (it == models.end()) {
            std::cerr << "WARNING: Updating an unknown model" << std::endl;
            continue;
        }
        it->second->load(*d.second);
    }

    for (const auto& m : models)
        m.second->draw();
}
