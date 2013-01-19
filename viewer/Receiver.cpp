#include <iostream>
#include <sstream>
#include <fstream>
#include <pcl/range_image/range_image_planar.h>
#include "../3dzip/3dzip/Reader.hh"
#include "Receiver.hpp"
#include "VideoDecoder.hpp"

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

Receiver::Receiver(QObject* parent, const int w, const int h, const int id) :
    QThread(parent),
    id(id),
    is_running(false),
    data(w, h),
    width(w), height(h)
{}

Receiver::~Receiver()
{
    wait();
}

void Receiver::run()
{
    std::ostringstream filename_stream;
    filename_stream << "kinect" << id;
    const std::string filename = filename_stream.str();
    std::ifstream in((filename + ".3d").c_str(), std::ios::binary);
    if (in.is_open() == false)
	std::cerr << "Unable to open model file " << filename << ".3d" << std::endl;
    std::ifstream in_video((filename + ".rgb").c_str(), std::ios::binary);
    if (in_video.is_open() == false)
	std::cerr << "Unable to open the video stream file " << filename << ".rgb" << std::endl;
    in.exceptions(std::ios::failbit | std::ios::badbit | std::ios::eofbit);
    in_video.exceptions(std::ios::failbit | std::ios::badbit | std::ios::eofbit);
    VideoDecoder decode;
    is_running = true;
    while (is_running) try {
	const bool compression = (in.get() != 0);
	if (compression) {
	    VBE::Reader read;
	    read(in);
	    const int n_tri = read.getNumTri();
	    const int n_ver = read.getNumVer();
	    data.tri.resize(3 * n_tri);
	    data.ver.resize(3 * n_ver);
	    data.tex.resize(2 * n_ver);
	    read.getTriangles(&data.tri[0]);
	    read.getAttrib("V", &data.ver[0]);
	} else {
	    int n_vertices, n_triangles;
	    in.read((char*)&n_vertices, sizeof(n_vertices));
	    data.ver.resize(3 * n_vertices);
	    data.tex.resize(2 * n_vertices);
	    in.read((char*)&data.ver[0], data.ver.size() * sizeof(float));
	    in.read((char*)&n_triangles, sizeof(n_triangles));
	    data.tri.resize(3 * n_triangles);
	    in.read((char*)&data.tri[0], data.tri.size() * sizeof(unsigned));
	}
	decode(in_video, &data.bgr[0]);
	compute_texture_coordinates(width, height, data.ver, data.tex);
	model_ready(&data);
    } catch (const std::ios::failure&) {
	std::cerr << "Error reading from input streams" << std::endl;
	break;
    }
    std::cerr << "Thread exited..." << std::endl;
}

void Receiver::stop()
{
    is_running = false;
}
