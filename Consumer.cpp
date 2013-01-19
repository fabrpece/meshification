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
#include "Consumer.hpp"
#include "VideoEncoder.hpp"
#include "3dzip/3dzip/Writer.hh"

Consumer::Consumer(const int w, const int h) :
    model_stream("kinect0.3d", std::ios::out | std::ios::binary),
    rgb_stream("kinect0.rgb", std::ios::out | std::ios::binary),
    encode(new VideoEncoder(w, h))
{
    if (model_stream.is_open() == false || rgb_stream.is_open() == false)
	throw std::logic_error("Unable to open output stream kinect.{3d,rgb}");
}

Consumer::~Consumer()
{}

void Consumer::operator()(const std::vector<float>& ver, const std::vector<unsigned>& tri, const char* rgb)
{
    const bool compression = false;
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
    model_stream << tmp.rdbuf() << std::flush;
    tmp.str("");
    (*encode)(tmp, rgb);
    tmp << std::flush;
    const int video_size = tmp.tellp();
    rgb_stream << tmp.rdbuf() << std::flush;
    //std::cout << "Model Size: " << model_size * 30 * 8 / 1024.0 <<  "kbps Video Size: " << video_size * 30 * 8 / 1024.0 << "kbps" << std::endl;
}
