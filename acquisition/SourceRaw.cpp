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

#include <sstream>
#include <stdexcept>
#include "SourceRaw.hpp"

SourceRaw::SourceRaw(int w, int h, const std::string& name):
    width(w), height(h), size(w * h),
    depth_stream((name + ".depth").c_str(), std::ios::in | std::ios::binary),
    rgb_stream((name + ".rgb").c_str(), std::ios::in | std::ios::binary)
{
    if (depth_stream.is_open() == false || rgb_stream.is_open() == false) {
        std::ostringstream error;
        error << "Unable to open file " << name << ".{rgb|depth}" << std::endl;
        throw std::logic_error(error.str());
    }
}

void SourceRaw::grab(char* buffer_rgb, char* buffer_depth)
{
    depth_stream.read(buffer_depth, size * 2);
    rgb_stream.read(buffer_rgb, size * 3);
    if (depth_stream.eof() != rgb_stream.eof())
        throw std::logic_error("Error: depth and rgb files are badly sized");
    if (depth_stream.eof()) {
        depth_stream.clear();
        depth_stream.seekg(0);
        rgb_stream.clear();
        rgb_stream.seekg(0);
        grab(buffer_rgb, buffer_depth);
    } else if (!depth_stream || !rgb_stream)
        throw std::runtime_error("Error reading depth or rgb file");
}
