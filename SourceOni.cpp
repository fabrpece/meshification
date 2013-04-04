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

#define linux 1
#define i386 1
#include <cstdlib>
#include <stdexcept>
#include <XnCppWrapper.h>
#include "SourceOni.hpp"

SourceOni::SourceOni(const char* filename) :
    ctx(new xn::Context),
    player(new xn::Player),
    depth_generator(new xn::DepthGenerator),
    image_generator(new xn::ImageGenerator)
{
    ctx->Init();
    if (ctx->OpenFileRecording(filename, *player) != XN_STATUS_OK)
        throw std::runtime_error("Unable to load oni file");
    if (ctx->FindExistingNode(XN_NODE_TYPE_DEPTH, *depth_generator) != XN_STATUS_OK)
        throw std::runtime_error("Selected ONI file doesn't contain depth data");
    if (ctx->FindExistingNode(XN_NODE_TYPE_IMAGE, *image_generator) != XN_STATUS_OK)
        throw std::runtime_error("Selected ONI file doesn't contain image data");
    player->SetRepeat(true);
    ctx->StartGeneratingAll();
}

SourceOni::~SourceOni()
{}

void SourceOni::grab(char* rgb, char* depth)
{
    ctx->WaitAndUpdateAll();
    const char* rgb_data = (const char*)image_generator->GetData();
    const char* depth_data = (const char*)depth_generator->GetData();
    std::copy(rgb_data, rgb_data + 3 * 640 * 480, rgb);
    std::copy(depth_data, depth_data + 2 * 640 * 480, depth);
}
