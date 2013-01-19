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
#include "Source.hpp"

namespace xn {
    class Context;
    class DepthGenerator;
    class ImageGenerator;
    class Player;
}

class SourceOni : public Source
{
    std::auto_ptr<xn::Context> ctx;
    std::auto_ptr<xn::Player> player;
    std::auto_ptr<xn::DepthGenerator> depth_generator;
    std::auto_ptr<xn::ImageGenerator> image_generator;
public:
    SourceOni(const char* filename);
    ~SourceOni();
    void grab(char* rgb, char* depth);
};
