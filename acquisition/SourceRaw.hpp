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

#include <fstream>
#include "Source.hpp"

class SourceRaw : public Source
{
    int width, height, size;
    std::ifstream depth_stream, rgb_stream;
public:
    SourceRaw(int w, int h, const std::string& name);
    void grab(char* rgb, char* depth);
    int getWidth(){return width;}
    int getHeigth(){return height;}
    void setWidthHeight(const int _w,const int _h){
        width = _w;
        height = _h;
        size = width * height;
    }
};
