#pragma once
/*
    Copyright (C) 2011-2013 Fabrizio Pece <f.pece@ucl.cs.ac.uk>

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
#include <memory>
#include "Source.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class SourceVideo : public Source
{
    int width, height, size;
    std::unique_ptr<cv::VideoCapture> cap;

public:
    SourceVideo(const std::string& name);
    void grab(char* rgb, char* depth = NULL);
    int getWidth(){return width;}
    int getHeigth(){return height;}
    void setWidthHeight(const int _w,const int _h){}
};
