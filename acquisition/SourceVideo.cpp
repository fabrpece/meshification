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

#include <sstream>
#include <stdexcept>
#include "SourceVideo.hpp"

SourceVideo::SourceVideo(const std::string& name):
    cap(new cv::VideoCapture(name))
{
    if (!cap->isOpened()) {
        std::ostringstream error;
        error << "Unable to open file " << name << std::endl;
        throw std::logic_error(error.str());
    }
    width = (int)cap->get(CV_CAP_PROP_FRAME_WIDTH);
    height = (int)cap->get(CV_CAP_PROP_FRAME_HEIGHT);
    size = width*height*3;
}

void SourceVideo::grab(char* buffer_rgb, char* buffer_depth)
{
    cv::Mat frame;
    (*cap)>>frame;

    if (frame.empty()){
        throw std::runtime_error("Video finished");
        return;
    }
    memcpy(buffer_rgb,frame.data,size);

}
