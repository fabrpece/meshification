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
#include "SourceWebcam.hpp"

SourceWebcam::SourceWebcam(const int device_id)
{

#ifdef OPENCV_VIDEO_C_INTERFACE
    cap = cvCreateCameraCapture(device_id);
    if(cap == NULL)
    {
        std::ostringstream error;
        error << "Unable to open device " << device_id << std::endl;
        throw std::logic_error(error.str());
    }
    width = (int)cvGetCaptureProperty(cap,CV_CAP_PROP_FRAME_WIDTH);
    height = (int)cvGetCaptureProperty(cap,CV_CAP_PROP_FRAME_HEIGHT);
    fps = (float)cvGetCaptureProperty(cap,CV_CAP_PROP_FPS);
#else
    cap.open(device_id);
    if (!cap.isOpened()) {
        std::ostringstream error;
        error << "Unable to open device " << device_id << std::endl;
        throw std::logic_error(error.str());
    }
    width = (int)cap.get(CV_CAP_PROP_FRAME_WIDTH);
    height = (int)cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    fps = (float)cap.get(CV_CAP_PROP_FPS);
#endif
    size = width*height*3;
}

SourceWebcam::~SourceWebcam()
{
#ifdef OPENCV_VIDEO_C_INTERFACE
    cvReleaseCapture(&cap);
#endif
}

void SourceWebcam::grab(char* buffer_rgb, char* depth)
{
    #ifdef OPENCV_VIDEO_C_INTERFACE
        cv::Mat frame(cvQueryFrame(cap));
    #else
        cv::Mat frame;
        (cap)>>frame;
    #endif

    if (frame.empty()){
        throw std::runtime_error("Stream interrupted!");
        return;
    }
    memcpy(buffer_rgb, frame.data, size);
}
