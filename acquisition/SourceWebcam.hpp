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

//#define OPENCV_VIDEO_C_INTERFACE

class SourceWebcam : public Source
{
    int width, height, size;
    float fps;
#ifdef OPENCV_VIDEO_C_INTERFACE
    CvCapture* cap;
#else
    cv::VideoCapture cap;
#endif

public:

    SourceWebcam(const int device_id=0);
    ~SourceWebcam();
    void grab(char* rgb, char* depth=NULL);

    int getWidth(){return width;}
    int getHeigth(){return height;}
    float getFPS(){return fps;}

    inline void setFPS(const int _fps)
    {
        #ifdef OPENCV_VIDEO_C_INTERFACE
            cvSetCaptureProperty( cap, CV_CAP_PROP_FPS, _fps );
            fps = (float)cvGetCaptureProperty(cap,CV_CAP_PROP_FPS);
        #else
            cap.set(CV_CAP_PROP_FPS, _fps);
            fps = (float)cap.get(CV_CAP_PROP_FPS);
        #endif
    }

    void setWidthHeight(const int _w,const int _h)
    {
        setWidth(_w);
        setHeight(_h);
        size = width*height*3;
    }

private:
    inline void setWidth(const int _w)
    {
        #ifdef OPENCV_VIDEO_C_INTERFACE
            cvSetCaptureProperty( cap, CV_CAP_PROP_FRAME_WIDTH, _w );
            width = (int)cvGetCaptureProperty(cap,CV_CAP_PROP_FRAME_WIDTH);
        #else
            cap.set(CV_CAP_PROP_FRAME_WIDTH, _w);
            width = (int)cap.get(CV_CAP_PROP_FRAME_WIDTH);
        #endif
        width = _w;
    }

    inline void setHeight(const int _h)
    {
        #ifdef OPENCV_VIDEO_C_INTERFACE
            cvSetCaptureProperty( cap, CV_CAP_PROP_FRAME_HEIGHT, _h );
            height = (int)cvGetCaptureProperty(cap,CV_CAP_PROP_FRAME_HEIGHT);
        #else
            cap.set(CV_CAP_PROP_FRAME_HEIGHT, _h);
            height = (int)cap.get(CV_CAP_PROP_FRAME_HEIGHT);
        #endif
    }

};
