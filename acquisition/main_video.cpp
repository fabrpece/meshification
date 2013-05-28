/*
    Copyright (C) 2011-2013 Fabrizio Pece <f.pece@cs.ucl.ac.uk>

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
#include <cstring>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <aruco/cameraparameters.h>
#include "SourceRaw.hpp"
#include "SourceKinect.hpp"
#include "SourceWebcam.hpp"
#include "Consumer.hpp"
#include "../common/AsyncWorker.hpp"

typedef std::chrono::high_resolution_clock myclock;

struct Options
{
    int camera_id;
    int camera_w;
    int camera_h;
    bool verbose;
    std::string calibfile;
    std::string address;

    Options()
    {
        camera_id =0;
        camera_w = 640;
        camera_h = 480;
        verbose = false;
        calibfile = std::string("calib_kin_srl.yml");
        address = std::string("127.0.0.1");
    }
};

void parseArgs(int argc, char** argv, Options& opts)
{

    for(int i=1; i<argc; ++i)
    {
        if( strcmp(argv[i],"--kinect")==0 ||
            strcmp(argv[i],"--webcam")==0)
            continue;
        else if(strcmp(argv[i],"--size")==0)
        {
            ++i;
            opts.camera_w = atoi(argv[i]);
            ++i;
            opts.camera_h = atoi(argv[i]);
        }
        else if(strcmp(argv[i],"--camID")==0)
        {
            ++i;
            if(atoi(argv[i])>=0)
                opts.camera_id = atoi(argv[i]);
        }
        else if(strcmp(argv[i],"--calibFile")==0)
        {
            ++i;
            opts.calibfile = argv[i];
            aruco::CameraParameters cam_params;
            cam_params.readFromXMLFile(opts.calibfile);
            opts.camera_w = cam_params.CamSize.width;
            opts.camera_h = cam_params.CamSize.height;

        }
        else if(strcmp(argv[i],"--ip")==0)
        {
            ++i;
            opts.address = argv[i];
        }
        else if(strcmp(argv[i],"--verbose")==0)
            opts.verbose = true;
        else
            std::cout << "Unknown parameter " << argv[i] << std::endl;
    }
}

int main(int argc, char** argv)
{
    try {
            Options opts;
            if (argc < 2) {
                std::cerr << "Usage: " << argv[0] << " --kinect|--webcam|filename{.depth} [--camID cameraID] [--calibFile calibFile] [--size w h] [--verbose]" << std::endl;
                return 1;
            }
            else
                parseArgs(argc,argv,opts);


        std::auto_ptr<Source> camera(std::string(argv[1]) == "--kinect" ? static_cast<Source*>(new SourceKinect) :
                                     std::string(argv[1]) == "--webcam" ? static_cast<Source*>(new SourceWebcam(opts.camera_id)) :
                                     static_cast<Source*>(new SourceRaw(opts.camera_w, opts.camera_h, argv[1])));
        std::string win_name("Video Sender");

        if(strcmp(argv[1],"--webcam")==0)
            camera->setWidthHeight(opts.camera_w, opts.camera_h);

        const int width = camera->getWidth();
        const int height = camera->getHeigth();
        cv::namedWindow(win_name, CV_WINDOW_AUTOSIZE);

        Consumer consume(width, height, opts.address, std::string("video"), opts.calibfile);

        char buffer_depth[2 * width * height];
        std::vector<char> buffer_rgb(3 * width * height);
        std::vector<unsigned> tri;
        std::vector<float> ver;
        int markerID = 45;
        AsyncWorker consumer_thread;
        for (int frame_id = 0;; ++frame_id) {

            const auto t0 = myclock::now();
            camera->grab(buffer_rgb.data(), buffer_depth);

            if (tri.empty() == false) {
                consumer_thread.begin([=, &consume]{
                    consume(ver, tri, buffer_rgb);
                });
            }

            cv::Mat img_color(height, width, CV_8UC3, buffer_rgb.data());
            cv::imshow(win_name, img_color);
            const int c = cv::waitKey(15);
            if (c == 'v')
                opts.verbose = !opts.verbose;
            else if (c == 27 || c == 'q') {
                break;
            }
            if(opts.verbose)
            {
                const auto t1 = myclock::now();
                std::cout << "Frame: " << frame_id <<  " #T: " << tri.size() / 3 << ' ' <<
                         std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << "ms" << std::endl;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
    }
    return 0;
}
