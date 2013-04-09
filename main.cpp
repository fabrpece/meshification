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
#include <fstream>
#include <iostream>
#include <opencv/cv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "SourceRaw.hpp"
#include "SourceKinect.hpp"
#include "SourceOni.hpp"
#include "Consumer.hpp"
#include "DepthMeshifier.hpp"
#include "AsyncWorker.hpp"

void compare(char* orig_buffer, unsigned short* final_buffer, const cv::Size& size)
{
    cv::Mat depth_orig(size, CV_16UC1, orig_buffer), depth_final(size, CV_16UC1, final_buffer);
    const int n_points = depth_orig.total();
    for (int i = 0; i < n_points; ++i)
        if (final_buffer[i] > 9000)
            final_buffer[i] = 0;
    cv::imshow("Orig Depth", depth_orig);
    cv::imshow("Final Depth", depth_final);
    int max_error = 0;
    cv::Mat error(depth_orig.size(), CV_16UC1), error_mask(depth_orig.size(), CV_8UC1);
    for (int i = 0; i < n_points; ++i) {
        const unsigned short orig = depth_orig.at<unsigned short>(i);
        const unsigned short final = depth_final.at<unsigned short>(i);
        const bool is_valid = orig != 0 && final < 9000;
        const int e = is_valid ? std::abs(orig - final) : 0;
        error.at<unsigned short>(i) = e;
        error_mask.at<unsigned char>(i) = is_valid ? 255 : 0;
        if (e > max_error)
            max_error = e;
    }
    cv::Mat error_graph(depth_orig.size(), CV_8UC3);
    double mse = 0, valid_points = 0;
    for (int i = 0; i < n_points; ++i) {
        const bool is_valid = error_mask.at<unsigned char>(i) != 0;
        const unsigned short e = error.at<unsigned short>(i);
        if (is_valid) {
            ++valid_points;
            mse += e * e;
        }
        const float tmp = e / 40.0f;
        const float v = std::min(1.0f, e / 40.0f);
        const int r = (v > 0.5f ? 0 : v > 0.25f ? -4 * v + 2 : 1.0f) * 255;
        const int g = (v > 0.75f ? -4 * v + 4 : v > 0.25f ? 1.0f : 4 * v) * 255;
        const int b = (v > 0.75f ? 1.0f : v > 0.5f ? 4 * v - 2 : 0.0f) * 255;
        error_graph.at<cv::Vec3b>(i) = is_valid == false ? cv::Vec3b(0, 0, 0) : tmp > 1.0f ? cv::Vec3b(255, 0, 255) : cv::Vec3b(b, g, r);
    }
    std::cout << "MSE: " << mse / valid_points << " (" << valid_points << ')' << std::endl;
    cv::imshow("Error", error_graph);
}

int main(int argc, char** argv)
try {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " filename{.depth} w h" << std::endl;
        return 1;
    }
    const int width = argc < 3 ? 640 : std::atoi(argv[2]);
    const int height = argc < 4 ? 480 : std::atoi(argv[3]);
    std::auto_ptr<Source> camera(std::string(argv[1]) == "-kinect" ? static_cast<Source*>(new SourceKinect) : std::strcmp(".oni", argv[1] + std::strlen(argv[1]) - 4) == 0 ? static_cast<Source*>(new SourceOni(argv[1])) : static_cast<Source*>(new SourceRaw(width, height, argv[1])));
    //std::ifstream decompressed_stream("kinect0.depth", std::ios::binary);
    //if (decompressed_stream.is_open() == false) {
    //std::cerr << "unable to open the stream to get back the decompressed depth map" << std::endl;
    //return -1;
    //}
    const std::string win = "Point Clout Meshifier";
    DepthMeshifier meshify(win, width, height);
    int is_animated = 1;
    bool is_2d_draw_enabled = false, use_color_edges = true;
    Consumer consume(width, height);
    char buffer_depth[2 * width * height];
    std::vector<char> buffer_rgb(3 * width * height);
    std::vector<unsigned> tri;
    std::vector<float> ver;
    std::vector<unsigned short> decompressed_buffer(width * height);
    AsyncWorker consumer_thread;
    for (int frame_id = 0;; ++frame_id) {
        camera->grab(buffer_rgb.data(), buffer_depth);
        int64_t t_begin = cv::getTickCount();
        meshify(buffer_rgb.data(), buffer_depth, tri, ver);
        if (tri.empty())
            continue;
        consumer_thread.begin([=, &consume]{
            consume(ver, tri, buffer_rgb);
        });
        const double t = (cv::getTickCount() - t_begin) / cv::getTickFrequency() * 1000.0;
        //decompressed_stream.read((char*)&decompressed_buffer[0], width * height * 2);
        //::compare(buffer_depth, &decompressed_buffer[0], cv::Size(width, height));
        std::ostringstream label;
        label << "Frame: " << frame_id <<  " #T: " << tri.size() / 3 << ' ' << t << "ms";
        cv::displayStatusBar(win, label.str(), 0);
        cv::Mat img_color(height, width, CV_8UC3, buffer_rgb.data());
        cv::cvtColor(img_color, img_color, CV_RGB2BGR);
        cv::imshow(win, img_color);
        const int c = cv::waitKey(is_animated);
        if (c == 27) {
            break;
        } else if (c == 's') {
            std::ostringstream img_filename;
            img_filename << "img" << std::setw(4) << std::setfill('0') << frame_id << ".jpg";
            cv::imwrite(img_filename.str(), img_color);
        } else if (c == 32)
            is_animated = 1 - is_animated;
        else if (c == 'd') {
            is_2d_draw_enabled = !is_2d_draw_enabled;
            meshify.enable_2d_draw(is_2d_draw_enabled);
        }
        else if (c == 'c') {
            use_color_edges = !use_color_edges;
            meshify.enable_color_edges(use_color_edges);
        }
    }
} catch (const std::exception& e) {
    std::cerr << "Fatal error: " << e.what() << std::endl;
}
