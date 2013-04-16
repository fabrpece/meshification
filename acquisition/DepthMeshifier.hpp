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
#include <string>
#include <vector>

class DepthFilter;
class AsyncWorker;

class DepthMeshifier
{
    const std::string& name;
    const int width, height;
    int near_plane, far_plane;
    int min_threshold, max_threshold;
    int approx_polygon, min_area;
    int dilate_erode_steps;
    int min_contour_area, depth_threshold;

    bool is_draw_2d_enabled;
    bool use_color_edges;

    std::auto_ptr<DepthFilter> filter;
    std::unique_ptr<AsyncWorker> canny_worker, cloud_worker;

public:
    DepthMeshifier(const std::string& name, int w, int h);
    ~DepthMeshifier();
    void operator()(char* buffer_rgb, char* buffer_depth, std::vector<unsigned>& tri, std::vector<float>& ver);
    void enable_2d_draw(bool enabled = true) {
        is_draw_2d_enabled = enabled;
    }
    void enable_color_edges(bool enabled = true) {
        use_color_edges = enabled;
    }
};
