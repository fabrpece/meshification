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

#include <opencv2/opencv.hpp>
#include "SurfaceReconstruction.hpp"
#include "MeshBuilder.hpp"

static bool is_inner(const cv::Point& p, const cv::Mat& depth)
{
    return depth.at<unsigned short>(p) != 0;
}

static bool is_joinable(const cv::Point& p1, const cv::Point& p2, const cv::Mat& depth)
{
    if (::is_inner(p1, depth) == false || ::is_inner(p2, depth) == false)
	return false;
    //return true;
    const unsigned short depth_diff = std::abs(depth.at<ushort>(p1) - depth.at<ushort>(p2));
    return depth_diff < 200;
}

static cv::Point intersect(const cv::Point& p1, const cv::Point& p2, const cv::Mat& depth)
{
    if (std::abs(p1.x - p2.x) <= 1 && std::abs(p1.y - p2.y) <= 1)
	return ::is_joinable(p1, p2, depth) == true ? p2 : p1;
    const cv::Point mid = (p1 + p2) * 0.5;
    return ::is_joinable(p1, mid, depth) == true ? ::intersect(mid, p2, depth) : ::intersect(p1, mid, depth);
}

static void extend_point(std::vector<cv::Point>& tri, const cv::Mat& depth)
{
    for (int j = 0; j < 2; ++j)
	tri[1 + j] = intersect(tri[0], tri[1 + j], depth);
}

static void extend_edge(std::vector<cv::Point>& tri, std::vector<cv::Point>& new_tri, const cv::Mat& depth)
{
    new_tri.resize(3);
    for (int j = 0; j < 2; ++j)
	new_tri[1 - j] = intersect(tri[j], tri[2], depth);
    std::vector<double> diagonal(2);
    diagonal[0] = cv::norm(depth.at<unsigned short>(tri[0]) - depth.at<unsigned short>(new_tri[0]));
    diagonal[1] = cv::norm(depth.at<unsigned short>(tri[1]) - depth.at<unsigned short>(new_tri[1]));
    if (diagonal[0] < diagonal[1]) {
	tri[2] = new_tri[0];
	new_tri[2] = tri[0];
    }  else {
	tri[2] = new_tri[1];
	new_tri[2] = tri[1];
    }
}

static void draw_triangle(const std::vector<cv::Point>& p, cv::Mat& mat)
{
    line(mat, p[0], p[1], 255);
    line(mat, p[1], p[2], 255);
    line(mat, p[2], p[0], 255);
}

SurfaceReconstruction::SurfaceReconstruction()
{}

SurfaceReconstruction::~SurfaceReconstruction()
{}

void SurfaceReconstruction::operator()(const std::vector<cv::Vec6f>& triangles, const cv::Mat& depth)
{
    mesh_builder_.reset(new MeshBuilder(depth));
    mesh2d = cv::Mat(depth.size(), CV_8UC1);
    mesh2d.setTo(0);
    for (size_t i = 0; i < triangles.size(); ++i) {
	const cv::Vec6f& t = triangles[i];
	std::vector<cv::Point> pt(3);
	for (int j = 0; j < 3; ++j)
	    pt[j] = cv::Point(t[2 * j], t[2 * j + 1]);
	int n_inner = 0;
	std::vector<bool> inner(3);
	for (int j = 0; j < 3; ++j)
	    if (::is_inner(pt[j], depth) == true) {
		++n_inner;
		inner[j] = true;
	    }
	if (n_inner == 0)
	    continue;
	else if (n_inner == 1) {
	    if (inner[1] == true) {
		std::swap(pt[0], pt[1]);
		std::swap(pt[1], pt[2]);
	    } else if (inner[2] == true) {
		std::swap(pt[0], pt[2]);
		std::swap(pt[1], pt[2]);
	    }
	    extend_point(pt, depth);
	    mesh_builder_->insert(pt);
	    //draw_triangle(pt, mesh2d);
	} else if (n_inner == 2) {
	    if (inner[0] == false) {
		std::swap(pt[2], pt[0]);
		std::swap(pt[0], pt[1]);
	    } else if (inner[1] == false) {
		std::swap(pt[2], pt[1]);
		std::swap(pt[0], pt[1]);
	    }
	    std::vector<cv::Point> pt2(3);
	    if (::is_joinable(pt[0], pt[1], depth)) {
		extend_edge(pt, pt2, depth);
	    } else {
		std::rotate_copy(pt.begin(), pt.begin() + 1, pt.end(), pt2.begin());
		extend_point(pt, depth);
		extend_point(pt2, depth);
	    }
	    mesh_builder_->insert(pt);
	    //draw_triangle(pt, mesh2d);
	    mesh_builder_->insert(pt2);
	    //draw_triangle(pt2, mesh2d);
	} else {
	    const bool ab = ::is_joinable(pt[0], pt[1], depth);
	    const bool bc = ::is_joinable(pt[1], pt[2], depth);
	    const bool ac = ::is_joinable(pt[0], pt[2], depth);
	    if ((ab && bc) || (ab && ac) || (bc && ac)) {
		mesh_builder_->insert(pt);
		//draw_triangle(pt, mesh2d);
		continue;
	    }
	    std::vector<cv::Point> pt2(3), pt3(3);
	    if (ab) {
		std::rotate_copy(pt.begin(), pt.begin() + 2, pt.end(), pt2.begin());
		extend_point(pt2, depth);
		extend_edge(pt, pt3, depth);
	    } else if (bc) {
		std::rotate_copy(pt.begin(), pt.begin() + 1, pt.end(), pt2.begin());
		extend_point(pt, depth);
		extend_edge(pt2, pt3, depth);
	    } else if (ac) {
		std::rotate_copy(pt.begin(), pt.begin() + 1, pt.end(), pt2.begin());
		extend_point(pt2, depth);
		std::rotate(pt.begin(), pt.begin() + 2, pt.end());
		extend_edge(pt, pt3, depth);
	    } else {
		std::rotate_copy(pt.begin(), pt.begin() + 1, pt.end(), pt2.begin());
		extend_point(pt2, depth);
		std::rotate_copy(pt.begin(), pt.begin() + 2, pt.end(), pt3.begin());
		extend_point(pt3, depth);
		extend_point(pt, depth);
	    }
	    mesh_builder_->insert(pt);
	    //draw_triangle(pt, mesh2d);
	    mesh_builder_->insert(pt2);
	    //draw_triangle(pt2, mesh2d);
	    mesh_builder_->insert(pt3);
	    //draw_triangle(pt3, mesh2d);
	}
    }
    //cv::imshow("2D Mesh", mesh2d);
}

void SurfaceReconstruction::write(const std::string& filename)
{
    if (mesh_builder_.get() != 0)
	mesh_builder_->write(filename);
    cv::imwrite("mesh2d.png", mesh2d);
}
