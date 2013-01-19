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

#include <cmath>
#include <opencv2/opencv.hpp>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/organized_edge_detection.h>
#include <pcl/2d/edge.h>
#include "DepthFilter.hpp"

DepthFilter::DepthFilter(const int w, const int h) :
    width(w), height(h),
    base(new cv::Mat(height, width, CV_16UC1))
{}

DepthFilter::~DepthFilter()
{}

static void traceEdge(const int i, const int j, cv::Mat& maxima)
{
    if (i < 0 || i > maxima.rows - 1 || j < 0 || j > maxima.cols - 1)
	return;
    float& m = maxima.at<float>(i, j);
    if (m == 0.0f || m == FLT_MAX)
	return;
    m = FLT_MAX;
    ::traceEdge(i + 1, j, maxima);
    ::traceEdge(i - 1, j, maxima);
    ::traceEdge(i + 1, j + 1, maxima);
    ::traceEdge(i - 1, j - 1, maxima);
    ::traceEdge(i, j - 1, maxima);
    ::traceEdge(i, j + 1, maxima);
    ::traceEdge(i - 1, j + 1, maxima);
    ::traceEdge(i + 1, j - 1, maxima);
}

void DepthFilter::operator()(char* color_buffer, unsigned short* depth_buffer, cv::Mat& edges)
{
    const double start_time = pcl::getTime();
    pcl::RangeImagePlanar::Ptr cloud(new pcl::RangeImagePlanar);
    cloud->setDepthImage(depth_buffer, width, height, width / 2, height / 2, 517, 517);
    pcl::IntegralImageNormalEstimation<pcl::RangeImagePlanar::PointType, pcl::Normal> ne;
    ne.setNormalEstimationMethod(ne.AVERAGE_DEPTH_CHANGE);
    ne.setNormalSmoothingSize(10.0f);
    ne.setBorderPolicy(ne.BORDER_POLICY_IGNORE);
    ne.setInputCloud(cloud);
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normal_cloud);
    const double normal_time = pcl::getTime();
    
    /*pcl::PointCloud<pcl::PointXYZI> nx(width, height), ny(width, height);
    nx.resize(width * height);
    ny.resize(width * height);
    for (int i = 0; i < width * height; ++i) {
	nx[i].intensity = (*normal_cloud)[i].normal_x;
	ny[i].intensity = (*normal_cloud)[i].normal_y;
    }
    typedef pcl::PointXYZIEdge PointCanny;
    pcl::Edge<pcl::PointXYZI, PointCanny> edge;
    edge.setHysteresisThresholdLow(0.4f);
    edge.setHysteresisThresholdHigh(1.1f);
    pcl::PointCloud<PointCanny> img_edge;
    edge.canny(nx, ny, img_edge);
    const double end_time = pcl::getTime();
    std::cout << "Normal time: " << normal_time - start_time << " Canny time: " << end_time - normal_time << std::endl;
    for (int i = 0; i < width; ++i)
	for (int j = 0; j < height; ++j) {
	    unsigned char& c = edges.at<unsigned char>(j, i);
	    const float mag = img_edge(i, j).magnitude;
	    c = mag == 255.f ? 255 : 0;
	}
    cv::imshow("PCL Edges", edges);
    return;*/

    cv::Mat nx(height, width, CV_32FC1), ny(height, width, CV_32FC1);
    for (int i = 0; i < nx.total(); ++i) {
	nx.at<float>(i) = (*normal_cloud)[i].normal_x;
	ny.at<float>(i) = (*normal_cloud)[i].normal_y;
    }
    cv::GaussianBlur(nx, nx, cv::Size(3, 3), 1);
    cv::GaussianBlur(ny, ny, cv::Size(3, 3), 1);
    cv::Sobel(nx, nx, -1, 1, 0);
    cv::Sobel(ny, ny, -1, 0, 1);
    cv::Mat magnitude(nx.size(), CV_32FC1), direction(nx.size(), CV_32FC1);
    for (int i = 0; i < nx.total(); ++i) {
	const float x = nx.at<float>(i);
	const float y = ny.at<float>(i);
	magnitude.at<float>(i) = std::sqrt(x * x + y * y);
	direction.at<float>(i) = std::atan2(y, x);
    }
    const float t_low = 0.4f, t_high = 1.1f;
    cv::Mat maxima = cv::Mat::zeros(nx.size(), CV_32FC1);
    for (int i = 0; i < height - 1; ++i)
	for (int j = 0; j < width - 1; ++j) {
	    const float mag = magnitude.at<float>(i, j);
	    if (mag < t_low)
		continue;
	    const float angle = direction.at<float>(i, j) * 180.0 / M_PI;
	    if (((angle <= 22.5) && (angle >= -22.5)) || (angle >= 157.5) || (angle <= -157.5)) {
		if (mag >= magnitude.at<float>(i, j - 1) && mag >= magnitude.at<float>(i, j + 1))
		    maxima.at<float>(i, j) = mag;
	    } else if (((angle > 22.5) && (angle < 67.5)) || ((angle < -112.5) && (angle > -157.5))) {
		if (mag >= magnitude.at<float>(i - 1, j - 1) && mag >= magnitude.at<float>(i + 1, j + 1))
		    maxima.at<float>(i, j) = mag;
	    } else if (((angle >= 67.5) && (angle <= 112.5)) || ((angle <= -67.5) && (angle >= -112.5)))
		if (mag >= magnitude.at<float>(i - 1, j) && mag >= magnitude.at<float>(i + 1, j)) {
		    maxima.at<float>(i, j) = mag;
	    } else if (((angle > 112.5) && (angle < 157.5)) || ((angle < -22.5) && (angle > -67.5))) {
		if (mag >= magnitude.at<float>(i + 1, j - 1) && mag >= magnitude.at<float>(i - 1, j + 1))
		    maxima.at<float>(i, j) = mag;
	    }
	}
    for (int i = 0; i < height - 1; ++i)
	for (int j = 0; j < width - 1; ++j) {
	    const float m = maxima.at<float>(i, j);
	    if (m < t_high)
		continue;
	    ::traceEdge(i, j, maxima);
	}
    const double end_time = pcl::getTime();
    //std::cout << "Normal time: " << normal_time - start_time << " Canny time: " << end_time - normal_time << std::endl;
    for (int i = 0; i < maxima.total(); ++i)
	edges.at<unsigned char>(i) = maxima.at<float>(i) == FLT_MAX ? 255 : 0;
    //cv::imshow("Maxima", maxima);

    /*pcl::PointCloud<pcl::Label> labels;
    std::vector<pcl::PointIndices> label_indices;
    pcl::OrganizedEdgeFromNormals<pcl::RangeImagePlanar::PointType, pcl::Normal, pcl::Label> oed;
    oed.setDepthDisconThreshold(0.04f);
    oed.setMaxSearchNeighbors(100);
    oed.setEdgeType(oed.EDGELABEL_NAN_BOUNDARY | oed.EDGELABEL_OCCLUDING | oed.EDGELABEL_OCCLUDED | oed.EDGELABEL_HIGH_CURVATURE);
    oed.setInputCloud(cloud);
    oed.setInputNormals(normal_cloud);
    oed.compute(labels, label_indices);
    const double end_time = pcl::getTime();
    std::cout << "Normal time: " << normal_time - start_time << " Detect time: " << end_time - normal_time << std::endl;
    edges.setTo(0);
    for (int i = 0; i < 5; ++i) {
	const std::vector<int>& indices = label_indices[i].indices;
	for (int j = 0; j < indices.size(); ++j) {
	    edges.at<unsigned char>(indices[j]) = 255;
	}
    }*/
}
