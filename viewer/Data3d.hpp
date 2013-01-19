#pragma once
#include <vector>

struct Data3d
{
    std::vector<unsigned> tri;
    std::vector<float> ver, tex;
    std::vector<unsigned char> bgr;
    const int width, height;

    Data3d(int w, int h) :
	bgr(w * h * 3),
	width(w), height(h)
    {}
};
