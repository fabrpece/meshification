#pragma once
#include <vector>

struct Data3d
{
    std::vector<unsigned> tri;
    std::vector<float> ver, tex, nor;
    std::vector<unsigned char> bgr;
    const int width, height;
    double modelview[16];

    Data3d(int w, int h) :
	bgr(w * h * 3),
	width(w), height(h)
    {}
};
