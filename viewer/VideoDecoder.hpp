#pragma once
#include <iosfwd>

class VideoDecoder
{
    struct Impl;
    Impl* p_;

    int n_frames;

public:
    VideoDecoder();
    ~VideoDecoder();
    void operator()(std::istream& in, unsigned char* buffer);
};
