#pragma once
#include <string>
#include <memory>
#include <vector>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace RakNet {
class RakPeerInterface;
class SystemAddress;
}

class FaceTracker
{
public:
    FaceTracker(const std::string &address, int _minFace = 110, int _maxFace = 320);
    ~FaceTracker();
    bool operator()(IplImage* frame);

private:
    std::string ip_address;
    RakNet::RakPeerInterface* peer;
    std::unique_ptr<RakNet::SystemAddress> address;
    bool is_connected;
    CvHaarClassifierCascade* face_cascade;
    CvMemStorage* pStorageface;
    cv::RNG rng;
    int min_face_w, min_face_h;
    int max_face_w, max_face_h;
    float screen_w, screen_h;
    float max_distance, min_distance;

    void connect();
    void getFOV(float face_w, float face_h, float& FOV_horizontal, float& FOV_vertical);
};
