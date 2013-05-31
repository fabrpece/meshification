#include <iostream>
#include "FaceTracker.hpp"
#include <RakPeerInterface.h>
#include <RakNetTypes.h>
#include <MessageIdentifiers.h>
#include <BitStream.h>
#include <RakString.h>
#include "../common/AsyncWorker.hpp"
#include "../common/PacketID.hpp"

FaceTracker::FaceTracker(const std::string &address, int _minFace, int _maxFace) :
    ip_address(address),
    peer(RakNet::RakPeerInterface::GetInstance()),
    address(new RakNet::SystemAddress),
    min_face_h(_minFace),
    min_face_w(_minFace),
    max_face_h(_maxFace),
    max_face_w(_maxFace),
    screen_w(255.2),
    screen_h(144.1),
    max_distance(1000.0),
    min_distance(200.0)
{
    face_cascade = (CvHaarClassifierCascade *)cvLoad("/home/fabrizio/Development/Libraries/OpenCV-2.4.3/data/haarcascades/haarcascade_frontalface_alt_tree.xml", 0 , 0, 0);
    pStorageface = cvCreateMemStorage(0);
    auto socket = RakNet::SocketDescriptor();
    peer->Startup(1, &socket, 1);
    connect();
}

FaceTracker::~FaceTracker()
{
    if(face_cascade)
        cvReleaseHaarClassifierCascade(&face_cascade);
    if(pStorageface)
        cvReleaseMemStorage(&pStorageface);
    RakNet::RakPeerInterface::DestroyInstance(peer);
}

void FaceTracker::connect()
{
    std::cout << "Trying connection to " << ip_address << "::12345" << std::endl;
    peer->Connect(ip_address.c_str(), 12345, 0, 0);
}

bool FaceTracker::operator()(IplImage* frame)
{
    auto packet_deleter = [this](RakNet::Packet* p) {
        peer->DeallocatePacket(p);
    };

    for (;;) {
        std::unique_ptr<RakNet::Packet, decltype(packet_deleter)> p(peer->Receive(), packet_deleter);
        if (p.get() == 0)
            break;
        const auto id = p->data[0];
        switch (id) {
        case ID_CONNECTION_REQUEST_ACCEPTED:
            if (is_connected == false)
                std::cerr << "Connection established" << std::endl;
            *address = p->systemAddress;
            is_connected = true;
            break;
        case ID_CONNECTION_ATTEMPT_FAILED:
            std::cerr << "Unable to connect to the server" << std::endl;
            is_connected = false;
            connect();
            break;
        case ID_CONNECTION_LOST:
        case ID_DISCONNECTION_NOTIFICATION:
            connect();
            std::cerr << "Connection lost" << std::endl;
            is_connected = false;
            break;
        case ID_NO_FREE_INCOMING_CONNECTIONS:
            connect();
            std::cerr << "The server is full" << std::endl;
            is_connected  = false;
            break;
        default:
            std::cout << "Packet received " << int(id) << std::endl;
        }
    }

    if (is_connected == false)
        return false;

    CvSeq * pFaceRectSeq;               // memory-access interface
    float FOV_horizontal =0, FOV_vertical =0;
    // detect faces in image
    pFaceRectSeq = cvHaarDetectObjects
            (frame, face_cascade, pStorageface,
             1.1,                       // increase search scale by 10% each pass
             3,                         // merge groups of three detections
             CV_HAAR_DO_CANNY_PRUNING,  // skip regions unlikely to contain a face
             cvSize(100,100));            // smallest size face to detect = 40x40

    // draw a rectangular outline around each detection
    for(int i=0;i<(pFaceRectSeq? pFaceRectSeq->total:0); i++ )
    {
        CvRect* r = (CvRect*)cvGetSeqElem(pFaceRectSeq, i);
        CvPoint pt1 = { r->x, r->y };
        CvPoint pt2 = { r->x + r->width, r->y + r->height };
        cvRectangle(frame, pt1, pt2, CV_RGB(255,0,0), 3, 4, 0);
        getFOV((float)r->width, (float)r->height, FOV_horizontal,FOV_vertical);
    }

    if(FOV_horizontal!=0.0 && FOV_vertical !=0.0)
    {
        RakNet::BitStream network_stream;
        network_stream.Write(static_cast<RakNet::MessageID>(ID_USER_PACKET_FACETRACKER));
        network_stream.Write(FOV_horizontal);
        network_stream.Write(FOV_vertical);
        peer->Send(&network_stream, LOW_PRIORITY, UNRELIABLE, 0, *address, false);
    }

    return true;
}

/**
 * @function getFOV
 */
void FaceTracker::getFOV(float face_w, float face_h, float& FOV_horizontal, float& FOV_vertical)
{
    float norm_face_w = (face_w - min_face_w) / (max_face_w - min_face_w); //-> [0;1]
    float real_world_dist = norm_face_w*(max_distance-min_distance) + min_distance;
    FOV_horizontal = 2*tan( (screen_w/2) / real_world_dist);

    float norm_face_h = (face_h - min_face_h) / (max_face_h - min_face_h); //-> [0;1]
    real_world_dist = norm_face_h*(max_distance-min_distance) + min_distance;
    FOV_vertical = 2*tan( (screen_h/2) / real_world_dist);
}
