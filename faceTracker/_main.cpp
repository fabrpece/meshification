#include <iostream>
#include <sstream>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "../common/AsyncWorker.hpp"
#include "../common/PacketID.hpp"

using namespace std;
using namespace cv;


/** Function Headers */
void calibrate(IplImage* frame );
void detectAndDisplay(IplImage* frame );
bool parseArgs(int argc, const char** argv);

/** Global variables */
CvHaarClassifierCascade* face_cascade = 0;
CvMemStorage* pStorageface = 0;
RNG rng(12345);
bool live = true;
char* input_video;
bool do_calibration = false;
const char * DISPLAY_WINDOW = "Face Detection";

int min_face_w = 110, min_face_h = 110;//int min_face_w = 102, min_face_h = 102;
int max_face_w = 320, max_face_h = 320;//int max_face_w = 205, max_face_h = 205;
float tablet_screen_w = 255.2, tablet_screen_h = 144.1;
float max_distance = 1000.0, min_distance = 200;

/*

  for each face_w, I can compute

    norm_face_w = (face_w-min_face_w)/(max_face_w-min_face_w) //-> [0;1]
    real_world_dist = norm_face_w*(max_distance-min-distance) + min_distance
    FOV_horizontal = 2*tan( (tablet_screen_w/2) / real_world_dist)

    same for vertical.

*/
/**
 * @function getFOV
 */
void getFOV(float face_w, float face_h, float& FOV_horizontal, float& FOV_vertical)
{
    float norm_face_w = (face_w - min_face_w) / (max_face_w - min_face_w); //-> [0;1]
    float real_world_dist = norm_face_w*(max_distance-min_distance) + min_distance;
    FOV_horizontal = 2*tan( (tablet_screen_w/2) / real_world_dist);

    float norm_face_h = (face_h - min_face_h) / (max_face_h - min_face_h); //-> [0;1]
    real_world_dist = norm_face_h*(max_distance-min_distance) + min_distance;
    FOV_vertical = 2*tan( (tablet_screen_h/2) / real_world_dist);
}

/**
 * @function calibrate
 */
void calibrate(IplImage* frame )
{

   CvSeq * pFaceRectSeq;               // memory-access interface
   pStorageface = cvCreateMemStorage(0);

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

        if( min_face_w > r->width &&
            min_face_h > r->height)
        {
            min_face_w = r->width;
            min_face_h = r->height;
        }
        else if( max_face_w < r->width &&
                 max_face_h < r->height)
        {
            max_face_w = r->width;
            max_face_h = r->height;
        }
    }

    std::stringstream ss1,ss2;
    ss1 << "Min face [" << min_face_w << ";" << min_face_h << "] " << std::flush;
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.4, 0.4, 0, 1, 8);
    cvPutText(frame, ss1.str().c_str(), cvPoint(10,10), &font, cvScalar(255, 255, 255, 0));
    ss2 << "Max face [" << max_face_w << ";" << max_face_h << "] " << std::flush;
    cvPutText(frame, ss2.str().c_str(), cvPoint(10,25), &font, cvScalar(255, 255, 255, 0));

    // display face detections
    cvShowImage(DISPLAY_WINDOW, frame);
}

/**
 * @function detectAndDisplay
 */
void detectAndDisplay(IplImage* frame )
{

   CvSeq * pFaceRectSeq;               // memory-access interface

   pStorageface = cvCreateMemStorage(0);
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


   std::stringstream ss;
   CvFont font;
   cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.4, 0.4, 0, 1, 8);
   ss << "H FOV: " << FOV_horizontal << " - V FOV " << FOV_vertical << std::flush;
   cvPutText(frame, ss.str().c_str(), cvPoint(10,35), &font, cvScalar(255, 255, 255, 0));

   // display face detections
    cvShowImage(DISPLAY_WINDOW, frame);

}

/**
 * @function parseArgsint
 */
bool parseArgs(int argc, const char** argv)
{
    for(int i=1; i<argc; ++i)
    {
        if(strcmp(argv[i],"-i")==0)
        {
            ++i;
            input_video = (char*)argv[i];
            live = false;
        }
        else if(strcmp(argv[i],"--calibrate")==0)
        {
            do_calibration = true;
            min_face_w = 1000;
            min_face_h = 1000;
            max_face_w = 0;
            max_face_h = 0;
        }
    }
    return true;
}

/**
 * @function main
 */
int main( int argc, const char** argv )
{
    VideoCapture cap;

    if(!parseArgs(argc,argv))
            return 1;
    if(live)
        cap.open(1);
    else
        cap.open(input_video);

    if(!cap.isOpened())
    {
        std::cerr << "Cannot open input stream." << std::endl;
        return 1;
    }

    cv::Mat frame;
    cap >> frame;
    if(!frame.data)
    {
        std::cerr << "Error reading innput stream." << std::endl;
        return 1;
    }

    // create a window to display detected faces
    cvNamedWindow(DISPLAY_WINDOW, CV_WINDOW_AUTOSIZE);
    //-- 1. Load the cascade
    face_cascade = (CvHaarClassifierCascade *)cvLoad("/home/fabrizio/Development/Libraries/OpenCV-2.4.3/data/haarcascades/haarcascade_frontalface_alt_tree.xml", 0 , 0, 0);

    while(frame.data)
    {
        //-- 2. Read the video stream
        IplImage c_frame = frame;

        //-- 3. Apply the classifier to the frame
        if(do_calibration)
            calibrate(&c_frame);
        else
            detectAndDisplay(&c_frame);

        int c = waitKey(100);

        if( (char)c == 27  || (char)c == 'q')
          break;

        cap >> frame;

    }

    // clean up and release resources
    if(face_cascade)
        cvReleaseHaarClassifierCascade(&face_cascade);
    if(pStorageface)
        cvReleaseMemStorage(&pStorageface);

    return 0;

}
