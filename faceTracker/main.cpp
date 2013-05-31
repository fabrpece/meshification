#include <iostream>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>
#include "FaceTracker.hpp"

const char * DISPLAY_WINDOW = "Face Detection";
char* input_video;
bool live = true;
std::string address = std::string("127.0.0.1");
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
        else if(strcmp(argv[i],"--ip")==0)
        {
            ++i;
            address = std::string(argv[i]);
        }
    }
    return true;
}

/**
 * @function main
 */
int main( int argc, const char** argv )
{
    cv::VideoCapture cap;

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
    FaceTracker ft(address);

    while(frame.data)
    {
        //-- 2. Read the video stream
        IplImage c_frame = frame;
        cv::displayOverlay(DISPLAY_WINDOW, std::string("Connection: ") + (ft(&c_frame) ? "Estalbished" : "OFF"), 1000);
        cv::imshow(DISPLAY_WINDOW, frame);
        int c = cv::waitKey(100);
        if( (char)c == 27  || (char)c == 'q')
          break;

        cap >> frame;

    }
    return 0;

}
