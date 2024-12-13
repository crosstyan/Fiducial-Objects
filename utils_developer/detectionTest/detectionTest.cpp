/*~~~~~~~~~~~~~~~
    Includes
~~~~~~~~~~~~~~~*/
// Library includes
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <string>
#include <ctime>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
// Third parties includes
#include "../../3rdparty/cucomarker/cucomarker.h"
#include "../utils/utils.hpp"


/*~~~~~~~~~~~~~~~
    Namespaces
~~~~~~~~~~~~~~~*/
using namespace std;


/*~~~~~~~~~~~~~~~~~~~~~
    Global variables
~~~~~~~~~~~~~~~~~~~~~*/
TimerAvrg Fps;


// Sliders
// SHow candidates
    bool showCandidates = false;
    int showCandidatesInt = 0;
    void slider_showCandidates(int state, void* userData){
        showCandidates = !showCandidates;
        if(showCandidates)
            showCandidatesInt = 1;
        else    
            showCandidatesInt = 0;
    }
// thresholdingBlockSize
    int thresholdingBlockSize = 5;
    void slider_thresholdingBlockSize(int state, void* userData){
        thresholdingBlockSize = state;
    }
// thresholdingWindowSize
    int thresholdingWindowSize = 5;
    void slider_thresholdingWindowSize(int state, void* userData){
        if (state < 2) {
            state = 2;
        }
        thresholdingWindowSize = state;
    }
// areaThreshold
    int areaThreshold = 200;
    void slider_areaThreshold(int state, void* userData){
        areaThreshold = state;
    }


/*~~~~~~~~~~~~~~~
      Main
~~~~~~~~~~~~~~~*/
int main(int argc, char **argv)
{
    CmdLineParser cml(argc, argv);

    cout << "[live:<index>|videoFile] config.yml [-skip] [-c colorDetectionMode: ALL|HSV|GRAY (default ALL)] [-showCandidates]" << endl;

    string TheInputVideo = argv[1];
    cv::VideoCapture TheVideoCapturer;
    int waitTime=10;
    bool isVideo=false;

    ///////////  OPEN VIDEO
    ///
    // read from camera or from  file
    if (TheInputVideo.find("live") != string::npos)
    {
        int vIdx = 0;
        // check if the :idx is here
        char cad[100];
        if (TheInputVideo.find(":") != string::npos)
        {
            std::replace(TheInputVideo.begin(), TheInputVideo.end(), ':', ' ');
            sscanf(TheInputVideo.c_str(), "%s %d", cad, &vIdx);
        }
        cout << "Opening camera index " << vIdx << endl;
        TheVideoCapturer.open(vIdx);
        TheVideoCapturer.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
        TheVideoCapturer.set(cv::CAP_PROP_FRAME_WIDTH,1920);
        TheVideoCapturer.set(cv::CAP_PROP_FRAME_HEIGHT,1080);
        waitTime = 10;
        isVideo=true;
    }
    else{
        std::cout << "Opening video" << std::endl;
        TheVideoCapturer.open(TheInputVideo);
        if ( TheVideoCapturer.get(cv::CAP_PROP_FRAME_COUNT)>=2) isVideo=true;
        if(cml["-skip"])
            TheVideoCapturer.set(cv::CAP_PROP_POS_FRAMES,stoi(cml("-skip")));
    }
    // check video is open
    if (!TheVideoCapturer.isOpened())
        throw std::runtime_error("Could not open video");

    cv::Mat image;
    TheVideoCapturer >> image;

    cucomarker::MarkerDetector MDetector;
    MDetector.loadConfigFile(argv[2]);
    MDetector.setColorDetectionMode(cml("-c", "GRAY"));


    // Window Management
    std::string windowName = "Detection testing";
        cv::namedWindow(windowName);
    std::string configWindowName = "Settings";
        cv::namedWindow(configWindowName);
        cv::Mat settingsImage = cv::Mat::zeros(cv::Size(400, 300), CV_8UC3);
        cv::imshow(configWindowName, settingsImage);
    // Trackbars
        // Show candidates
        cv::createTrackbar("Show Candidates", configWindowName, &showCandidatesInt, 1, slider_showCandidates);
        // Adaptive Threshold Block Size
        thresholdingBlockSize = MDetector._adapThresBlockSize; // Get the value by default
        cv::createTrackbar("Adaptive threshold block size", configWindowName, &thresholdingBlockSize, 50, slider_thresholdingBlockSize);
        // thresholdingWindowSize
        thresholdingWindowSize = MDetector._adapThresWindowSize; // Get the value by default
        cv::createTrackbar("Adaptive threshold window size", configWindowName, &thresholdingWindowSize, 50, slider_thresholdingWindowSize);
        // areaThreshold
        areaThreshold = MDetector._areaThreshold; // Get the value by default
        cv::createTrackbar("Area threshold", configWindowName, &areaThreshold, 1000, slider_areaThreshold);
        

    // PROCESSING
    showCandidates=cml["-showCandidates"];
    char key = 0;
    int frameId;
    do{
        TheVideoCapturer.retrieve(image);
        frameId = TheVideoCapturer.get(cv::CAP_PROP_POS_FRAMES);


        // ---- Configure detector ----
        MDetector._adapThresBlockSize = thresholdingBlockSize;
        MDetector._adapThresWindowSize = thresholdingWindowSize;
        MDetector._areaThreshold = areaThreshold;
        // ----------------------------


        Fps.start();
        std::vector<cucomarker::Marker> markers=MDetector.detect(image);
        Fps.stop();

        std::cout << "\rFrame "<<frameId<<", time detection=" << Fps.getAvrg()*1000 << " ms, nmarkers=" << markers.size() <<" images resolution="<<image.size() <<std::endl;

        for(auto m:markers){
            m.draw(image, cv::Scalar(0,0,255));
            m.drawBits(image);
            std::cout<<"Marker detected, id: "<<m.id<<" | crc: "<<cucomarker::_private::crc_16(m.id)<<std::endl;
        }
        if(showCandidates)
            for(auto m:MDetector.getCandidates())
                m.draw(image, cv::Scalar(0,0,255));


        cv::imshow(windowName, image);

        key=cv::waitKey(waitTime);
        if (key == 's')
            waitTime = waitTime == 0 ? 10 : 0;
        if (key == 'w')
            cv::imwrite("image.jpg",image);

        if (isVideo)
            if ( TheVideoCapturer.grab()==false) key=27;


    }while(key!=27);

}