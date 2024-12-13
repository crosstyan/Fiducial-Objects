/*~~~~~~~~~~~~~~~
    Includes
~~~~~~~~~~~~~~~*/
// Library includes
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <cmath> 
#include <string>
#include <fstream>
#include <sstream>
#include <ctime>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <fstream>
#include <chrono>
// Internal includes
#include "../src/fiducialObject.h"
#include "utils/utils.cpp"
#include "utils/colors.hpp"


using namespace fiducialObject;

// Parameters
const cv::String keys =
"{help h usage ? | | Print this message}"
// Markers
"{@fiducialObjectPath  |<none>| Fiducial Object path. Default: none}"
// Video
"{@inputVideo  |<none>| Video to be processed}"
"{cameraParams |<none>| Camera parameters.}"
;

// Main
int main(int argc, char**argv){

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        Reading the parameters
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about("Fiducial object example application.");
    if(parser.has("help")){
        parser.printMessage();
        return 0;
    }
    if(argc < 3){
        parser.printMessage(); return -1;
    }

    // Parameters
    std::cout<<std::endl;
    std::cout<<BGREEN<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Parameters: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<RESET<<std::endl;
        
    std::string fiducialObjectPath = parser.get<std::string>("@fiducialObjectPath");
    std::string inputVideoPath = parser.get<std::string>("@inputVideo");
    std::string cameraParamsPath = "-"; if(parser.has("cameraParams")){ cameraParamsPath = parser.get<std::string>("cameraParams"); }
    std::cout<<GREEN<<"- Fiducial object path: "<<RESET<<fiducialObjectPath<<std::endl;
    std::cout<<GREEN<<"- Video path: "<<RESET<<inputVideoPath<<std::endl;
    std::cout<<GREEN<<"- Camera params path: "<<RESET<<cameraParamsPath<<std::endl;

    std::cout<<BGREEN<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<RESET<<std::endl;
    std::cout<<std::endl;

    /*~~~~~~~~~~~~~~~~~~~~~~
        Sources setting
    ~~~~~~~~~~~~~~~~~~~~~~*/
    // Input video
    cv::VideoCapture vcap(inputVideoPath);
    if(!vcap.isOpened()){
        std::cout<<"Cannot open the video file. \n";
        return -1;
    }

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        Fiducial object creation
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    Object fObj;
    fObj.loadFromFile(fiducialObjectPath);

    /*~~~~~~~~~~~~~~~~~~~~~~ 
        Compute K and D
    ~~~~~~~~~~~~~~~~~~~~~~*/
    auto fs = cv::FileStorage();
    cv::Mat K, D;
    fs.open(cameraParamsPath, cv::FileStorage::READ);
    fs["camera_matrix"] >> K;
    fs["distortion_coefficients"] >> D;
    fs.release();

    /*~~~~~~~~~~~~~~~~~~~~~~~~
        Video processing
    ~~~~~~~~~~~~~~~~~~~~~~~~*/
    cv::Mat im;
    while(vcap.grab()){
        vcap.retrieve(im);
        for(auto d: Detector::detect({fObj},im,K,D)){
            d.draw(im,K,D);
        }
        cv::imshow("image",im);
        cv::waitKey();
    }
    cv::destroyAllWindows();

    // End of execution
    return 0;
}
