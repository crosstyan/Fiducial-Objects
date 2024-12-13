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
#include "../src/polyhuco/polyhuco.h"
#include "../utils/utils.hpp"
#include "colors.hpp"


// printAlert
void printAlert(cv::String message){
    std::cout<<BRED<<"[!] "+message+" [!]"<<RESET<<std::endl;
}

using namespace std;

// Parameters
const cv::String keys =
"{help h usage ? | | Print this message}"
// Verboses
"{v | | Activate verbose mode. Default: Deactivated.}"
"{vdebug | | Activate verbose debug mode. Default: Deactivated.}"
"{vframe | | Activate verbose frame mode. Default: Deactivated.}"
// Minimal options
"{minimal | | Activate minimal mode. Default: Deactivated.}"
"{noTerminal | | Deactivate terminal printing. Default: Activated.}"
// Detection mode
"{detectionMode |1| Detection mode. 1: Cucomarker. 2: Aruco}"
// Input
"{inputImage         |<none>| Image to be processed}"
"{inputVideo         |<none>| Video to be processed}"
// Mandatory params
"{@cameraParams |<none>| Camera parameters.}"
"{@markerSize |<none>| Real size of the marker.}"
"{@mapPath |<none>| Path for the map.}"
// Marker params
"{markerParams        |<none>| Markers parameters. Default: none.yml}"
"{idsRange | | Defines the id range, in order to determine the id looking for range, and can be like '1-7'. Default: none. }"
// Outputs
    // Video
    "{outputVideoPath |output.avi| Path for the output video. Default: output.avi. Recommended format: avi.}"
    "{noOutputVideo | | Generate output video. Default: yes.}"
    // Poses file
    "{outputPosePath |poses.txt| Output filename of pose path}"
    "{generatePosesOutputFile | | Generate poses output file. Default: Deactivated.}"
    // Distances file
    "{outputDistancePath |distances.txt| Define the distances output path. Default: none. }"
    "{generateDistancesOutputFile | | Generate distances output file. Default: Deactivated.}"
// Thresholds
"{thresNDet |0| Threshold for the minimun number of detected markers to count as a valid pose.}"
"{thresArea |0| Threshold for the minimun average computed area to count as a valid pose.}"
"{thresAngle |0| Threshold for the minimun average computed angle to count as a valid pose.}"
"{thresDiff |0| Threshold for the minimun average computed angle to count as a valid pose.}"
// Refinement
"{applyRefinement | | Applies refinement since beginning of execution. Starting with look for markers = 1. Default: Deactivated.}"
"{lookForMarkers |<none>| Establishes look for markers value. Default: None.}"
"{checkCandidates |<none>| Establishes the mandatory checking with candidates for the looked markers. Default: Deactivated.}"
    // Options
    "{subPixSize |3| Size of the kernel of cornerSubPix. Default: 3 .}"
    "{subPixRep |12| Number of repetitions of cornerSubPix. Default: 12 .}"
    "{subPixEpsilon |0.005| Value of Epsilon for cornerSubPiex. Default: 0.005 .}"
// Pose estimation
"{solvePNPMethod |0| Defines the method to compute the pose of the polyhuco. 0: ITERATIVE, 1: EPNP, 2: P3P, 3: DLS, 4: UPNP, 5: AP3P, 6: IPPE, 7: IPPE_SQUARE, 8: SQPNP, 9: MAX_COUNT. Default: 0.}" 
// Valid frames
"{framesRanges | | Defines the frames range to process. It can be like '1,3-8,13' or like 'frames.txt' where the content of the file is the same changing the , for new line character. Default: none. }"
// Scaling for map
"{scaleForMap |1| ... .}"
// Pause options
"{startingPaused | | Start the processing in a pause, and with the help extended. }"
// Printing on image information
"{lineStartingShift |0| Line starting point. By default the first possible line. }"
;
// Main
int main(int argc, char**argv){
    /*~~~~~~~~~~~~~~
        Styles
    ~~~~~~~~~~~~~~*/
    cv::Scalar frameTextColor = cv::Scalar(88,2,237);


    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        Reading the parameters
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about("Developer Fiducial object example application.");
    if(parser.has("help")){
        parser.printMessage();
        return 0;
    }
    if(argc < 3){
        parser.printMessage(); return -1;
    }


    // No terminal
    bool terminal=true; if(parser.has("noTerminal")){terminal=false;}
    if(!terminal){
        streambuf* orig_buf = cout.rdbuf();
        cout.rdbuf(NULL);
    }
    // Parameters
    std::cout<<std::endl;
    std::cout<<BGREEN<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Parameters: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<RESET<<std::endl;
    // Verboses
    bool verbose=false; if(parser.has("v")){verbose=true;}
    bool verboseDebug=false; if(parser.has("vdebug")){verboseDebug=true;}
    bool verboseFrame=false; if(parser.has("vframe")){verboseFrame=true;}
    std::cout<<CYAN<<"- Verbose: "<<RESET<<verbose<<std::endl;
    std::cout<<CYAN<<"- Verbose debug: "<<RESET<<verboseDebug<<std::endl;
    std::cout<<CYAN<<"- Verbose frame: "<<RESET<<verboseFrame<<std::endl;
    // Minimal 
    bool minimal=false; if(parser.has("minimal")){minimal=true;}
    std::cout<<CYAN<<"- Minimal: "<<RESET<<minimal<<std::endl;
    std::cout<<CYAN<<"- Terminal: "<<RESET<<terminal<<std::endl;
    if(minimal){
        verbose = false;
        verboseDebug = false;
        verboseFrame = false;
    }
    // Detection mode
    int detectionMode = parser.get<int>("detectionMode");
    std::cout<<MAGENTA<<"- Detection mode: "<<RESET<<detectionMode<<std::endl;
    // Others
    bool isImage=false; std::string inputImagePath = ""; if(parser.has("inputImage")){ inputImagePath = parser.get<std::string>("inputImage"); isImage=true; };
    bool isVideo=false; std::string inputVideoPath = ""; if(parser.has("inputVideo")){ inputVideoPath = parser.get<std::string>("inputVideo"); isVideo=true; };
    auto cameraParamsPath = parser.get<std::string>("@cameraParams");
    double markerSize = parser.get<double>("@markerSize");
    cv::String mapPath = parser.get<std::string>("@mapPath");
    auto markersParamsPath = parser.get<std::string>("markerParams");
    std::cout<<GREEN<<"- Image path: "<<RESET<<inputImagePath<<std::endl;
    std::cout<<GREEN<<"- Video path: "<<RESET<<inputVideoPath<<std::endl;
    std::cout<<GREEN<<"- Camera params path: "<<RESET<<cameraParamsPath<<std::endl;
    std::cout<<GREEN<<"- Marker size: "<<RESET<<markerSize<<std::endl;
    std::cout<<GREEN<<"- Map path: "<<RESET<<mapPath<<std::endl;
    std::cout<<GREEN<<"- Markers params path: "<<RESET<<markersParamsPath<<std::endl;
    // Output
        // Video
        auto outputVideoPath = parser.get<std::string>("outputVideoPath");
        bool noOutputVideo = false; if(parser.has("noOutputVideo")){noOutputVideo=true;}
        std::cout<<GREEN<<"- Output video path: "<<RESET<<outputVideoPath<<std::endl;
        std::cout<<GREEN<<"\t- Generate output video?: "<<RESET<<!noOutputVideo<<std::endl;
        // Poses
        cv::String outputPosePath = parser.get<std::string>("outputPosePath");
        bool generatePosesOutputFile=false; if(parser.has("generatePosesOutputFile")){generatePosesOutputFile=true;}
            // Exists?
            if(generatePosesOutputFile){
                if(std::ifstream(outputPosePath).good()){
                    system(("rm "+outputPosePath).c_str());
                }
                system(("touch "+outputPosePath).c_str());
            }
        std::cout<<GREEN<<"- Output poses path: "<<RESET<<outputPosePath<<std::endl;
        std::cout<<GREEN<<"\t- Generate output poses file?: "<<RESET<<generatePosesOutputFile<<std::endl;
        // Distances
        cv::String outputDistancePath = parser.get<std::string>("outputDistancePath");
        bool generateDistancesOutputFile=false; if(parser.has("generateDistancesOutputFile")){generateDistancesOutputFile=true;}
            // Exists?
            if(generateDistancesOutputFile){
                if(std::ifstream(outputDistancePath).good()){
                    system(("rm "+outputDistancePath).c_str());
                }
                system(("touch "+outputDistancePath).c_str());
            }
        std::cout<<GREEN<<"- Output distances path: "<<RESET<<outputDistancePath<<std::endl;
        std::cout<<GREEN<<"\t- Generate output distances file?: "<<RESET<<generateDistancesOutputFile<<std::endl;
    // Refinement
    int lookForMarkers = 0;
    bool applyRefinement = false;
    if(parser.has("applyRefinement")){
        applyRefinement = true;
        lookForMarkers = 1;
    }
    std::cout<<CYAN<<"- Apply refinement: "<<RESET<<applyRefinement<<std::endl;
    // Look for markers
    if(parser.has("lookForMarkers")){
        lookForMarkers = parser.get<int>("lookForMarkers");
    }
    std::cout<<MAGENTA<<"- Look for markers: "<<RESET<<lookForMarkers<<std::endl;
    // checkCandidates
    bool checkCandidates = false;
    if(parser.has("checkCandidates")){
        checkCandidates = true;
    }
    std::cout<<CYAN<<"- Check candidates: "<<RESET<<checkCandidates<<std::endl;
    // Thresholds
    double thresNDet = parser.get<double>("thresNDet");
    double thresArea = parser.get<double>("thresArea");
    double thresAngle = parser.get<double>("thresAngle");
    double thresDiff = parser.get<double>("thresDiff");
    std::cout<<BLUE<<"- Threshold minimum num of detections: "<<RESET<<thresNDet<<std::endl;
    std::cout<<BLUE<<"- Threshold minimun average area: "<<RESET<<thresArea<<std::endl;
    std::cout<<BLUE<<"- Threshold minimum average angle: "<<RESET<<thresAngle<<std::endl;
    std::cout<<BLUE<<"- Threshold maximun difference between poses: "<<RESET<<thresDiff<<std::endl;
    // Pose computation option
    int solvePNPMethod = parser.get<int>("solvePNPMethod");
    std::cout<<BLUE<<"- Solve PNP Method: "<<RESET<<solvePNPMethod<<std::endl;
    // CornerSubPix options
    double subPixSize = parser.get<double>("subPixSize");
    double subPixRep = parser.get<double>("subPixRep");
    double subPixEpsilon = parser.get<double>("subPixEpsilon");
    std::cout<<BLUE<<"- CornerSubPix subPixSize: "<<RESET<<subPixSize<<std::endl;
    std::cout<<BLUE<<"- CornerSubPix subPixRep: "<<RESET<<subPixRep<<std::endl;
    std::cout<<BLUE<<"- CornerSubPix subPixEpsilon: "<<RESET<<subPixEpsilon<<std::endl;
    // Id range
    int id_min = -1;
    int id_max = -1;
    if(parser.has("idsRange")){
        std::string range = parser.get<std::string>("idsRange");
        int pos = -1;
        for(int spos = 0; spos < range.length(); spos++){
            if(range[spos] == '-'){
                pos = spos;
                break;
            }
        }
        if(pos != -1){
            id_min = std::stoi(  range.substr(0, pos)  );
            id_max = std::stoi(  range.substr(pos+1, range.length())  );
        }
    }
    std::cout<<GREEN<<"- Id min: "<<RESET<<id_min<<std::endl;
    std::cout<<GREEN<<"- Id max: "<<RESET<<id_max<<std::endl;
    // Frames range
    std::vector<int> valid_frames;
    if(parser.has("framesRanges")){
        std::string range = parser.get<std::string>("framesRanges");
        std::vector<std::string> splits;
        // File content or directly parametrised
        FILE *file = fopen(range.c_str(), "r");
        if(!file){ // PARAMETRISED CONTENT
            // Different ranges or values?
            std::vector<int> commas;
            commas.push_back(-1);
            for(int spos = 0; spos < range.length(); spos++){
                if(range[spos] == ','){
                    commas.push_back(spos);
                }
            }
            // String substrs
            commas.push_back(range.length());
            for(int c = 0; c < (commas.size()-1); c++){
                std::cout<<"pos: "<<c<<" |"<<commas[c]+1<<" - "<<commas[c+1]<<std::endl;
                std::string split = range.substr(commas[c]+1, commas[c+1]-commas[c]-1); splits.push_back(split);
            }
        }
        else{ // FILE CONTENT
            std::ifstream framesFile(range);
            std::string line;
            while(std::getline(framesFile, line)){
                if(line != "\n"){
                    line.erase(remove(line.begin(), line.end(), '\n'), line.end());
                    splits.push_back(line);
                }
            }
        }
        // Valid frames processing
        std::cout<<std::endl;
        for(int s = 0; s < splits.size(); s++){
            int pos = -1;
            for(int sc = 0; sc < splits[s].size(); sc++){
                if(splits[s][sc] == '-'){
                    pos = sc;
                }
            }
            if(pos == -1){ // No range
                int v = std::stoi(splits[s]); 
                valid_frames.push_back(v);
            }
            else{ // Range
                int min = std::stoi( splits[s].substr(0, pos) );
                int max = std::stoi( splits[s].substr(pos+1, splits[s].length()) );
                for(int v = min; v <= max; v++){
                    valid_frames.push_back(v);
                }
            }
        }
    }
    // Scale to map
    double scaleForMap = parser.get<double>("scaleForMap");
    std::cout<<MAGENTA<<"- Scale for map: "<<RESET<<scaleForMap<<std::endl;
    // Refinement
    bool showUserInputsMessages = false;
    bool startingPaused = false;
    if(parser.has("startingPaused")){
        startingPaused = true;
        showUserInputsMessages = true;
    }
    std::cout<<MAGENTA<<"- Starting paused: "<<RESET<<startingPaused<<std::endl;
    // lineStartingShift
    int lineStartingShift = parser.get<int>("lineStartingShift");
    std::cout<<GREEN<<"- Line starting point: "<<RESET<<lineStartingShift<<std::endl;
    std::cout<<BGREEN<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<RESET<<std::endl<<std::endl;
    if(!minimal) cv::waitKey(0);


    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        Polyhuco configuration
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    std::string polyhucoName = "Polyhuco figure";
    std::string markersTypeName = "Cucomarker";
    polyhuco::Polyhuco polyhucoProcessor = polyhuco::Polyhuco(polyhucoName,
        12,
        markersTypeName,
        markerSize,
        detectionMode,
        markersParamsPath,
        mapPath,
        verbose,
        verboseDebug,
        verboseFrame,
        solvePNPMethod,
        subPixSize,
        subPixRep,
        subPixEpsilon,
        id_min,
        id_max,
        outputDistancePath,
        scaleForMap,
        checkCandidates
    );
    polyhucoProcessor.setThresAngle(thresAngle);
    polyhucoProcessor.setThresNDet(thresNDet);
    polyhucoProcessor.setThresArea(thresArea);
    polyhucoProcessor.setThresDiff(thresDiff);



    if(isImage){
        cv::Mat inputImage = cv::imread(inputImagePath);


        TimerAvrg Fps;
        polyhuco::foundInFrame foundInformation = polyhucoProcessor.findInFrame(inputImage, cameraParamsPath, 1, lookForMarkers, false);
        polyhuco::foundInFrame foundInformationRef = polyhucoProcessor.findInFrame(inputImage, cameraParamsPath, 1, lookForMarkers, true);
//        polyhuco::foundInFrame foundInformationRef;
//        Fps.start();
//        double repetitions = 1;
//        for(int r = 0; r < repetitions; r++){
//            foundInformationRef = polyhucoProcessor.findInFrame(inputImage, cameraParamsPath, 1, lookForMarkers, true);
//        }
//        Fps.stop();
//        std::cout<<"Process time: " << (Fps.getAvrg()*1000)/repetitions << " ms"<<std::endl;

        cv::Mat outputImage = foundInformation.outputFrame.clone();
        cv::Mat outputImageRef = foundInformationRef.outputFrame.clone();
        if(!minimal) cv::imshow("Input image", inputImage);
        if(!minimal) cv::imshow("Processed image", outputImage);
        if(!minimal) cv::imshow("Refined Processed image", outputImageRef);
        if(!minimal) cv::waitKey();

    }
    else if(isVideo){
        /*~~~~~~~~~~~~~~~~~~~~~~
            Sources setting
        ~~~~~~~~~~~~~~~~~~~~~~*/
        // Input video
        cv::VideoCapture cap(inputVideoPath);
        if(!cap.isOpened()){
            std::cout<<"Cannot open the video file. \n";
            return -1;
        }
        int videoWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH);
        int videoHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        cv::Size frameSize = cv::Size(videoWidth, videoHeight);



        /*~~~~~~~~~~~~~~~~~~~
            Output files
        ~~~~~~~~~~~~~~~~~~~*/
        // Poses output file
        ofstream posesFile;
        if(generatePosesOutputFile){
            posesFile.open(outputPosePath);
        }
        // Distances output file
        ofstream distancesFile;
        if(generateDistancesOutputFile){
            distancesFile.open(outputDistancePath);
        }



        /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            Generate output video preparation
        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
        cv::VideoWriter outputVideo;
        if(!noOutputVideo){
            outputVideo.open(outputVideoPath, cv::VideoWriter::fourcc('P', 'I', 'M', '1'), cap.get(cv::CAP_PROP_FPS), frameSize, true);
        }



        /*~~~~~~~~~~~~~~~~~~~~~~~~~~~
            Processing the video
        ~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
        double zoom = 100;
        int processed_frames = 0;
        TimerAvrg detectionFPS, systemFPS;
        double valueSystemFPS = 0.0;
        std::vector<cv::Mat> outputFrames;
        bool continueProcessing = true;
        int detectionType = detectionMode; cv::String detectionTypeNames[3] = {"None", "Cucomarker", "Aruco"};
        int drawingType = 1; cv::String drawingTypeNames[5] = {"No drawing", "Minimal drawing", "3D drawing (Local pose)", "Normal drawing (global pose)", "3D drawing (global pose)"};
        bool skippedFrame = false;
        cv::String lookForMarkersNames[4] = {"None", "Only good", "Both good and bad", "3D Only good"};
        if(!minimal) cv::namedWindow("Processed frame");
        while(continueProcessing){
            systemFPS.start();

            /*~~~~~~~~~~~~~~~~~~~~
                Frame obtaining
            ~~~~~~~~~~~~~~~~~~~~*/
            // Obtaining
            cv::Mat frame;
            if (!cap.read(frame)){
                std::cout<<"\n Cannot read a new frame of the video\n";
                break;
            }
            // Print information on terminal
            if(!minimal){
                std::cout<<std::endl<<BGREEN<<"~+~~~~~~~~~~~~~~~~~~~~ Frame: "<<processed_frames<<" ~~~~~~~~~~~~~~~~~~~~+~"<<RESET<<std::endl;
            }
            else{
                std::cout<<"Processing frame "<<processed_frames<<"..."<<std::endl;
            }
            // Print frame identification on frame
            int step=15;
            int linePoint=15; for(int i = 0; i < lineStartingShift; i++){linePoint += step;}
            cv::putText(frame, "Frame: "+std::to_string(processed_frames), cv::Point2f(5,linePoint), cv::FONT_HERSHEY_PLAIN, 1, frameTextColor, 1, false); linePoint+=step;
            // Print zoom information on frame
            if(verboseFrame){
                cv::putText(frame, "| Zoom: "+std::to_string(zoom)+" %", cv::Point2f(110,linePoint-step), cv::FONT_HERSHEY_PLAIN, 1, frameTextColor, 1, false);
            }


            /*~~~~~~~~~~~~~~~~~~~~~~~~~~
                Polyhuco processing
            ~~~~~~~~~~~~~~~~~~~~~~~~~~*/
            detectionFPS.start();
            cv::Mat processedFrame = frame.clone();
            polyhuco::foundInFrame foundInformation = polyhucoProcessor.findInFrame(processedFrame, cameraParamsPath, drawingType, lookForMarkers, applyRefinement); 
            std::cout<<"LookingMarkers: "<<lookForMarkers<<std::endl;
            std::cout<<"ApplyRefinement: "<<applyRefinement<<std::endl;
            
            
            std::vector<int> markersId = foundInformation.detectionInfo.foundMarkersIds;
            processedFrame = foundInformation.outputFrame.clone();
            detectionFPS.stop();


            /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                Printing some frame information
            ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
            // Print detection information
            double detectionTime = detectionFPS.getAvrg()*1000;
            if(verbose) std::cout<<"> Detection type: "<<detectionType<<" < ("<<detectionTypeNames[detectionType]<<")"<<std::endl;
            if(verbose) std::cout<<"> Drawing type: "<<drawingType<<" < ("<<drawingTypeNames[drawingType]<<")"<<std::endl;
            if(verbose) std::cout<<"> Detection fps: "<<detectionTime<<" |System fps: "<<valueSystemFPS<<std::endl;
            if(verbose) std::cout<<"> Nº of detected markers: "<<markersId.size()<<std::endl;
            if(verbose) std::cout<<"> Looking for markers: "<<lookForMarkers<<" < ("+lookForMarkersNames[lookForMarkers]+")"<<std::endl;
            if(verbose) std::cout<<"> Apply refinement: "<<applyRefinement<<std::endl;
            if(verboseFrame){
                cv::putText(processedFrame, "Detection type: "+std::to_string(detectionType)+" < ("+detectionTypeNames[detectionType]+")", cv::Point2f(5,linePoint), cv::FONT_HERSHEY_PLAIN, 1, frameTextColor, 1, false); linePoint+=step;
                cv::putText(processedFrame, "Drawing type: "+std::to_string(drawingType)+" < ("+drawingTypeNames[drawingType]+")", cv::Point2f(5,linePoint), cv::FONT_HERSHEY_PLAIN, 1, frameTextColor, 1, false); linePoint+=step;
                cv::putText(processedFrame, "Detection fps: "+std::to_string(detectionTime)+" |System fps: "+std::to_string(valueSystemFPS), cv::Point2f(5,linePoint), cv::FONT_HERSHEY_PLAIN, 1, frameTextColor, 1, false); linePoint+=step;
                cv::putText(processedFrame, "N. of detected markers: "+std::to_string(markersId.size()), cv::Point2f(5,linePoint), cv::FONT_HERSHEY_PLAIN, 1, frameTextColor, 1, false); linePoint+=step;
                cv::putText(processedFrame, "Looking for markers: "+std::to_string(lookForMarkers)+" < ("+lookForMarkersNames[lookForMarkers]+")", cv::Point2f(5,linePoint), cv::FONT_HERSHEY_PLAIN, 1, frameTextColor, 1, false); linePoint+=step;
                cv::putText(processedFrame, "Apply refinement: "+std::to_string(applyRefinement), cv::Point2f(5,linePoint), cv::FONT_HERSHEY_PLAIN, 1, frameTextColor, 1, false); linePoint+=step;
            }



            // Print detected markers ids
            std::string markersIdsString = "-";
            for(int i = 0; i < markersId.size(); i++){
                if (i == 0)
                    markersIdsString = "[ ";
                markersIdsString += std::to_string(markersId[i]);
                if (i != (markersId.size()-1))
                    markersIdsString += " | ";
                else
                    markersIdsString += " ]";
            }
            if(verbose) std::cout<<"> Detected markers ids: "<<markersIdsString<<std::endl;
            if(verboseFrame){
                cv::putText(processedFrame, "Detected markers ids: "+markersIdsString, cv::Point2f(5,linePoint), cv::FONT_HERSHEY_PLAIN, 1, frameTextColor, 1, false); linePoint+=step;
            }


            /*~~~~~~~~~~~~~~~~~~~~~~
                Pose validation
            ~~~~~~~~~~~~~~~~~~~~~~*/
            if(foundInformation.found){                     

                /*~~~~~~~~~~~~~~~~~~~~~~~~~~
                    Output managing
                ~~~~~~~~~~~~~~~~~~~~~~~~~~*/
                // Condition to check if the frame id is a valid frame (Included in the valid frames ranges set by parameter. By default: All)
                bool validFrameId = (valid_frames.size() == 0);
                if(validFrameId == false){
                    for(int vf = 0; vf < valid_frames.size(); vf++){
                        if(processed_frames == valid_frames[vf]){
                            validFrameId = true;
                        }
                    }
                }
                // Poses approving
                bool approved_nonRefinedInfo = foundInformation.detectionInfo.woRefinement.poseApproved;
                bool approved_RefinedInfo = false; if(applyRefinement){ approved_RefinedInfo = foundInformation.detectionInfo.wRefinement.poseApproved; }
                std::cout<<"Refined approved? "<<approved_RefinedInfo<<std::endl;
                std::cout<<"nonRefined approved? "<<approved_nonRefinedInfo<<std::endl;
                // Extracting the selected pose: [refined, non-refined, none]
                if(validFrameId){
                    std::cout<<"Managing outputs..."<<std::endl;
                    cv::Mat tvec; 
                    cv::Mat rvec;
                    // There is a valid pose?
                    std::string valid_pose_id = "None";
                    if(foundInformation.detectionInfo.poseIsSelected){ // Approved refined pose?
                        valid_pose_id = foundInformation.detectionInfo.selectedPoseId;
                        std::cout<<BGREEN<<"[ "<<Symbols_Check<<GREEN<<" Approved the pose: "<<BGREEN<<valid_pose_id<<" ]"<<RESET<<std::endl;
                        tvec = foundInformation.detectionInfo.selectedPose.tvec.clone();
                        rvec = foundInformation.detectionInfo.selectedPose.rvec.clone();
                    }
                    else{
                        std::cout<<BRED<<"[ "<<Symbols_Cross<<" No pose was approved ]"<<RESET<<std::endl;
                    }
                    // Write valid pose if there is any
                    if( valid_pose_id != "None" ){
                        std::cout<<GREEN<<"\tWriting in output the pose: "<<RESET<<valid_pose_id<<std::endl;
                        if(verboseDebug) std::cout<<"Tvec: "<<tvec<<std::endl;
                        if(verboseDebug) std::cout<<"Rvec: "<<rvec<<std::endl;
                        /*~~~~~~~~~~~~~~~~~~~~~~~~~
                            Writing pose file
                        ~~~~~~~~~~~~~~~~~~~~~~~~~*/
                        if(generatePosesOutputFile){ // Poses output file?
                            //Compute quaternions
                            std::string poseLine = getPoseLine(processed_frames, rvec, tvec, verboseDebug);
                            posesFile << poseLine;  
                            if(verbose) std::cout<<BOLD<<"\t\t "<<Symbols_Check<<" Pose line written: "<<RESET<<poseLine;
                        }
                        /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                            Writing distances file
                        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
                        if(generateDistancesOutputFile){ // Distances output file?
                            // Computing
                            double distanceToObject = cv::norm(tvec, cv::NORM_L2);
                                // Not used -> double angleToObject = cv::norm(rvec, cv::NORM_L2);
                            std::string distanceStr = std::to_string(distanceToObject);
                            // Writing
                            std::string distanceLine = distanceStr+"\n";
                            distancesFile << distanceLine;
                            if(verbose) std::cout<<BOLD<<"\t\t "<<Symbols_Check<<" Distance line written: "<<RESET<<distanceLine;
                        }
                    }
                    else{
                        std::cout<<RED<<"\tNor the woRefinement and wRefinement poses have been approved"<<RESET<<std::endl;
                        std::cout<<RED<<"\t\t- No pose in output file and no distance in output file"<<RESET<<std::endl;
                    }
                }
                std::cout<<"\n\n"<<std::endl;
            }



            /*~~~~~~~~~~~~~~~~~~~~~~~~~~
                User help managing
            ~~~~~~~~~~~~~~~~~~~~~~~~~~*/
            if(verboseFrame){
                cv::putText(processedFrame, "Pulse 'h' to show/hide help", cv::Point2f(5,linePoint), cv::FONT_HERSHEY_PLAIN, 1, frameTextColor, 1, false); linePoint+=step;
            }
            // User inputs
            std::vector<cv::String> userInputsOptions;
            userInputsOptions.push_back("User input options: ");
            userInputsOptions.push_back("   Escape: Finishes the execution");
            userInputsOptions.push_back("   d: Changes detection mode");
            userInputsOptions.push_back("      - 1: "+detectionTypeNames[1]);
            userInputsOptions.push_back("      - 2: "+detectionTypeNames[2]);
            userInputsOptions.push_back("   3: Changes drawing type");
            userInputsOptions.push_back("      - 0: "+drawingTypeNames[0]);
            userInputsOptions.push_back("      - 1: "+drawingTypeNames[1]);
            userInputsOptions.push_back("      - 2: "+drawingTypeNames[2]);
            userInputsOptions.push_back("      - 3: "+drawingTypeNames[3]);
            userInputsOptions.push_back("      - 4: "+drawingTypeNames[4]);
            userInputsOptions.push_back("   r: Activates/Deactivates applying of looked markers");
            userInputsOptions.push_back("   s: Takes screenshot");
            userInputsOptions.push_back("   v: Activates/Deactivates verbose frame");
            userInputsOptions.push_back("   -: Zooms out");
            userInputsOptions.push_back("   +: Zooms in");
            userInputsOptions.push_back("   p: Pauses/Resumes processing");
            userInputsOptions.push_back("   n: Skips to the next frame (Only in pause mode)");
            userInputsOptions.push_back("   l: Looks for some good markers");
            userInputsOptions.push_back("      - 0: "+lookForMarkersNames[0]);
            userInputsOptions.push_back("      - 1: "+lookForMarkersNames[1]);
            userInputsOptions.push_back("      - 2: "+lookForMarkersNames[2]);
            userInputsOptions.push_back("      - 3: "+lookForMarkersNames[3]);
            userInputsOptions.push_back("   h: Shows/Hides help");
            // Print options on frame
            if(verboseFrame && showUserInputsMessages){
                for(int o = 0; o < userInputsOptions.size(); o++){
                    cv::putText(processedFrame, userInputsOptions[o], cv::Point2f(5,linePoint), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(245, 200, 66), 1, false); linePoint+=step;
                }
            }


            // Scale the image
            cv::Mat zoomedProcessedFrame;
            cv::resize(processedFrame, zoomedProcessedFrame, cv::Size(processedFrame.cols * (zoom/100.0), processedFrame.rows * (zoom/100.0)));

            // Show the results
            if(!minimal){
                cv::imshow("Processed frame", zoomedProcessedFrame);
            }

            // User input
            int key = 0;
            if(!minimal) key = cv::waitKey(1);
            // if((processed_frames == 1) && (startingPaused)){
            //     key == 112;
            // }
            // [ Escape ]
            if(key == 27){
                break;
            }
            // [ d ]
            if(key == 100){
                if(detectionType == 1){
                    detectionType = 2;
                }
                else if(detectionType == 2){
                    detectionType = 1;
                }
            }
            // [ 3 ]
            if(key == 51){
                if(drawingType < 4){
                    drawingType += 1;
                }
                else{
                    drawingType = 1;
                }
            }
            // [ r ]
            if(key == 114){
                applyRefinement = !applyRefinement;
            }
            // [ s ]
            if (key == 115) {
                std::string ssname = "screenshot.png";
                cv::imwrite(ssname, processedFrame);
            }
            // [ v ]
            if(key == 118){
                verboseFrame = !verboseFrame;
                polyhucoProcessor.setVerboses(verbose, verboseDebug, verboseFrame);
            }
            // [ - ]
            if(key == 45){
                if(zoom >= 20)
                    zoom -= 10;
                else
                    printAlert("Cannot zoom out less than 10%");
            }
            // [ + ]
            if(key == 43){
                if(zoom < 500)
                    zoom += 10;
                else
                    printAlert("Cannot zoom in more than 500%");
            }
            // [ l ]
            if(key == 108){
                if(lookForMarkers < 3){
                    lookForMarkers += 1;
                }
                else{
                    lookForMarkers = 0;
                }
            }
            // [ h ]
            if(key == 104){
                showUserInputsMessages = !showUserInputsMessages;
            }
            // [ p ]
            if(((processed_frames == 0) && (startingPaused)) or ((key == 112) or (skippedFrame == true))){
                skippedFrame = false;
                while(1){
                    int key = cv::waitKey(1);
                    if(key == 112){ // [ p ]
                        break;
                    }
                    if(key == 110){
                        skippedFrame = true;
                        break;
                    }
                    if(key == 27){ // [ Escape ]
                        continueProcessing = false;
                        break;
                    }
                    if(key == 100){ // [ d ]
                        if(detectionType == 1){
                            detectionType = 2;
                        }
                        else if(detectionType == 2){
                            detectionType = 1;
                        }
                    }
                    if(key == 51){ // [ 3 ]
                        if(drawingType < 4){
                            drawingType += 1;
                        }
                        else{
                            drawingType = 1;
                        }
                    }
                    if(key == 114){ // [ r ]
                        applyRefinement = !applyRefinement;
                    }
                    if(key == 115){ // [ s ]
                        std::string ssname = "screenshot.png";
                        cv::imwrite(ssname, processedFrame);
                    }
                    if(key == 118){ // [ v ]
                        verboseFrame = !verboseFrame;
                        polyhucoProcessor.setVerboses(verbose, verboseDebug, verboseFrame);
                    }
                    if(key == 45){ // [ - ]
                        if(zoom >= 20)
                            zoom -= 10;
                        else
                            printAlert("Cannot zoom out less than 10%");
                    }
                    if(key == 43){ // [ + ]
                        if(zoom < 500)
                            zoom += 10;
                        else
                            printAlert("Cannot zoom in more than 500%");
                    }
                    if(key == 108){ // [ l ]
                        if(lookForMarkers < 3){
                            lookForMarkers += 1;
                        }
                        else{
                            lookForMarkers = 0;
                        }
                    }
                    if(key == 104){ // [ h ]
                        showUserInputsMessages = !showUserInputsMessages;
                    }
                }
            }
            //std::cout<<"Actived key: "<<key<<std::endl;


            // End of processing each frame
            processed_frames++;
            if(!noOutputVideo){
                outputVideo << processedFrame;
            }
            systemFPS.stop(); valueSystemFPS = systemFPS.getAvrg()*1000;
        }
        // Close frame window
        if(!minimal) cv::destroyAllWindows();


        // Close output poses file if needed
        if(generatePosesOutputFile){
            posesFile.close();
        }

    }

    // End of execution
    return 0;
}
