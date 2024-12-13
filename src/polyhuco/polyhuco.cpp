#include "polyhuco.h"
#include "../utils/colors.hpp"
#include "polyhucoPoseFunctions.hpp"

#include <fstream>

namespace polyhuco {

Polyhuco::Polyhuco( cv::String polyhucoName,
    int numFaces,
    cv::String markersTypeName,
    double markersSize,
    int detectionType,
    cv::String markersParamPath,
    cv::String mappedPolyhedronMapPath,
    bool verbose,
    bool verboseDebug,
    bool verboseFrame,
    int solvePNPMethod,
    double subPixSize,
    double subPixRep,
    double subPixEpsilon,
    int id_min,
    int id_max,
    std::string distancesOutputsPath,
    double scaleForMap,
    bool checkCandidates){
    // Verboses
    this->_verbose = verbose;
    this->_verboseDebug = verboseDebug;
    this->_verboseFrame = verboseFrame;
    // General
    this->_polyhucoName = polyhucoName;
    this->_numFaces = numFaces;
    this->_id_min = id_min;
    this->_id_max = id_max;
    this->_distancesOutputsPath = distancesOutputsPath;
    // Refining checking
    this->lastDistanceToObject = -1;
    this->lastAngleToObject = -1;
    // Thresholds
    this->_thresAngle = 0.0; // 0.0 by default
    this->_enableThresAngle = true; // true by default
    double _thresNDet = 0.0;
    double _thresArea = 0.0;
    double _thresDiff = 0.0;
    // SolvePNP Method
    this->_solvePNPMethod = solvePNPMethod;
    // CornerSubPix Refinement
    this->_subPixSize = subPixSize;
    this->_subPixRep = subPixRep;
    this->_subPixEpsilon = subPixEpsilon;
    // Checking candidates
    this->_checkCandidates = checkCandidates;
    // Markers
    this->_markersTypeName = markersTypeName;
    this->_markersSize = markersSize;
    this->_markersParamPath = markersParamPath;
    this->_detectionType = detectionType;
    // Map
    this->_mappedPolyhedronMapPath = mappedPolyhedronMapPath;
    this->_scaleForMap = scaleForMap;
    this->processMap();
    // Printing polyhuco information
    if(this->_verbose){
        this->printInfo();
    }
    // Styles
    // Detected
        this->_innerColor = cv::Scalar(250, 0, 158); this->_innerColorError = cv::Scalar(74, 0, 179);
        this->_outerColor = cv::Scalar(245, 200, 66); this->_outerColorError = cv::Scalar(106, 2, 250);
        this->_innerWeight = 1.0; this->_innerWeightError = 1.0;
        this->_outerWeight = 2.0; this->_outerWeightError = 2.0;
        this->_normalColor = cv::Scalar(0,0,255);
    // Guessed
        this->_innerColorGuessed = cv::Scalar(132, 207, 2); this->_innerColorErrorGuessed = cv::Scalar(124, 4, 194);
        this->_outerColorGuessed = cv::Scalar(158, 250, 0); this->_outerColorErrorGuessed = cv::Scalar(152, 0, 240);
        this->_innerWeightGuessed = 1.0; this->_innerWeightErrorGuessed = 1.0;
        this->_outerWeightGuessed = 1.0; this->_outerWeightErrorGuessed = 1.0;
}


// loadFromContent
void Polyhuco::loadFromContent(const std::vector<std::vector<std::string>>& markersConfig,
                     const std::unordered_map<int,std::vector<cv::Point3f>>& markers) {
    _markersConfig.insert(_markersConfig.end(), markersConfig.begin(), markersConfig.end());
    _markers.insert(markers.begin(), markers.end());
    this->processMap();
}


// printInfo
void Polyhuco::printInfo(){
    std::cout<<std::endl;
    std::cout<<BGREEN<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Polyhuco information: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<RESET<<std::endl;
    std::cout<<BCYAN<<"Verboses: "<<RESET<<std::endl;
    std::cout<<CYAN<<"\t- Verbose: "<<RESET<<this->_verbose<<std::endl;
    std::cout<<CYAN<<"\t- Verbose debug: "<<RESET<<this->_verboseDebug<<std::endl;
    std::cout<<CYAN<<"\t- Verbose frame: "<<RESET<<this->_verboseFrame<<std::endl;
    std::cout<<BGREEN<<"General: "<<RESET<<std::endl;
    std::cout<<GREEN<<"\t- Polyhuco name: "<<RESET<<this->_polyhucoName<<std::endl;
    std::cout<<GREEN<<"\t- Number of faces: "<<RESET<<this->_numFaces<<std::endl;
    std::cout<<BGREEN<<"Markers: "<<RESET<<std::endl;
    std::cout<<GREEN<<"\t- Markers type name: "<<RESET<<this->_markersTypeName<<std::endl;
    std::cout<<GREEN<<"\t- Markers size: "<<RESET<<this->_markersSize<<std::endl;
    std::cout<<GREEN<<"\t- Markers parameters file: "<<RESET<<this->_markersParamPath<<std::endl;
    std::cout<<BGREEN<<"Map: "<<RESET<<std::endl;
    std::cout<<GREEN<<"\t- Map file: "<<RESET<<this->_mappedPolyhedronMapPath<<std::endl;
    std::cout<<BGREEN<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<RESET<<std::endl;
    std::cout<<std::endl;
}




foundInFrame Polyhuco::findInFrame(cv::Mat frame, cv::String camParamsPath, int drawingDetectionType, int lookForMarkers, bool applyRefinement, cv::Mat K_Mat, cv::Mat D_Mat){


    /*~~~~~~~~~~~~~~~~~~~~~~~~~
        Initials settings
    ~~~~~~~~~~~~~~~~~~~~~~~~~*/
    // Copy of the frame passed
    cv::Mat outputFrame = frame.clone();
    // Initial information setting
    this->_foundInformation.outputFrame = outputFrame.clone();
    this->_foundInformation.drawingType = drawingDetectionType;
    this->_foundInformation.detectionInfo.detectionType = this->_detectionType;
    this->_foundInformation.lookForMarkers = lookForMarkers;



    /*~~~~~~~~~~~~~~~~~~~~~~ 
        Compute K and D
    ~~~~~~~~~~~~~~~~~~~~~~*/
    auto fs = cv::FileStorage();
    cv::Mat K, D;
    if(camParamsPath != "-"){
        fs.open(camParamsPath, cv::FileStorage::READ);
        fs["camera_matrix"] >> K;
        fs["distortion_coefficients"] >> D;
        this->_foundInformation.detectionInfo.K = K;
        this->_foundInformation.detectionInfo.D = D;
    }
    else{
        if((K_Mat.empty() == false)&&(D_Mat.empty() == false)){
            K = K_Mat.clone();
            D = D_Mat.clone();
            this->_foundInformation.detectionInfo.K = K;
            this->_foundInformation.detectionInfo.D = D;
        }
    }
    fs.release();
    


    /*~~~~~~~~~~~~~~~~~~~~~
        Detect markers
    ~~~~~~~~~~~~~~~~~~~~~*/
    this->_frameDetector = frame_detector::FrameDetector(this->_markersSize, this->_detectionType, this->_verbose, this->_verboseDebug);
    if(this->_markersParamPath != "LOADED"){
        _frameDetector.setConfiguration(this->_markersParamPath);
    }
    else{
        _frameDetector.setConfiguration(this->_markersConfig);
    }
    std::vector<general_marker::GeneralMarker> markerDetections = _frameDetector.detect(frame);


    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        Relate with mapped markers
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    std::vector<int> globalIds;
    std::vector<cv::Point2f> global2DCorners;
    std::vector<cv::Point3f> global3DCorners;
    std::vector<general_marker::GeneralMarker> detectedMarkers; detectedMarkers.clear();
    for(int i = 0; i < markerDetections.size(); i++){
        general_marker::GeneralMarker markerDetection = markerDetections[i];
        int foundId = markerDetection.getId();
        map_processor::MappedMarkerInformation mappedMarkerInfo = this->_mapProcessor.getMarkerInformation(foundId);
        if(
            (mappedMarkerInfo.id != -1) &&
            (
                (
                    (this->_id_min != -1) &&
                    (this->_id_max != -1) &&
                    (mappedMarkerInfo.id >= this->_id_min) &&
                    (mappedMarkerInfo.id <= this->_id_max)
                ) ||
                (
                    (this->_id_min == -1) &&
                    (this->_id_max == -1)
                )
            )
        ){
        //if(mappedMarkerInfo.id != -1){ // Set for normal |||| //if((mappedMarkerInfo.id >= 1) && (mappedMarkerInfo.id <= 6)){ // Set for aruco cube
            globalIds.push_back(foundId);
            std::vector<cv::Point2f> corners2D = markerDetection.getCorners();
            std::vector<cv::Point3f> corners3D = mappedMarkerInfo.corners;
            markerDetection.setCorners3D(corners3D); markerDetection.setName("Detected marker");
            detectedMarkers.push_back(markerDetection);
            for(int j = 0; j < mappedMarkerInfo.corners.size(); j++){
                global2DCorners.push_back(corners2D[j]);
                global3DCorners.push_back(corners3D[j]);
            }
        }
    }


    if(this->_verboseDebug) std::cout<<"Global ids size: "<<globalIds.size()<<std::endl;


    if((globalIds.size() > 0)){

        /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            Setting the information
        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
        // Found information
        this->_foundInformation.found = true;
        this->_foundInformation.detectionInfo.foundMarkersIds = globalIds;
        this->_foundInformation.detectionInfo.detectedMarkers = detectedMarkers;
        this->_foundInformation.detectionInfo.frameDetector = _frameDetector;
        if((K.empty() == false)&&(D.empty() == false)){


        /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            Initial pose estimation
        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
        computePose(global2DCorners, global3DCorners, applyRefinement, true);
        

        /*~~~~~~~~~~~~~~~~~~~~~~~~
            Look for markers?
        ~~~~~~~~~~~~~~~~~~~~~~~~*/
        if(lookForMarkers > 0){
            // Look for markers
            this->lookForMarkers();
        }
        

        /*~~~~~~~~~~~~~~~~~~~~~~~~
            Apply refinement?
        ~~~~~~~~~~~~~~~~~~~~~~~~*/
        if(applyRefinement){
            // Apply refinement
            this->applyRefinement();
        }



        /*~~~~~~~~~~~~~~~~~~~
            Draw results
        ~~~~~~~~~~~~~~~~~~~*/
        this->drawDetections(applyRefinement);
        if(lookForMarkers > 0){
            this->drawGuessedMarkers(applyRefinement);
        }



        }
    }
    else{
        // No found markers return
        this->_foundInformation.found = false;
        this->_foundInformation.detectionInfo.foundMarkersIds = globalIds;
    }

    // Return the obtained information
    return this->_foundInformation;
}


detectionInformation Polyhuco::getDetectionInfo(){
    detectionInformation detectionInfo = this->_foundInformation.detectionInfo;
    return detectionInfo;
}


map_processor::MappedMarkerInformation Polyhuco::getMappedMarkerInformation3D(int id){
    return this->_mapProcessor.getMarkerInformation(id, true);
}



void Polyhuco::processMap(){
    if(this->_mappedPolyhedronMapPath == "LOADED"){
        if(this->_markers.size() > 0){
            this->_mapProcessor = map_processor::MapProcessor(this->_markers, this->_verbose, this->_verboseDebug, this->_scaleForMap);
            this->_mapProcessor.computeCenter();
            if(this->_verbose){
                this->_mapProcessor.printInfo();
            }
        }
    }
    else{
        this->_mapProcessor = map_processor::MapProcessor(this->_mappedPolyhedronMapPath, this->_verbose, this->_verboseDebug, this->_scaleForMap);
        this->_mapProcessor.computeCenter();
        if(this->_verbose){
            this->_mapProcessor.printInfo();
        }
    }
}



void Polyhuco::lookForMarkers(){

    std::cout<<Colour_Terciary_Bold<<Symbols_SquareEmpty<<" Starting Process:  Looking for markers"<<RESET<<std::endl;

    /*~~~~~~~~~~~~~~~~~~~~~~~~
        Not detected ids
    ~~~~~~~~~~~~~~~~~~~~~~~~*/
    std::cout<<Colour_Terciary<<"\t1. Looking for not detected ids"<<RESET<<std::endl;
    std::vector<int> totalIds = this->_mapProcessor.getIds();
    std::vector<int> notDetectedIds;
    for(int i = 0; i < totalIds.size(); i++){
        bool appear = false;
        for(int j = 0; j < this->_foundInformation.detectionInfo.foundMarkersIds.size(); j++){
            if(this->_foundInformation.detectionInfo.foundMarkersIds[j] == totalIds[i]){
                appear = true;
            }
        }
        if(!appear){
            notDetectedIds.push_back(totalIds[i]);
        }   
    }
    if(this->_verboseDebug){
        std::cout<<"\t\t> No detected ids: { ";
        for(int i = 0; i < notDetectedIds.size(); i++){
            std::cout<<notDetectedIds[i]<<" ";
        }
        std::cout<<"}"<<std::endl;
    }
    

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        Is the no detected marker visible
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    std::cout<<Colour_Terciary<<"\t2. Computing visibility of markers"<<RESET<<std::endl;
    std::vector<int> visibleMarkersIds;
    std::vector<std::vector<cv::Point3f>> visibleMarkersCorners3D;
    std::vector<double> visibleMarkersAngles;
        // Start the looking for
        if(this->_verboseDebug) std::cout<<Colour_Terciary<<"\t\tChecking of: camera plane angle intersections"<<RESET<<std::endl;
        for(int i = 0; i < notDetectedIds.size(); i++){
            // Extract info
            int noDetectedId = notDetectedIds[i];
            map_processor::MappedMarkerInformation markerInfo = this->_mapProcessor.getMarkerInformation(noDetectedId);
            std::vector<cv::Point3f> corners3D = markerInfo.corners;
            // Angle computed (Always look for markers with the pose obtained without refinement)
            double angle = getAngleFromCorners3D(corners3D, this->_foundInformation.detectionInfo.woRefinement.rvec, this->_foundInformation.detectionInfo.woRefinement.tvec);
            bool accepted = angle > this->_thresAngle;
            if(!accepted){
                if(this->_verboseDebug) std::cout<<"\t\t\t"<<RED<<Symbols_Cross<<RESET<<" Marker Id: "<<noDetectedId<<"| Angle: "<<angle<<" (Thres: "<<this->_thresAngle<<")"<<std::endl;
                continue;
            }
            // If accepted
            if(this->_verboseDebug) std::cout<<"\t\t\t"<<GREEN<<Symbols_Check<<RESET<<" Marker Id: "<<noDetectedId<<"| Angle: "<<angle<<" (Thres: "<<this->_thresAngle<<")"<<std::endl;
            // Save found information    
            visibleMarkersIds.push_back(noDetectedId);
            visibleMarkersCorners3D.push_back(corners3D);
            visibleMarkersAngles.push_back(angle);
        }
    if(this->_verboseDebug){
        std::cout<<"\t\t> Accepted ids: { ";
        for(int i = 0; i < visibleMarkersIds.size(); i++){
            std::cout<<visibleMarkersIds[i]<<" ";
        }
        std::cout<<"}"<<std::endl;
    }
    
    


    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        Create guessed markers vector
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    std::cout<<Colour_Terciary<<"\t3. Creating guessedMarkers"<<RESET<<std::endl;
    std::vector<general_marker::GeneralMarker> guessedMarkers;
    for(int i = 0; i < visibleMarkersIds.size(); i++){ // (int id, float size, bool verbose, bool verboseDebug)
        general_marker::GeneralMarker guessedMarker = general_marker::GeneralMarker(visibleMarkersIds[i], this->_verbose, this->_verboseDebug);
        // Set corners 3D
        guessedMarker.setCorners3D(visibleMarkersCorners3D[i]);
        // Set angle
        guessedMarker.setAngle(visibleMarkersAngles[i]);
        // Set name
        guessedMarker.setName("Guessed marker");
        // Set corners 2D
        std::vector<cv::Point2f> local2D;
        cv::projectPoints(guessedMarker.getCorners3D(), this->_foundInformation.detectionInfo.woRefinement.rvec, this->_foundInformation.detectionInfo.woRefinement.tvec, this->_foundInformation.detectionInfo.K, this->_foundInformation.detectionInfo.D, local2D);    
            // Refine with cornerSubPix
            cv::Mat img = this->_foundInformation.outputFrame.clone(); cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
            cv::cornerSubPix(img, local2D, cv::Size(this->_subPixSize,this->_subPixSize), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, this->_subPixRep, this->_subPixEpsilon));
        guessedMarker.setCorners2D(local2D);
        // Push into vector
        guessedMarkers.push_back(guessedMarker);
    }
    // Accepted markers
    std::vector<general_marker::GeneralMarker> guessedSelected = guessedMarkers;


    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        Obtaining 2D Corners: With candidates or reprojecting&subPixRefine
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    if(this->_checkCandidates){
        guessedSelected.clear();
        std::cout<<Colour_Terciary<<"\t4. Relation to contours"<<RESET<<std::endl;
        // Retrieve candidates
        std::vector<general_marker::GeneralMarker> candidates =  this->_frameDetector.getCandidates();
        // For every found marker...
        for(int i = 0; i < guessedMarkers.size(); i++){
            int foundMarkerId = guessedMarkers[i].getId();
            std::cout<<Colour_Terciary<<"\t\tRelations of marker: "<<RESET<<foundMarkerId<<std::endl;
            // Lets analyse each center, lets apply it
            for(int cand_id = 0; cand_id < candidates.size(); cand_id++){
                general_marker::GeneralMarker candidate = candidates[cand_id];
                // Candidate 2D center
                cv::Point2f candidateCenter = obtainCenter2D(candidate.getCorners());
                /*~~~~    Is the candidate related to the markerCandidate?    ~~~~*/
                // Analyzing the centers
                    // Found marker center
                    std::vector<cv::Point2f> corners2D = guessedMarkers[i].getCorners();
                    cv::Point2f foundCenter = obtainCenter2D(corners2D);
                // Difference is under 5%?
                bool approvedCandidate = true;
                double difThres = 5;
                // Diffenrece of x
                double xDifference = computePercentageDifference(candidateCenter.x, foundCenter.x);
                if(xDifference > difThres){
                    approvedCandidate = false;
                }
                // Diffenrece of y
                double yDifference = computePercentageDifference(candidateCenter.y, foundCenter.y);
                if(yDifference > difThres){
                    approvedCandidate = false;
                }  
                // Checking
                if(approvedCandidate){
                    general_marker::GeneralMarker gm = guessedMarkers[i]; 
                    gm.setCorners2D(candidate.getCorners()); // Set new corners 2D
                    gm.setContours(candidate.getContours()); // Set contours
                    guessedSelected.push_back(gm);

                    if(this->_verboseDebug){
                        std::cout<<"\t\t\t"<<GREEN<<Symbols_Check<<RESET<<" Relation with candidate: "<<cand_id<<std::endl;
                        std::cout<<"\t\t\t\tCenter: "<<foundCenter<<std::endl;
                        std::cout<<"\t\t\t\tCandidateCenter: "<<candidateCenter<<std::endl;
                        std::cout<<"\t\t\t\txDifference: "<<xDifference<<std::endl;
                        std::cout<<"\t\t\t\tyDifference: "<<yDifference<<std::endl;
                        std::cout<<"\t\t\t\tyDifference: "<<yDifference<<std::endl;
                    }

                    break;
                }            
                else{
                    if(this->_verboseDebug){
                        std::cout<<"\t\t\t"<<RED<<Symbols_Cross<<RESET<<" Relation with candidate: "<<cand_id<<std::endl;
                        std::cout<<"\t\t\t\tCenter: "<<foundCenter<<std::endl;
                        std::cout<<"\t\t\t\tCandidateCenter: "<<candidateCenter<<std::endl;
                        std::cout<<"\t\t\t\txDifference: "<<xDifference<<std::endl;
                        std::cout<<"\t\t\t\tyDifference: "<<yDifference<<std::endl;
                        std::cout<<"\t\t\t\tyDifference: "<<yDifference<<std::endl;
                    }
                }
            }
        }
        if(this->_verboseDebug){
            std::cout<<"\t\t> Accepted ids: { ";
            for(int i = 0; i < guessedSelected.size(); i++){
                std::cout<<guessedSelected[i].getId()<<" ";
            }
            std::cout<<"}"<<std::endl;
        }
    }

    // Write the information in the found in frame attribute
    // this->_foundInformation.guessedInfo.ids = visibleMarkersIds;
    // this->_foundInformation.guessedInfo.corners3D = visibleMarkersCorners3D;
    // this->_foundInformation.guessedInfo.angles = visibleMarkersAngles;
    this->_foundInformation.guessedInfo.guessedMarkers.clear();
    for(int i = 0; i < guessedSelected.size(); i++){
        this->_foundInformation.guessedInfo.guessedMarkers.push_back(guessedSelected[i]);
    }


    std::cout<<Colour_Terciary_Bold<<Symbols_SquareFilled<<" Finishing Process:  Looking for markers"<<RESET<<std::endl;

}





void Polyhuco::applyRefinement(){

    std::cout<<Colour_Quaternary_Bold<<Symbols_SquareEmpty<<" Starting Process:  Applying refinement"<<RESET<<std::endl;

    // Englobal corners 2D and 3D for making the refinement
    std::cout<<Colour_Quaternary<<"\t1. Reunite corners 2D and 3D from detected and guessed markers"<<RESET<<std::endl;
    std::vector<cv::Point2f> corners2D;
    std::vector<cv::Point3f> corners3D;
        /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            Detected markers information
        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
        std::cout<<Colour_Quaternary<<"\t\tChecking detected markers"<<RESET<<std::endl;
        std::vector<general_marker::GeneralMarker> detectedMarkers = this->_foundInformation.detectionInfo.detectedMarkers;
        for(int i = 0; i < detectedMarkers.size(); i++){
            general_marker::GeneralMarker marker = detectedMarkers[i];
            std::vector<cv::Point2f> local2D = marker.getCorners();
            std::vector<cv::Point3f> local3D = marker.getCorners3D();
            checkMarkerInfo checkInfo = checkMarker(marker);
            if(checkInfo.correct){
                if(this->_verboseDebug) std::cout<<"\t\t\t"<<GREEN<<Symbols_Check<<RESET<<" Marker Id: "<<marker.id<<std::endl;
                for(int j = 0; j < local2D.size(); j++){
                    corners2D.push_back(local2D[j]);
                    corners3D.push_back(local3D[j]);
                }
            }
            else{
                if(this->_verboseDebug) std::cout<<"\t\t\t"<<RED<<Symbols_Cross<<RESET<<" Marker Id: "<<marker.id<<"| Reason: "<<checkInfo.reason<<std::endl;
            }
        }
        /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            Guessed markers information
        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
        std::cout<<Colour_Quaternary<<"\t\tChecking guessed markers"<<RESET<<std::endl;
        // Use guessed markers if there is any valid one
        if(this->_foundInformation.lookForMarkers != 0){
            // Maybe there are some guessed markers
            for(int i = 0; i < this->_foundInformation.guessedInfo.guessedMarkers.size(); i++){
                general_marker::GeneralMarker guessedMarker = this->_foundInformation.guessedInfo.guessedMarkers[i];
                // Guessed marker information
                std::vector<cv::Point2f> local2D = guessedMarker.getCorners();
                std::vector<cv::Point3f> local3D = guessedMarker.getCorners3D();
                // Checking
                checkMarkerInfo checkInfo = checkMarker(guessedMarker);
                if(checkInfo.correct){
                    if(this->_verboseDebug) std::cout<<"\t\t\t"<<GREEN<<Symbols_Check<<RESET<<" Marker Id: "<<guessedMarker.id<<std::endl;
                    for(int j = 0; j < local2D.size(); j++){
                        corners2D.push_back(local2D[j]);
                        corners3D.push_back(local3D[j]);
                    }
                }
                else{
                    if(this->_verboseDebug) std::cout<<"\t\t\t"<<RED<<Symbols_Cross<<RESET<<" Marker Id: "<<guessedMarker.id<<"| Reason: "<<checkInfo.reason<<std::endl;
                }
            }
        }

    // There are enoughs corners to compute a new pose?
    if(corners2D.size() == 0){
        std::cout<<Colour_Quaternary<<"\tEnd: There is not enough accepted markers for refinement"<<RESET<<std::endl;
    }
    else{
        std::cout<<Colour_Quaternary<<"\t2. Recomputing the pose estimation with the accepted markers"<<RESET<<std::endl;
        
        /*~~~~~~~~~~~~~~~~~~
            Recomputing
        ~~~~~~~~~~~~~~~~~~*/
        // Compute new pose
        computePose(corners2D, corners3D, true, false); // PONER APPLYREFINEMENT

        /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            Reproject 2d corners of detected markers
        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
        std::cout<<Colour_Quaternary<<"\t3. Reprojecting detected markers"<<RESET<<std::endl;
        // Reprojections
        std::vector<general_marker::GeneralMarker> refinedDetectedMarkers;
        for(int i = 0; i < this->_foundInformation.detectionInfo.detectedMarkers.size(); i++){
            general_marker::GeneralMarker rMarker = this->_foundInformation.detectionInfo.detectedMarkers[i]; rMarker.setName("Detected marker (Reprojected)");
            std::vector<cv::Point2f> local2D;
            std::vector<cv::Point3f> local3D = rMarker.getCorners3D();
            // For this reprojections, we are gonna use the refined pose
            cv::projectPoints(local3D, this->_foundInformation.detectionInfo.wRefinement.rvec, this->_foundInformation.detectionInfo.wRefinement.tvec, this->_foundInformation.detectionInfo.K, this->_foundInformation.detectionInfo.D, local2D);
            rMarker.setCorners2D(local2D);
            refinedDetectedMarkers.push_back(rMarker);
        }
        // Set the refined markers
        this->_foundInformation.detectionInfo.detectedMarkers = refinedDetectedMarkers;

        /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            Reproject 2d corners of guessed markers
        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
        std::cout<<Colour_Quaternary<<"\t4. Reprojecting guessed markers"<<RESET<<std::endl;
        // Reprojections
        std::vector<general_marker::GeneralMarker> refinedGuessedMarkers;
        for(int i = 0; i < this->_foundInformation.guessedInfo.guessedMarkers.size(); i++){
            general_marker::GeneralMarker rMarker = this->_foundInformation.guessedInfo.guessedMarkers[i]; rMarker.setName("Guessed marker (Reprojected)");
            std::vector<cv::Point2f> local2D;
            std::vector<cv::Point3f> local3D = rMarker.getCorners3D();
            // For this reprojections, we are gonna use the refined pose
            cv::projectPoints(local3D, this->_foundInformation.detectionInfo.wRefinement.rvec, this->_foundInformation.detectionInfo.wRefinement.tvec, this->_foundInformation.detectionInfo.K, this->_foundInformation.detectionInfo.D, local2D);
            rMarker.setCorners2D(local2D);
            refinedGuessedMarkers.push_back(rMarker);
        }
        // Set the refined markers
        this->_foundInformation.guessedInfo.guessedMarkers = refinedGuessedMarkers;

    }


    std::cout<<Colour_Quaternary_Bold<<Symbols_SquareFilled<<" Finishing Process:  Applying refinement"<<RESET<<std::endl;

}


void Polyhuco::checkSinglePose(polyhuco::detectionInformationModule &poseModule){
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        One pose checking.
            a. (DISABLED) meanArea aboce thresAngle
            b. meanAngle above thresArea
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        a. meanArea aboce thresAngle
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
        if(poseModule.poseApproved){
            std::cout<<Colour_Secondary<<"\t\ta. (DISABLED) meanArea above thresAngle"<<RESET<<std::endl;
            // // Threshold
            // int thresArea = this->_thresArea;
            // if(this->_verboseDebug) std::cout<<"\t\t\t- Threshold: "<<RESET<<thresArea<<std::endl;
            // // avgArea
            // double avgArea = poseModule.avgArea;
            // std::cout<<"\t\t\tAvg detected area: "<<RESET<<avgArea<<std::endl;
            // // Condition
            // bool condArea = avgArea >= thresArea;
            // if(condArea){
            //     // Approved
            //     std::cout << "\t\t\t\t" << GREEN << Symbols_Check << " Condition passed!" << "\033[0m\n";
            // }
            // else{
            //     // Non approved --> We disapprove every computed pose
            //     std::cout << "\t\t\t\t" << RED << Symbols_Cross << " Condition not passed, discarding the pose" << "\033[0m\n";
            //     poseModule.poseApproved = false;
            // }
        }
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        b. meanAngle above thresArea
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
        if(poseModule.poseApproved){
            std::cout<<Colour_Secondary<<"\t\tb. meanAngle above thresArea"<<RESET<<std::endl;
            // Threshold
            int thresAngle = this->_thresAngle;
            if(this->_verboseDebug) std::cout<<"\t\t\t- Threshold: "<<RESET<<thresAngle<<std::endl;
            // avgArea
            double avgAngle = poseModule.avgAngle;
            std::cout<<"\t\t\tAvg detected angle: "<<RESET<<avgAngle<<std::endl;
            // Condition
            bool condAngle = avgAngle >= thresAngle;
            if(condAngle){
                // Approved
                std::cout << "\t\t\t\t" << GREEN << Symbols_Check << " Condition passed!" << "\033[0m\n";
            }
            else{
                // Non approved --> We disapprove every computed pose
                std::cout << "\t\t\t\t" << RED << Symbols_Cross << " Condition not passed, discarding the pose" << "\033[0m\n";
                poseModule.poseApproved = false;
            }
        }
}


void Polyhuco::computePose(std::vector<cv::Point2f> corners2D, std::vector<cv::Point3f> corners3D, bool applyLookForMarkers, bool isfirstCallBeforeRefinement){
    
    // Compute pose
    cv::Mat rvec, tvec; 

    std::cout<<Colour_Primary_Bold<<Symbols_SquareEmpty<<" Starting Process:  Computing the pose"<<RESET<<std::endl;
    std::cout<<Colour_Primary<<"\t1. Computing the pose"<<RESET<<std::endl;
    bool response = cv::solvePnP(corners3D, corners2D, this->_foundInformation.detectionInfo.K, this->_foundInformation.detectionInfo.D, rvec, tvec, false, this->_solvePNPMethod); // cv::SOLVEPNP_EPNP
    
    
    // Refine estimated pose
    //if(applyRefinement){
    // if(false){
    //     refineEstimatedPose<double>(rvec, tvec, this->_foundInformation.detectionInfo.K, this->_foundInformation.detectionInfo.D); 
    // }
    
    if(this->_verboseDebug){
        std::cout<<YELLOW;
        std::cout<<"\t\tTVEC: "<<tvec<<std::endl;
        std::cout<<"\t\tRVEC: "<<rvec<<std::endl;
        for (auto c : corners2D){
            std::cout<<"\t\t2D: "<<c<<std::endl;
        }
        for (auto c3 : corners3D){
            std::cout<<"\t\t3D: "<<c3<<std::endl;
        }
        std::cout<<RESET;
    }


    /*~~~~~~~~~~~~~~~~~~~
        Saving pose
    ~~~~~~~~~~~~~~~~~~~*/
    if(applyLookForMarkers == true){
        if(isfirstCallBeforeRefinement == true){ 
            this->_foundInformation.detectionInfo.woRefinement.rvec = rvec.clone();
            this->_foundInformation.detectionInfo.woRefinement.tvec = tvec.clone();
            this->_foundInformation.detectionInfo.woRefinement.poseApproved = true; // By default approved
        }
        else{ 
            this->_foundInformation.detectionInfo.wRefinement.rvec = rvec.clone();
            this->_foundInformation.detectionInfo.wRefinement.tvec = tvec.clone();
            this->_foundInformation.detectionInfo.wRefinement.poseApproved = true; // By default approved
        }
    }
    else{ 
        this->_foundInformation.detectionInfo.woRefinement.rvec = rvec.clone();
        this->_foundInformation.detectionInfo.woRefinement.tvec = tvec.clone();
        this->_foundInformation.detectionInfo.woRefinement.poseApproved = true; // By default approved
    }


    // Compute markers angles
    if(applyLookForMarkers == true){
        if(isfirstCallBeforeRefinement == true){
            // Compute angles without refinement
            this->computeMarkersAngles(false);
        }
        else{
            // Compute angles with refinement
            this->computeMarkersAngles(true);
        }
    }
    else{
        // Compute angles without refinement
        this->computeMarkersAngles(false);
    }


    // Compute areas
    std::cout<<Colour_Primary<<"\t2. (DISABLED) Computing the marker 2D areas"<<RESET<<std::endl;
    //     // Compute normalised average area
    //     // Avg area
    //     double avgArea = 0.0;
    //     for(int m = 0; m < this->_foundInformation.detectionInfo.detectedMarkers.size(); m++){
    //         avgArea += this->_foundInformation.detectionInfo.detectedMarkers[m].getArea();
    //     }
    //     avgArea /= this->_foundInformation.detectionInfo.detectedMarkers.size();
    //     // Normalised area
    //     double normalisedArea = avgArea;
    //         //normalisedArea = normalisedArea * distanceToObject;
    //     // Resolution
    //         //normalisedArea = normalisedArea / (this->_foundInformation.outputFrame.cols*this->_foundInformation.outputFrame.rows);
    //         double min_x, max_x, min_y, max_y;
    //         for(int m = 0; m < this->_foundInformation.detectionInfo.detectedMarkers.size(); m++){
    //             general_marker::GeneralMarker marker = this->_foundInformation.detectionInfo.detectedMarkers[m];
    //             std::vector<cv::Point2f> corners2D = marker.getCorners();
    //             for(int p = 0; p < corners2D.size(); p++){
    //                 double x = corners2D[p].x;
    //                 double y = corners2D[p].y;
    //                 if(m == 0 && p == 0){
    //                     min_x = x;
    //                     max_x = x;
    //                     min_y = y;
    //                     max_y = y;
    //                 }
    //                 else{
    //                     if(min_x > x){min_x = x;}
    //                     if(max_x < x){max_x = x;}
    //                     if(min_y > y){min_y = y;}
    //                     if(max_y < y){max_y = y;}
    //                 }
    //             }
    //         }
    //         // Lenghts of the rectangle
    //         double width = max_x - min_x;
    //         double height = max_y - min_y;
    //         double resolution = width*height;
    //         //normalisedArea = normalisedArea / resolution;
    //     // Applying markerSize
    //     normalisedArea = normalisedArea * this->_markersSize;

    // // Saving average area
    // if(applyLookForMarkers == true){
    //     if(isfirstCallBeforeRefinement == true){ // Save non-refined area
    //         this->_foundInformation.detectionInfo.woRefinement.avgArea = normalisedArea;
    //     }
    //     else{ // Save refined area
    //         this->_foundInformation.detectionInfo.wRefinement.avgArea = normalisedArea;
    //     }
    // }
    // else{ // Save non-refined area
    //     this->_foundInformation.detectionInfo.woRefinement.avgArea = normalisedArea;
    // }

    
    
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        Checking quality of the poses
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    std::cout<<Colour_Primary<<"\t3. Checking quality of the poses"<<RESET<<std::endl;
    // Checking groups
    bool check_woRefinement = false;
    bool check_wRefinement = false;
    bool check_globalCheking = false;
    bool check_RefinementVSwoRefinement = false;
    // Processing schema
    if(applyLookForMarkers == true){
        if(isfirstCallBeforeRefinement == false){
            // Check: global checking
            check_globalCheking = true;
            // Check: rest
            check_woRefinement = true;
            check_wRefinement = true;
            check_RefinementVSwoRefinement = true;
        }
    }
    else{
        // Check: global checking
        check_globalCheking = true;
        // Check: rest
        check_woRefinement = true;
    }
    bool someCheckToDo = check_woRefinement || check_wRefinement || check_globalCheking || check_RefinementVSwoRefinement;



    if(someCheckToDo) std::cout<<Colour_Secondary_Bold<<Symbols_SquareEmpty<<" Starting Process:  Checking quality of poses"<<RESET<<std::endl;
    /* 
        Checking in order:
            1. Global checking
            2. wRefinement vs woRefinement.
            3. wRefinement.
            4. woRefinement.
    */



    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        1. Global checking
            Every computed pose (woRefinement, wRefinement) is bad if:
                a. Number of detected markers is lower than a threshold          
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    if(check_globalCheking){
        std::cout<<Colour_Secondary<<"\tChecking of: Global checking..."<<RESET<<std::endl;
            /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                a. Number of detected markers is lower than a threshold
            ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
            // Threshold
            int thresNDet = this->_thresNDet;
            std::cout<<Colour_Secondary<<"\t\ta. Number of detected markers is lower than a threshold"<<RESET<<std::endl;
            if(this->_verboseDebug) std::cout<<"\t\t\t- Threshold: "<<RESET<<thresNDet<<std::endl;
            double numDetections = this->_foundInformation.detectionInfo.foundMarkersIds.size();
            std::cout<<"\t\t\tNum detections: "<<numDetections<<std::endl;
            bool condNumberDetectedMarkers = numDetections < thresNDet;
            if(condNumberDetectedMarkers){
                // Non approved --> We disapprove every computed pose
                std::cout << "\t\t\t\t" << RED << Symbols_Cross << " Condition not passed, discarding every computed pose" << "\033[0m\n";
                this->_foundInformation.detectionInfo.woRefinement.poseApproved = false;
                this->_foundInformation.detectionInfo.wRefinement.poseApproved = false;
            }
            else{
                // Approved
                std::cout << "\t\t\t\t" << GREEN << Symbols_Check << " Condition passed!" << "\033[0m\n";
            }
    }


    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        2. Rejection of refined versus unrefined pose?
            If any of these gets true:
                a. Difference of tvec modules over difThres%.
                b. Difference of rvec modules over difThres%.
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    double difThres = this->_thresDiff;
    if((check_RefinementVSwoRefinement) && (this->_foundInformation.detectionInfo.woRefinement.poseApproved) && (this->_foundInformation.detectionInfo.wRefinement.poseApproved)){
        std::cout<<Colour_Secondary<<"\tChecking of: wRefinement vs woRefinement..."<<RESET<<std::endl;
            /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
                a. Difference of tvec modules over difThres%
            ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
            std::cout<<Colour_Secondary<<"\t\ta. Difference of tvec modules"<<RESET<<std::endl;
            if(this->_verboseDebug) std::cout<<"\t\t\tThreshold: "<<difThres<<"%"<<std::endl;
            // Distances
            double tvec_distance_wRefinement = cv::norm(this->_foundInformation.detectionInfo.wRefinement.tvec, cv::NORM_L2);
            double tvec_distance_woRefinement = cv::norm(this->_foundInformation.detectionInfo.woRefinement.tvec, cv::NORM_L2);
            // Difference
            double dist_percentage_difference = (tvec_distance_wRefinement / tvec_distance_woRefinement)*100;
            dist_percentage_difference = fabs(100-dist_percentage_difference); 
            std::cout << "\t\t\t(TVEC) The percentage difference between " << tvec_distance_wRefinement << " and " << tvec_distance_woRefinement << " is " << dist_percentage_difference << "%" << std::endl;
            if(dist_percentage_difference > difThres){
                // Non approved
                this->_foundInformation.detectionInfo.wRefinement.poseApproved = false;
                std::cout << "\t\t\t\t" << RED << Symbols_Cross << " Condition not passed, discarding refined distance" << "\033[0m\n";
            }
            else{
                // Approved
                std::cout << "\t\t\t\t" << GREEN << Symbols_Check << " Condition passed!" << "\033[0m\n";
            }
            /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
                b. Difference of rvec modules over difThres%
            ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
            std::cout<<Colour_Secondary<<"\t\ta. Difference of rvec modules"<<RESET<<std::endl;
            if(this->_verboseDebug) std::cout<<"\t\t\tThreshold: "<<difThres<<"%"<<std::endl;
            // Distances
            double rvec_distance_wRefinement = cv::norm(this->_foundInformation.detectionInfo.wRefinement.rvec, cv::NORM_L2);
            double rvec_distance_woRefinement = cv::norm(this->_foundInformation.detectionInfo.woRefinement.rvec, cv::NORM_L2);
            // Difference
            double angle_percentage_difference = (rvec_distance_wRefinement / rvec_distance_woRefinement)*100;
            angle_percentage_difference = fabs(100-angle_percentage_difference); 
            std::cout << "\t\t\t(RVEC) The percentage difference between " << rvec_distance_wRefinement << " and " << rvec_distance_woRefinement << " is " << angle_percentage_difference << "%" << std::endl;
            if(angle_percentage_difference > difThres){
                // Non approved
                this->_foundInformation.detectionInfo.wRefinement.poseApproved = false;
                std::cout << "\t\t\t\t" << RED << Symbols_Cross << " Condition not passed, discarding refined distance" << "\033[0m\n";
            }
            else{
                // Approved
                std::cout << "\t\t\t\t" << GREEN << Symbols_Check << " Condition passed!" << "\033[0m\n";
            }
    }



    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        3. Checking wRefinement pose -> Running one pose checking
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    if(check_wRefinement && this->_foundInformation.detectionInfo.wRefinement.poseApproved){
        std::cout<<Colour_Secondary<<"\tChecking of: wRefinement pose..."<<RESET<<std::endl;
        this->checkSinglePose(this->_foundInformation.detectionInfo.wRefinement);
    }


    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        4. Checking woRefinement pose -> Running one pose checking
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    if(check_woRefinement && this->_foundInformation.detectionInfo.woRefinement.poseApproved){
        std::cout<<Colour_Secondary<<"\tChecking of: woRefinement pose..."<<RESET<<std::endl;
        this->checkSinglePose(this->_foundInformation.detectionInfo.woRefinement);
    }




    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        Selection of the best pose overall
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    this->_foundInformation.detectionInfo.poseIsSelected = false;
    this->_foundInformation.detectionInfo.selectedPoseId = "-";
    this->_foundInformation.detectionInfo.selectedPose.poseApproved = false;
    // Selecting the pose
    if(applyLookForMarkers == true){
        if(isfirstCallBeforeRefinement == false){
            // wRefinement OR woRefinement OR none
            if(this->_foundInformation.detectionInfo.wRefinement.poseApproved){
                this->_foundInformation.detectionInfo.poseIsSelected = true;
                this->_foundInformation.detectionInfo.selectedPoseId = "wRefinement";
                this->_foundInformation.detectionInfo.selectedPose.poseApproved = true;
                this->_foundInformation.detectionInfo.selectedPose.tvec = this->_foundInformation.detectionInfo.wRefinement.tvec.clone();
                this->_foundInformation.detectionInfo.selectedPose.rvec = this->_foundInformation.detectionInfo.wRefinement.rvec.clone();
                this->_foundInformation.detectionInfo.selectedPose.avgAngle = this->_foundInformation.detectionInfo.wRefinement.avgAngle;
                this->_foundInformation.detectionInfo.selectedPose.avgArea = this->_foundInformation.detectionInfo.wRefinement.avgArea;
            }
            else if(this->_foundInformation.detectionInfo.woRefinement.poseApproved){
                this->_foundInformation.detectionInfo.poseIsSelected = true;
                this->_foundInformation.detectionInfo.selectedPoseId = "woRefinement";
                this->_foundInformation.detectionInfo.selectedPose.poseApproved = true;
                this->_foundInformation.detectionInfo.selectedPose.tvec = this->_foundInformation.detectionInfo.woRefinement.tvec.clone();
                this->_foundInformation.detectionInfo.selectedPose.rvec = this->_foundInformation.detectionInfo.woRefinement.rvec.clone();
                this->_foundInformation.detectionInfo.selectedPose.avgAngle = this->_foundInformation.detectionInfo.woRefinement.avgAngle;
                this->_foundInformation.detectionInfo.selectedPose.avgArea = this->_foundInformation.detectionInfo.woRefinement.avgArea;
            }
        }
    }
    else{
        if(this->_foundInformation.detectionInfo.woRefinement.poseApproved){
            this->_foundInformation.detectionInfo.poseIsSelected = true;
            this->_foundInformation.detectionInfo.selectedPoseId = "woRefinement";
            this->_foundInformation.detectionInfo.selectedPose.poseApproved = true;
            this->_foundInformation.detectionInfo.selectedPose.tvec = this->_foundInformation.detectionInfo.woRefinement.tvec.clone();
            this->_foundInformation.detectionInfo.selectedPose.rvec = this->_foundInformation.detectionInfo.woRefinement.rvec.clone();
            this->_foundInformation.detectionInfo.selectedPose.avgAngle = this->_foundInformation.detectionInfo.woRefinement.avgAngle;
            this->_foundInformation.detectionInfo.selectedPose.avgArea = this->_foundInformation.detectionInfo.woRefinement.avgArea;
        }
    }



    // Printing every pose for debugging
    if(this->_verboseDebug){
        if(applyLookForMarkers == true){
            if(isfirstCallBeforeRefinement == false){
                std::cout<<"wRefinement:"<<std::endl;
                    std::cout<<"- tvec: "<<this->_foundInformation.detectionInfo.wRefinement.tvec<<std::endl;
                    std::cout<<"- tvec (module): "<<cv::norm(this->_foundInformation.detectionInfo.wRefinement.tvec, cv::NORM_L2)<<std::endl;
                    std::cout<<"- rvec: "<<this->_foundInformation.detectionInfo.wRefinement.rvec<<std::endl;
                    std::cout<<"- pose_line: "<<getPoseLine(-1, this->_foundInformation.detectionInfo.wRefinement.rvec, this->_foundInformation.detectionInfo.wRefinement.tvec, false);
                    std::cout<<"- avgAngle: "<<this->_foundInformation.detectionInfo.wRefinement.avgAngle<<std::endl;
                    std::cout<<"- avgArea: "<<this->_foundInformation.detectionInfo.wRefinement.avgArea<<std::endl;
                    std::cout<<"- poseApproved: "<<this->_foundInformation.detectionInfo.wRefinement.poseApproved<<std::endl;
                std::cout<<"woRefinement:"<<std::endl;
                    std::cout<<"- tvec: "<<this->_foundInformation.detectionInfo.woRefinement.tvec<<std::endl;
                    std::cout<<"- tvec (module): "<<cv::norm(this->_foundInformation.detectionInfo.woRefinement.tvec, cv::NORM_L2)<<std::endl;
                    std::cout<<"- rvec: "<<this->_foundInformation.detectionInfo.woRefinement.rvec<<std::endl;
                    std::cout<<"- pose_line: "<<getPoseLine(-1, this->_foundInformation.detectionInfo.woRefinement.rvec, this->_foundInformation.detectionInfo.woRefinement.tvec, false);
                    std::cout<<"- avgAngle: "<<this->_foundInformation.detectionInfo.woRefinement.avgAngle<<std::endl;
                    std::cout<<"- avgArea: "<<this->_foundInformation.detectionInfo.woRefinement.avgArea<<std::endl;
                    std::cout<<"- poseApproved: "<<this->_foundInformation.detectionInfo.woRefinement.poseApproved<<std::endl;
            }
        }
        else{
            std::cout<<"woRefinement:"<<std::endl;
                std::cout<<"- tvec: "<<this->_foundInformation.detectionInfo.woRefinement.tvec<<std::endl;
                std::cout<<"- tvec (module): "<<cv::norm(this->_foundInformation.detectionInfo.woRefinement.tvec, cv::NORM_L2)<<std::endl;
                std::cout<<"- rvec: "<<this->_foundInformation.detectionInfo.woRefinement.rvec<<std::endl;
                std::cout<<"- pose_line: "<<getPoseLine(-1, this->_foundInformation.detectionInfo.woRefinement.rvec, this->_foundInformation.detectionInfo.woRefinement.tvec, false);
                std::cout<<"- avgAngle: "<<this->_foundInformation.detectionInfo.woRefinement.avgAngle<<std::endl;
                std::cout<<"- avgArea: "<<this->_foundInformation.detectionInfo.woRefinement.avgArea<<std::endl;
                std::cout<<"- poseApproved: "<<this->_foundInformation.detectionInfo.woRefinement.poseApproved<<std::endl;

        }
    }


    if(someCheckToDo) std::cout<<Colour_Secondary_Bold<<Symbols_SquareFilled<<" Finishing Process:  Checking quality of poses"<<RESET<<std::endl;



    std::cout<<Colour_Primary_Bold<<Symbols_SquareFilled<<" Finishing Process:  Computing the pose"<<RESET<<std::endl;
}



void Polyhuco::computeMarkersAngles(bool wRefinement){

    // Which pose to use
    cv::Mat tvec;
    cv::Mat rvec;
    if(wRefinement){
        tvec = this->_foundInformation.detectionInfo.wRefinement.tvec.clone();
        rvec = this->_foundInformation.detectionInfo.wRefinement.rvec.clone();
    }
    else{
        tvec = this->_foundInformation.detectionInfo.woRefinement.tvec.clone();
        rvec = this->_foundInformation.detectionInfo.woRefinement.rvec.clone();
    }


    // Compute angles for detected markers
    std::vector<general_marker::GeneralMarker> newDetectedMarkers;
    double angles = 0.0;
    for(int i = 0; i < this->_foundInformation.detectionInfo.detectedMarkers.size(); i++){
        general_marker::GeneralMarker markerDetection = this->_foundInformation.detectionInfo.detectedMarkers[i]; 
        double angle = getAngleFromCorners3D(markerDetection.getCorners3D(), rvec, tvec);
        markerDetection.setAngle(angle);
        angles += angle;
        newDetectedMarkers.push_back(markerDetection);
    }
    this->_foundInformation.detectionInfo.detectedMarkers = newDetectedMarkers;
    // Set mean angle
        angles /= this->_foundInformation.detectionInfo.detectedMarkers.size();
        if(wRefinement){
            this->_foundInformation.detectionInfo.wRefinement.avgAngle = angles;
        }
        else{
            this->_foundInformation.detectionInfo.woRefinement.avgAngle = angles;
        }

    // Compute angles for guessed markers
    for(int i = 0; i < this->_foundInformation.guessedInfo.guessedMarkers.size(); i++){
        std::vector<cv::Point3f> corners3D = this->_foundInformation.guessedInfo.guessedMarkers[i].getCorners3D();
        double angle = getAngleFromCorners3D(corners3D, rvec, tvec);
        this->_foundInformation.guessedInfo.guessedMarkers[i].setAngle(angle);
    }
}





checkMarkerInfo Polyhuco::checkMarker(general_marker::GeneralMarker marker){
    // Initial information
    checkMarkerInfo checkInfo;
    checkInfo.correct = true;
    checkInfo.reason = "";

    /*~~~~~~~~~~~~~~~~~~~~~~~~
        Check for reasons
    ~~~~~~~~~~~~~~~~~~~~~~~~*/
    // angle thresholding
    if((this->_enableThresAngle == true) && (marker.getAngle() < this->_thresAngle)){
        checkInfo.correct = false;
        checkInfo.reason = "Angle is insufficient";
    }   

    // Return
    return checkInfo;
};



}
