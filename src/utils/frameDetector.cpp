#include "frameDetector.hpp"
#include "corners3D.cpp"

namespace frame_detector {


FrameDetector::FrameDetector(double markerSize, int detectionType, bool verbose, bool verboseDebug, bool applyRefinement){
    this->_markerSize = markerSize;
    this->_detectionType = detectionType;
    this->_verbose = verbose;
    this->_verboseDebug = verboseDebug;
    this->_applyRefinement = applyRefinement;
}

general_marker::GeneralMarker FrameDetector::getGeneralMarkerById(int id){
    // Search for the id
    for(int i = 0; i < this->_detectedMarkersIds.size(); i++){
        if(id == this->_detectedMarkersIds[i]){
            return this->_detectedMarkers[i];
        }
    }
    // Default return for not found (id = -1)
    std::vector<cv::Point2f> defaultCorners;
    std::vector<cv::Point3f> defaultCorners3D;
    general_marker::GeneralMarker defaultGeneralMarker = general_marker::GeneralMarker(-1, defaultCorners, defaultCorners3D);
    return defaultGeneralMarker;
}



void FrameDetector::setConfiguration(std::string markersParamsPath){
    this->_markersParamsPath = markersParamsPath;
    if(this->_detectionType == 1){ // Cucomarker
        // free(this->_cucomarkerDetector);
        this->_cucomarkerDetector = new cucomarker_marker_detector::CucomarkerMarkerDetector(1, false, false);
        this->_cucomarkerDetector->setParams(markersParamsPath);
        this->_cucomarkerDetector->setColorDetectionMode("GRAY");
    }
}


void FrameDetector::setConfiguration(std::vector<std::vector<std::string>> markersConfig){
    this->_markersParamsPath = "LOADED";
    if(this->_detectionType == 1){ // Cucomarker
        // free(this->_cucomarkerDetector);
        this->_cucomarkerDetector = new cucomarker_marker_detector::CucomarkerMarkerDetector(1, false, false);
        this->_cucomarkerDetector->setParams(markersConfig);
        this->_cucomarkerDetector->setColorDetectionMode("GRAY");
    }
}


std::vector<general_marker::GeneralMarker> FrameDetector::detect(cv::Mat frame){
    this->_frameOfDetection = frame.clone();

    if(this->_detectionType == 1){

        /*~~~~~~~~~~~~~~~~~~~~~~~~~~
            Cucomarker detection
        ~~~~~~~~~~~~~~~~~~~~~~~~~~*/

        /* // OPCION NATIVA */
//        // Configuration
//        cucomarker::MarkerDetector MDetector;
//        MDetector.loadConfigFile(this->_markersParamsPath);
//        MDetector.setColorDetectionMode("GRAY");
//        // Detection
//        std::vector<cucomarker::Marker> markers = MDetector.detect(frame);
//        for(int i = 0; i < markers.size(); i++){
//            std::vector<cv::Point2f> corners;

//            //std::vector<cv::Point3f> corners3D = markers[i].get3DCorners(this->_markerSize);
//            std::vector<cv::Point3f> corners3D = getMarkers3DCornersSchema(this->_markerSize, this->_markersParamsPath);

//            for(int j = 0; j < markers[i].size(); j++){
//                corners.push_back(markers[i][j]);
//            }
//            general_marker::GeneralMarker newDetectedMarker = general_marker::GeneralMarker(markers[i].id, corners, corners3D, markers[i].getArea());
//            newDetectedMarker.setContours(markers[i].contours);
//            this->_detectedMarkersIds.push_back(newDetectedMarker.getId());
//            this->_detectedMarkers.push_back(newDetectedMarker);
//        }
        


        /* // OPCION WRAPPED */
        // No needed extra configuration of the detector, already configured
        // Detection of the markers
        std::vector<general_marker::GeneralMarker> detectedMarkers = this->_cucomarkerDetector->detect(frame);
        for(int i = 0; i < detectedMarkers.size(); i++){
            general_marker::GeneralMarker newDetectedMarker = detectedMarkers[i];
            this->_detectedMarkersIds.push_back(newDetectedMarker.getId());
            this->_detectedMarkers.push_back(newDetectedMarker);
        }
        // Marker candidates
        std::vector<general_marker::GeneralMarker> markerCandidates = this->_cucomarkerDetector->getCandidates();
        this->_markerCandidates.clear();
        for(auto m: markerCandidates){
            this->_markerCandidates.push_back(m);
        }

    }

    // Default return
    return _detectedMarkers;
}



}
