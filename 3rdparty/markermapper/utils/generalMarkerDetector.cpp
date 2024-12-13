#include "generalMarkerDetector.hpp"
#include "markerDetectors/arucoMarkerDetector.hpp"
#include "markerDetectors/cucomarkerMarkerDetector.hpp"

namespace general_marker_detector {



GeneralMarkerDetector::GeneralMarkerDetector(double markerSize, bool verbose, bool verboseDebug){
    this->_markerSize = markerSize;
    this->_verbose = verbose;
    this->_verboseDebug = verboseDebug;
}


general_marker::GeneralMarker GeneralMarkerDetector::getGeneralMarkerById(int id){
    // Search for the id
    for(uint i = 0; i < this->_detectedMarkersIds.size(); i++){
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



std::vector<cv::Point3f> GeneralMarkerDetector::getMinimum4Corners3DSchema(float size, cv::Mat RT, int config){

    std::vector<cv::Point3f> cornersSchema = this->getMarkers3DCornersSchema(size, RT, config);

    // Normal markers
    if(cornersSchema.size() >= 4){
        // Nothing to do
        // ...
    }

    // Triangle markers
    if(cornersSchema.size() == 3){
        // We compute the center and push it as a new "corner"
        cv::Point3f center;
        for(auto c: cornersSchema){
            center += c;
        }
        center = center/3;
        cornersSchema.push_back(center);
    }

    // Default case
    if(cornersSchema.size() == 0){
        // Nothing to do
        // ...
    }

    return cornersSchema;
}




std::shared_ptr<GeneralMarkerDetector> GeneralMarkerDetector::readFromFile(std::string file){


    std::shared_ptr<GeneralMarkerDetector> result;


    // Aruco reading
    try{
        auto test = std::make_shared<aruco_marker_detector::ArucoMarkerDetector> ();
        test->readFromFile(file);
        result = test;
    }catch(std::exception &ex){
        // ...
    }


    // Cucomarker reading
    try{
        auto test = std::make_shared<aruco_mm::CucomarkerMarkerDetector> ();
        test->readFromFile(file);
        result = test;
    }catch(std::exception &ex){
        // ...
    }


    return result;
}



}
