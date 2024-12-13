#ifndef _CucomarkerMarkerDetector_H_
#define _CucomarkerMarkerDetector_H_

#include <cstdlib>
#include <vector>

#include "../generalMarker.hpp"
#include "../generalMarkerDetector.hpp"
#include "../../../3rdparty/cucomarker/cucomarker.h"


// Eliminar inlcudes -----------
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
//----------------


namespace cucomarker_marker_detector {

/**
 * @brief Class that represents the related information of several detected markers of a single frame
 *
 */
class CucomarkerMarkerDetector: public general_marker_detector::GeneralMarkerDetector {
public:

    // Constructor
    CucomarkerMarkerDetector():GeneralMarkerDetector(){}
    inline CucomarkerMarkerDetector(double markerSize, bool verbose = false, bool verboseDebug = false);

    /* To implement */
    // Print
    inline void print();
        // Detection
    inline std::vector<general_marker::GeneralMarker> detect(cv::Mat frame);
        // Computing 3D corners
    inline std::vector<cv::Point3f> getMarkers3DCornersSchema(float size = -1, cv::Mat RT = cv::Mat(), int config = -1); // Currentlty no scaled corners
        // File handling
    inline void saveToFile(std::string filePath) const;
    inline std::shared_ptr<CucomarkerMarkerDetector> readFromFile(std::string filePath);

    /* To modify (optional) */
    // Threshold
    float getEstimatedPosesDifferenceThreshold(){ return 0.6; }


    /* New */
    cucomarker::MarkerDetector getDetector(){return _cucomarkerDetector; }
    inline void setParams(std::string markersParamsPath);
    inline void setParams(std::vector<std::vector<std::string>> markersParamsContent);
    std::string getParams(){return this->_markersParamsPath; }
    inline void setColorDetectionMode(std::string colorDetectionMode);
    std::string getColorDetectionMode(){return _colorDetectionMode; }
    // Candidates
    inline std::vector<general_marker::GeneralMarker> getCandidates();



protected:
    // Detector
    cucomarker::MarkerDetector _cucomarkerDetector;
    // Config
    std::string _markersParamsPath;
    std::string _colorDetectionMode;
    std::vector<std::vector<std::string>> _markersParamsContent;

    // Configurations
    inline  std::vector<std::string> splitConfigurationsString(std::string markersParamsPath);

    // 3d corners
    //std::vector<cv::Point3f> _schemaCorners3D;
    inline  std::vector<cv::Point3f> compute3DCornersSchema(int config = -1);
};




// Class [Aruco Marker Detector] signature
const double signature = 1664895083006;





/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                                Public
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/



CucomarkerMarkerDetector::CucomarkerMarkerDetector(double markerSize, bool verbose, bool verboseDebug):GeneralMarkerDetector(markerSize, verbose, verboseDebug){
        this->_markersParamsPath = "_";
        this->_colorDetectionMode = "_";
};


std::vector<general_marker::GeneralMarker> CucomarkerMarkerDetector::detect(cv::Mat frame){

        // Erase previous detections
        this->_detectedMarkersIds.clear();
        this->_detectedMarkers.clear();

        /*~~~~~~~~~~~~~~~~~~~~~~~~~~
        Cucomarker detection
    ~~~~~~~~~~~~~~~~~~~~~~~~~~*/
        // Detection of the markers
        std::vector<cucomarker::Marker> markers = this->_cucomarkerDetector.detect(frame);
        for(uint i = 0; i < markers.size(); i++){
            std::vector<cv::Point2f> corners;
            std::vector<cv::Point3f> corners3D; // = markers[i].get3DCorners(this->_markerSize);
            for(uint j = 0; j < markers[i].size(); j++){
                corners.push_back(markers[i][j]);
            }
            general_marker::GeneralMarker newDetectedMarker = general_marker::GeneralMarker(markers[i].id, corners, corners3D, markers[i].getArea());
            newDetectedMarker.setVerboses(this->_verbose, this->_verboseDebug);
            newDetectedMarker.setContours(markers[i].contours);
            this->_detectedMarkersIds.push_back(newDetectedMarker.getId());
            this->_detectedMarkers.push_back(general_marker::GeneralMarker(newDetectedMarker));
        }


        // Computing 3D coordinates for each marker
        for(uint i = 0; i < this->_detectedMarkers.size(); i++){
            // For Customized markers we use get3DCorners()
            this->_detectedMarkers[i].setCorners3D( this->getMarkers3DCornersSchema( -1, cv::Mat(), markers[i].getConfigurationSchema()) );
            // Save configuration information
            this->_detectedMarkers[i].setConfig(markers[i].getConfigurationSchema());
        }


        return _detectedMarkers;
}





void CucomarkerMarkerDetector::print(){
        std::cout<<BMAGENTA<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~["<<RESET<<" Cucomarker Marker Detector "<<BMAGENTA<<"]~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<RESET<<std::endl;
        std::cout<<MAGENTA<<"\t- verbose: "<<RESET<<this->_verbose<<std::endl;
        std::cout<<MAGENTA<<"\t- verboseDebug: "<<RESET<<this->_verboseDebug<<std::endl;
        std::cout<<MAGENTA<<"\t- markerSize: "<<RESET<<this->_markerSize<<std::endl;
        std::cout<<MAGENTA<<"\t- detectedMarkers: "<<RESET<<std::endl;
        for(auto m: this->_detectedMarkers){
            std::cout<<"\t\t> "<<m<<std::endl;
        }
        std::cout<<CYAN<<"\t- markersParamsPath: "<<RESET<<this->_markersParamsPath<<std::endl;
        std::cout<<CYAN<<"\t- colorDetectionMode: "<<RESET<<this->_colorDetectionMode<<std::endl;
        std::cout<<BMAGENTA<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<RESET<<std::endl;
}




/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  FILES  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Files:
        // Specific
            double signature

        // General
            bool _verbose;
            bool _verboseDebug;
            double _markerSize;
            std::vector<int> _detectedMarkersIds;
            std::vector<general_marker::GeneralMarker> _detectedMarkers;

        // Specific
            // Detector
            cucomarker::MarkerDetector _cucomarkerDetector;
                // Config
                std::string _markersParamsPath;
                std::string _colorDetectionMode;
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

// Save to file
void CucomarkerMarkerDetector::saveToFile(std::string filePath) const{
        cv::FileStorage fs(filePath, cv::FileStorage::WRITE);
        fs << "signature" << signature;
        // General
        fs << "verbose" << this->_verbose;
        fs << "verboseDebug" << this->_verboseDebug;
        fs << "markerSize" << this->_markerSize;
        // Specific
        fs << "markersParamsPath" << cv::String(this->_markersParamsPath);
        fs << "colorDetectionMode" << cv::String(this->_colorDetectionMode);
        fs.release();
}

// Read from file
std::shared_ptr<CucomarkerMarkerDetector> CucomarkerMarkerDetector::readFromFile(std::string filePath){

        std::shared_ptr<CucomarkerMarkerDetector> result;

        cv::FileStorage fs(filePath, cv::FileStorage::READ);
        if((double)fs["signature"] == signature){

            // General
            bool verbose = false; if((int)fs["verbose"] == 1){ verbose = true; }
            bool verboseDebug = false; if((int)fs["verboseDebug"] == 1){ verboseDebug = true; }
            result = std::make_shared<cucomarker_marker_detector::CucomarkerMarkerDetector>((double)fs["markerSize"], verbose, verboseDebug);

            // Specific
            if((std::string)fs["markersParamsPath"] != "_")
                result->setParams((std::string)fs["markersParamsPath"]);
            if((std::string)fs["colorDetectionMode"] != "_")
                result->setColorDetectionMode((std::string)fs["colorDetectionMode"]);

        }else{
            // Exception
            // ...
        }
        fs.release();

        return result;
}



void CucomarkerMarkerDetector::setParams(std::string markersParamsPath){
        this->_markersParamsPath = markersParamsPath;

        // Are there multiple configurations?
        std::string configurationsLine = markersParamsPath;
        std::string comma = ",";

        std::vector<std::string> configurations = splitConfigurationsString(markersParamsPath);

        for(auto config: configurations){
            this->_cucomarkerDetector.loadConfigFile(config);
        }
}


void CucomarkerMarkerDetector::setParams(std::vector<std::vector<std::string>> markersParamsContent){
        this->_markersParamsPath = "LOADED";
        this->_markersParamsContent = markersParamsContent;
        for(auto markerParamsContent: markersParamsContent){
            this->_cucomarkerDetector.loadConfigFile(markerParamsContent);
        }
}



void CucomarkerMarkerDetector::setColorDetectionMode(std::string colorDetectionMode){
        this->_colorDetectionMode = colorDetectionMode;
        this->_cucomarkerDetector.setColorDetectionMode(colorDetectionMode);
}


std::vector<general_marker::GeneralMarker> CucomarkerMarkerDetector::getCandidates(){
        // Get marker candidates
        std::vector<cucomarker::Marker> candidates = this->_cucomarkerDetector.getCandidates();
        // Apply it to general_marker::GeneralMarker objects
        std::vector<general_marker::GeneralMarker> markerCandidates;
        if(candidates.size() != 0){
            /*
            // Detection of the markers
            std::vector<cucomarker::Marker> markers = this->_cucomarkerDetector.detect(frame);
            for(uint i = 0; i < markers.size(); i++){
                std::vector<cv::Point2f> corners;
                std::vector<cv::Point3f> corners3D; // = markers[i].get3DCorners(this->_markerSize);
                for(uint j = 0; j < markers[i].size(); j++){
                    corners.push_back(markers[i][j]);
                }
                general_marker::GeneralMarker newDetectedMarker = general_marker::GeneralMarker(markers[i].id, corners, corners3D, markers[i].getArea());
                newDetectedMarker.setVerboses(this->_verbose, this->_verboseDebug);
                newDetectedMarker.setContours(markers[i].contours);
                this->_detectedMarkersIds.push_back(newDetectedMarker.getId());
                this->_detectedMarkers.push_back(general_marker::GeneralMarker(newDetectedMarker));
            }

            // Computing 3D coordinates for each marker
            for(uint i = 0; i < this->_detectedMarkers.size(); i++){
                // For Customized markers we use get3DCorners()
                this->_detectedMarkers[i].setCorners3D( this->getMarkers3DCornersSchema( -1, cv::Mat(), markers[i].getConfigurationSchema()) );
                // Save configuration information
                this->_detectedMarkers[i].setConfig(markers[i].getConfigurationSchema());
            }
        */
            for(uint i = 0; i < candidates.size(); i++){
                // Saving: id, corners 2D, Area
                std::vector<cv::Point2f> corners;
                std::vector<cv::Point3f> corners3D;
                for(uint j = 0; j < candidates[i].size(); j++){
                    corners.push_back(candidates[i][j]);
                }
                general_marker::GeneralMarker candidateMarker = general_marker::GeneralMarker(candidates[i].id, corners, corners3D, candidates[i].getArea());
                // Saving: verboses
                candidateMarker.setVerboses(this->_verbose, this->_verboseDebug);
                // Saving: contours
                candidateMarker.setContours(candidates[i].contours);
                // Saving: corners3D
                candidateMarker.setCorners3D( this->getMarkers3DCornersSchema( -1, cv::Mat(), candidates[i].getConfigurationSchema()) );
                // Saving: configuration information
                candidateMarker.setConfig(candidates[i].getConfigurationSchema());
                // Push back to vector
                markerCandidates.push_back(candidateMarker);
            }
        }
        return markerCandidates;
}


std::vector<std::string> CucomarkerMarkerDetector::splitConfigurationsString(std::string markersParamsPath){

        std::string comma = ",";
        int pos = 0;
        std::vector<std::string> configurations;
        while(std::string::npos != (pos = markersParamsPath.find(comma))){
            std::string configuration = markersParamsPath.substr(0,pos);
            configurations.push_back(configuration);
            //        std::cout<<"Configuration: "<<configuration<<std::endl;
            markersParamsPath.erase(0,pos+comma.length());
        }
        configurations.push_back(markersParamsPath);
        //    std::cout<<"Configuration: "<<configurationsLine<<std::endl;

        return configurations;
}




/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                            Protected
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
std::vector<cv::Point3f> CucomarkerMarkerDetector::compute3DCornersSchema(int config){

        if(config == -1){
            // std::cout<<"Invalid config: "<<config<<std::endl;
            config = 0;
        }

        // Load 3D corners schema
        cv::FileStorage fs;
        if(this->_markersParamsPath != "LOADED"){
            // Split configurations if needed
            std::vector<std::string> configurations = splitConfigurationsString(this->_markersParamsPath);
            std::string configurationToUse = configurations[config];
            fs = cv::FileStorage(configurationToUse, cv::FileStorage::READ);
        }
        else{
            std::string yml_content;
            for (const auto& line : this->_markersParamsContent[config]) {
                yml_content += line + "\n";
            }
            // Luego, crea un FileStorage a partir de este string.
            fs = cv::FileStorage(yml_content, cv::FileStorage::READ | cv::FileStorage::MEMORY);
        }


        //    cv::FileStorage fs(configurationToUse, cv::FileStorage::READ);
        cv::FileNode corners = fs["marker"]["corners"];
        cv::FileNodeIterator it = corners.begin(), it_end = corners.end(); // Go through the node

        std::vector<cv::Point3f> corners3D;
        for (; it != it_end; ++it){
            // Read each point
            cv::FileNode pt = *it;
            cv::Point3d point;
            cv::FileNodeIterator itPt = pt.begin();
            point.x = *itPt; ++itPt;
            point.y = *itPt;
            corners3D.push_back(point);

            //std::cout<<"Point: "<<point<<std::endl;
        }

        /*
        Set the point (0,0) in the center
    */
        // Move the points to the top left corner
        // Obtain quantity
        float min_x = 0;
        float min_y = 0;
        for(auto c: corners3D){
            if((min_x == 0) || (c.x < min_x)){
                min_x = c.x;
            }
            if((min_y == 0) || (c.y < min_y)){
                min_y = c.y;
            }
        }
        // Move
        std::vector<cv::Point3f> newCorners3D;
        for(auto c: corners3D){
            newCorners3D.push_back(cv::Point3f(c.x - min_x, c.y - min_y, 0));
        }
        corners3D = newCorners3D;
        // Obtain center point now
        float max_x = -1;
        float max_y = -1;
        for(auto c: corners3D){
            if(c.x > max_x){
                max_x = c.x;
            }
            if(c.y > max_y){
                max_y = c.y;
            }
        }
        // Rest quantities to move every point
        newCorners3D.clear();
        for(auto c: corners3D){
            newCorners3D.push_back(cv::Point3f(c.y - max_y/2., c.x - max_x/2., 0));
        }

        return newCorners3D;
}


std::vector<cv::Point3f> CucomarkerMarkerDetector::getMarkers3DCornersSchema(float size, cv::Mat RT, int config){


        std::vector<cv::Point3f> corners3D, newCorners3D;
        corners3D = compute3DCornersSchema(config);


        // Size to use
        float sizeToUse = size;
        if (size == -1){
            sizeToUse = this->_markerSize;
        }



        // Rest quantities to move every point
        newCorners3D.clear();
        float scale = cv::norm(corners3D[0]-corners3D[1])/sizeToUse;
        for(auto c: corners3D){
            newCorners3D.push_back(c/scale);
        }


        // APPLY RT?
        if (!RT.empty()){
            cv::Mat m = RT.clone();
            //aruco_mm::mult<cv::Point3f>(rt_g2m,newCorners3D);
            if ( m.type() ==CV_32F ) {
                const float *ptr=m.ptr<float> ( 0 );
                cv::Point3f res;
                for ( auto & p :  newCorners3D ) {
                    res.x= ptr[0]*p.x +ptr[1]*p.y +ptr[2]*p.z+ptr[3];
                    res.y= ptr[4]*p.x +ptr[5]*p.y +ptr[6]*p.z+ptr[7];
                    res.z= ptr[8]*p.x +ptr[9]*p.y +ptr[10]*p.z+ptr[11];
                    p=res;
                }
            } else 	if ( m.type() ==CV_64F ) {
                const double *ptr=m.ptr<double> ( 0 );
                cv::Point3f res;
                for ( auto & p :  newCorners3D ) {
                    res.x= ptr[0]*p.x +ptr[1]*p.y +ptr[2]*p.z+ptr[3];
                    res.y= ptr[4]*p.x +ptr[5]*p.y +ptr[6]*p.z+ptr[7];
                    res.z= ptr[8]*p.x +ptr[9]*p.y +ptr[10]*p.z+ptr[11];
                    p=res;
                }
            }
        }



        corners3D = newCorners3D;

        //    for(auto c: corners3D){
        //        std::cout<<c<<std::endl;
        //    }

        return corners3D;
}

}

#endif
