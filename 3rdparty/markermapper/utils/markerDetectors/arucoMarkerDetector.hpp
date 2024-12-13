#ifndef _ArucoMarkerDetector_H_
#define _ArucoMarkerDetector_H_

#include<cstdlib>

#include "../generalMarker.hpp"
#include "../generalMarkerDetector.hpp"
#include <aruco/aruco.h>


namespace aruco_marker_detector {

/**
 * @brief Class that represents the related information of several detected markers of a single frame
 * 
 */
class ArucoMarkerDetector: public general_marker_detector::GeneralMarkerDetector {
    public:
        // Constructor
        inline ArucoMarkerDetector():GeneralMarkerDetector(){};
        ArucoMarkerDetector(double markerSize, bool verbose = false, bool verboseDebug = false);
        
        // To implement
            // Print
            void print();
            // Detection
            std::vector<general_marker::GeneralMarker> detect(cv::Mat frame);
            // File handling
            void saveToFile(std::string filePath) const;
            std::shared_ptr<ArucoMarkerDetector> readFromFile(std::string filePath);

        // To modify (optional)
        inline std::string getMarkerLabelerDict(){ return this->_markersDictPath; };
        inline float getEstimatedPosesDifferenceThreshold(){ return 0.6; };

        // New
        inline aruco::MarkerDetector getDetector(){return this->_arucoDetector; };
        void setParams(std::string markersParamsPath);
        inline std::string setParams(){return this->_markersParamsPath; };
        void setDict(std::string markersDictPath);
        inline std::string setDict(){return this->_markersDictPath; };


        // Computing 3D corners
        std::vector<cv::Point3f> getMarkers3DCornersSchema(float size = -1, cv::Mat RT = cv::Mat(), int config = 0);

    protected:
        // Detector
        aruco::MarkerDetector _arucoDetector;
            // Config
            std::string _markersParamsPath;
            std::string _markersDictPath;
        
};


}

#endif
