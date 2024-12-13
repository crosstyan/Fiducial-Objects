#ifndef _CucomarkerMarkerDetector_H_
#define _CucomarkerMarkerDetector_H_

#include <cstdlib>
#include <vector>

#include "../generalMarker.hpp"
#include "../generalMarkerDetector.hpp"
#include "cucomarker/cucomarker.h"


// Eliminar inlcudes -----------
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
//----------------


namespace aruco_mm {

/**
 * @brief Class that represents the related information of several detected markers of a single frame
 * 
 */
class CucomarkerMarkerDetector: public general_marker_detector::GeneralMarkerDetector {
    public:
        // Constructor
        inline CucomarkerMarkerDetector():GeneralMarkerDetector(){};
        CucomarkerMarkerDetector(double markerSize, bool verbose = false, bool verboseDebug = false);
        
        // To implement
            // Print
            void print();
            // Detection
            std::vector<general_marker::GeneralMarker> detect(cv::Mat frame);
            // Computing 3D corners
            std::vector<cv::Point3f> getMarkers3DCornersSchema(float size = -1, cv::Mat RT = cv::Mat(), int config = -1); // Currentlty no scaled corners
            // File handling
            void saveToFile(std::string filePath) const;
            std::shared_ptr<CucomarkerMarkerDetector> readFromFile(std::string filePath);

        // To modify (optional)
            // Threshold
            inline float getEstimatedPosesDifferenceThreshold(){ return 0.6; };


        // New
        inline cucomarker::MarkerDetector getDetector(){return this->_cucomarkerDetector; };
        void setParams(std::string markersParamsPath);
        inline std::string getParams(){return this->_markersParamsPath; };
        void setColorDetectionMode(std::string colorDetectionMode);
        inline std::string getColorDetectionMode(){return this->_colorDetectionMode; };



    protected:
        // Detector
        cucomarker::MarkerDetector _cucomarkerDetector;
            // Config
            std::string _markersParamsPath;
            std::string _colorDetectionMode;

        // Configurations
        std::vector<std::string> splitConfigurationsString(std::string markersParamsPath);

        // 3d corners
        //std::vector<cv::Point3f> _schemaCorners3D;
        std::vector<cv::Point3f> compute3DCornersSchema(int config = -1);
};


}

#endif
