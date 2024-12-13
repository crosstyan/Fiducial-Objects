#ifndef _FRAME_DETECTOR_
#define _FRAME_DETECTOR_

#include<cstdlib>

#include "generalMarker.hpp"
#include "markerDetectors/cucomarkerMarkerDetector.hpp"
namespace frame_detector {

/**
 * @brief Class that represents the related information of several detected markers of a single frame
 * 
 */
class FrameDetector {
    public:
        inline FrameDetector(){};
        /**
         * @brief Construct a new Frame Detector object
         * 
         * @param markerSize ...
         * @param detectionType Detection type
         *      Types:
         *          - 1: Cucomarker
         *          - 2: Aruco
         * @param verbose ...
         * @param verboseDebug ...
         */
        FrameDetector(double markerSize, int detectionType = 1, bool verbose = false, bool verboseDebug = false, bool applyRefinement = true);
        void setConfiguration(std::string markersParamsPath);
        void setConfiguration(std::vector<std::vector<std::string>> markersConfig);
        std::vector<general_marker::GeneralMarker> detect(cv::Mat frame);
        inline std::vector<int> getDetectedIds(){ return this->_detectedMarkersIds; };
        inline void setDetectedMarkers(std::vector<general_marker::GeneralMarker> newMarkers){ this->_detectedMarkers = newMarkers; };
        inline std::vector<general_marker::GeneralMarker> getDetectedMarkers(){ return this->_detectedMarkers; };
        general_marker::GeneralMarker getGeneralMarkerById(int id);
        inline std::vector<general_marker::GeneralMarker> getCandidates(){return this->_markerCandidates;};
    private:
        bool _verbose;
        bool _verboseDebug;
        double _markerSize;
        int _detectionType;
        cv::Mat _frameOfDetection;
        // Configuration
        cv::String _markersParamsPath;
        // Detectors
        cucomarker_marker_detector::CucomarkerMarkerDetector *_cucomarkerDetector;
        // Detections
        std::vector<int> _detectedMarkersIds;
        std::vector<general_marker::GeneralMarker> _detectedMarkers;
        bool _applyRefinement;
        std::vector<general_marker::GeneralMarker> _markerCandidates;
};


}

#endif