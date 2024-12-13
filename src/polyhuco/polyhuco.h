#ifndef _POLYHUCO_
#define _POLYHUCO_

#include <vector>
#include <iostream>
#include <cstdlib>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include "opencv2/highgui.hpp"

#include "../utils/mapProcessor.hpp"
#include "../utils/frameDetector.hpp"
#include "../utils/generalMarker.hpp"

#include "../utils/colors.hpp"
#include "../utils/utils.hpp"
#include "levmarq.h"

namespace polyhuco {


struct agruppedCornersInfo{
    std::vector<std::vector<cv::Point2f>> agruppedCorners2D;
    std::vector<std::vector<cv::Point3f>> agruppedCorners3D;
    std::vector<cv::Point2f> corners2D;
    std::vector<cv::Point3f> corners3D;
};


struct checkMarkerInfo
{
    bool correct;
    std::string reason;
};



struct guessedInformation{
    // std::vector<int> ids;
    // std::vector<std::vector<cv::Point3f>> corners3D;
    // std::vector<double> angles;
    std::vector<general_marker::GeneralMarker> guessedMarkers;
};


struct detectionInformationModule{
    // Pose
    cv::Mat rvec;
    cv::Mat tvec;
    // Mean metrics
    double avgArea;
    double avgAngle;
    // Approved ?
    bool poseApproved;
};
struct detectionInformation{
    /* Modules */
    // woRefinement
    detectionInformationModule woRefinement;
    // wRefinement
    detectionInformationModule wRefinement;
    // Selected pose
    detectionInformationModule selectedPose;
    bool poseIsSelected;
    std::string selectedPoseId;
    /* General information */
    // Camera parameters
    cv::Mat K;
    cv::Mat D;
    // Detection info
    frame_detector::FrameDetector frameDetector;
    int detectionType;
    // Detected markers
    std::vector<int> foundMarkersIds; 
    std::vector<general_marker::GeneralMarker> detectedMarkers;
};  


/**
 * @brief Auxiliar structure that allows you to return the 
 * necessary variables after searching in a frame.
 * 
 */
struct foundInFrame{
    bool found;
    cv::Mat outputFrame;
    detectionInformation detectionInfo;
    int drawingType;
    int lookForMarkers;
    guessedInformation guessedInfo;
};



/*~~~~~~~~~~~~~~~
    POLYHUCO
~~~~~~~~~~~~~~~*/
/**
 * @brief Polyhuco class represents all the related information of a polyhedron formed with
 * customized fiducial markers in every side.
 * 
 */
class Polyhuco {
    public:
        /*~~~~~~~~~~~~~~~~
            Attributes
        ~~~~~~~~~~~~~~~~*/
        // Detector
        frame_detector::FrameDetector _frameDetector;
        // Map
        map_processor::MapProcessor _mapProcessor;


        /*~~~~~~~~~~~~~~
            Methods
        ~~~~~~~~~~~~~~*/
        // ...
        inline Polyhuco(){};
        Polyhuco(cv::String polyhucoName,
            int numFaces,
            cv::String markersTypeName,
            double markersSize,
            int detectionType,
            cv::String markersParamPath,
            cv::String mappedPolyhedronMapPath,
            bool verbose = false,
            bool verboseDebug = false,
            bool verboseFrame = false,
            int solvePNPMethod = 0,
            double subPixSize = 10,
            double subPixRep = 50,
            double subPixEpsilon = 0.001,
            int id_min = -1,
            int id_max = -1,
            std::string distancesOutputPath = "-",
            double scaleForMap = 1,
            bool checkCandidates = false
            );

        void loadFromContent(const std::vector<std::vector<std::string>> &markersConfig,
                            const std::unordered_map<int,std::vector<cv::Point3f>> &markers);
            std::vector<std::vector<std::string>> _markersConfig;
            std::unordered_map<int,std::vector<cv::Point3f>> _markers;
        // ...
        void printInfo();

        /**
         * @brief ...
         * 
         * @param frame ...
         * @param camParamsPath ...
         * @param detectionType Detection type
         *      Types:
         *          - 1: Cucomarker
         *          - 2: Aruco
         * @param drawingDetectionType Drawing detection type
         *      Types:
         *          - 0: No drawing
         *          - 1: Normal drawing
         *          - 2: 3D drawing
         * @return foundInFrame ...
         */
        foundInFrame findInFrame(cv::Mat frame, cv::String camParamsPath = "-", int drawingDetectionType = 1, int lookForMarkers = 0, bool applyRefinement = false, cv::Mat K_Mat = {}, cv::Mat D_Mat = {});
        void drawGuessedMarkers(bool wRefinement);
        inline double getMarkerSize(){ return this->_markersSize; };
        detectionInformation getDetectionInfo();
        map_processor::MappedMarkerInformation getMappedMarkerInformation3D(int id);

        inline void setThresAngle(double thresAngle){this->_thresAngle = thresAngle;};
        inline void setThresNDet(double thresNDet){this->_thresNDet = thresNDet;};
        inline void setThresArea(double thresArea){this->_thresArea = thresArea;};
        inline void setThresDiff(double thresDiff){this->_thresDiff = thresDiff;};

        inline void setVerboses(bool verbose, bool verboseDebug, bool verboseFrame){this->_verbose = verbose; this->_verboseDebug = verboseDebug; this->_verboseFrame=verboseFrame;};
        inline void setSolvePNPMethod(int solvePNPMethod){this->_solvePNPMethod = solvePNPMethod; };
        inline void disableThresAngle(){this->_enableThresAngle = false; };

        int _id_min;
        int _id_max;
        // Map
        cv::String _mappedPolyhedronMapPath;
        void processMap();
    private:
        /*~~~~~~~~~~~~~~~~
            Attributes
        ~~~~~~~~~~~~~~~~*/
        // Verboses
        bool _verbose;
        bool _verboseDebug;
        bool _verboseFrame;
        // General
        cv::String _polyhucoName;
        int _numFaces;
        std::string _distancesOutputsPath;
        // Refinement filtering
        double lastDistanceToObject;
        double lastAngleToObject;
        // Thresholds
        double _thresAngle;
        bool _enableThresAngle;
        double _thresNDet;
        double _thresArea;
        double _thresDiff;
        // solvePNPMethod
        int _solvePNPMethod;
        // CornerSubPix Refinement options
        double _subPixSize;
        double _subPixRep;
        double _subPixEpsilon;
        // Check candidates
        bool _checkCandidates;
        // Markers
        cv::String _markersTypeName;
        double _markersSize;
        cv::String _markersParamPath;
        int _detectionType;
        // Map
        double _scaleForMap;
        // Found information
        foundInFrame _foundInformation;
        // Styles
        // Detected
            cv::Scalar _innerColor; cv::Scalar _innerColorError;
            cv::Scalar _outerColor; cv::Scalar _outerColorError;
            double _innerWeight; double _innerWeightError;
            double _outerWeight; double _outerWeightError;
            cv::Scalar _normalColor;
        // Guessed
            cv::Scalar _innerColorGuessed; cv::Scalar _innerColorErrorGuessed;
            cv::Scalar _outerColorGuessed; cv::Scalar _outerColorErrorGuessed;
            double _innerWeightGuessed; double _innerWeightErrorGuessed;
            double _outerWeightGuessed; double _outerWeightErrorGuessed;
        /*~~~~~~~~~~~~~~
            Methods
        ~~~~~~~~~~~~~~*/
        void lookForMarkers();
        void applyRefinement();
        void drawDetections(bool wRefinement);
        void drawGuessedMarker(cv::Mat &frame, int noDetectedId, double angle, std::vector<cv::Point3f> corners3D, cv::Mat rvec, cv::Mat tvec, cv::Mat K, cv::Mat D, cv::Scalar guessedColor, cv::Scalar guessedColorFrame, bool draw3D);
        void computePose(std::vector<cv::Point2f> corners2D, std::vector<cv::Point3f> corners3D, bool applyLookForMarkers, bool isfirstCallBeforeRefinement = false);
        void computeMarkersAngles(bool wRefinement);

        void drawMarkers(cv::Mat &image);
        void drawMarkers3D(cv::Mat &image, std::vector<cv::Point2f> elevatedCenters);
        void drawMarkersNormal(cv::Mat &image, std::vector<std::vector<cv::Point2f>> markersAxes);
        
        checkMarkerInfo checkMarker(general_marker::GeneralMarker marker);


        template <typename T>
        void refineEstimatedPose(cv::Mat& rvec, cv::Mat& tvec, cv::Mat K, cv::Mat D);
            // Auxiliar functions
            agruppedCornersInfo obtainAgruppedCorners3D();
            std::vector<cv::Point2f> obtainSelectedPoints(cv::Mat img, cv::Mat rvec, cv::Mat tvec, cv::Mat K, cv::Mat D);
            std::vector<double> obtainQualityFromSelectedPoints(std::vector<cv::Point2f> selectedPoints, cv::Mat laplacianImg);
            std::vector<double> obtainDerivativesErrorFromSolution(cv::Mat img, cv::Mat rvec, cv::Mat tvec, cv::Mat K, cv::Mat D);

        // Checking pose
        void checkSinglePose(polyhuco::detectionInformationModule &poseModule);
};



}

#endif