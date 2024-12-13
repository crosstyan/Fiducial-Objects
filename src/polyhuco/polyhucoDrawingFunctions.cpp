#ifndef _POLYHUCO_DRAWING_FUNCTIONS_CPP_
#define _POLYHUCO_DRAWING_FUNCTIONS_CPP_

#include "polyhuco.h"
#include "polyhucoPoseFunctions.hpp"

namespace polyhuco {


void Polyhuco::drawMarkers(cv::Mat &image){
    for(int i = 0; i < this->_foundInformation.detectionInfo.detectedMarkers.size(); i++){
        // Marker
        general_marker::GeneralMarker marker = this->_foundInformation.detectionInfo.detectedMarkers[i];
        checkMarkerInfo checkInfo = this->checkMarker(marker);
        // Printing
        if(checkInfo.correct){
            marker.drawMarker(image,
            this->_innerColor,
            this->_outerColor,
            this->_innerWeight,
            this->_outerWeight,
            "",
            this->_verboseFrame);
        }
        else{
            marker.drawMarker(image,
            this->_innerColorError,
            this->_outerColorError,
            this->_innerWeightError,
            this->_outerWeightError,
            checkInfo.reason,
            this->_verboseFrame);
        }
    }
}

void Polyhuco::drawMarkers3D(cv::Mat &image, std::vector<cv::Point2f> elevatedCenters){
    for(int i = 0; i < this->_foundInformation.detectionInfo.detectedMarkers.size(); i++){        
        // Marker
        general_marker::GeneralMarker marker = this->_foundInformation.detectionInfo.detectedMarkers[i];
        checkMarkerInfo checkInfo = this->checkMarker(marker);
        // Printing
        if(checkInfo.correct){
            marker.drawMarker3D(image, elevatedCenters[i],
            this->_innerColor,
            this->_outerColor,
            this->_innerWeight,
            this->_outerWeight,
            "",
            this->_verboseFrame);
        }
        else{
            marker.drawMarker3D(image, elevatedCenters[i],
            this->_innerColorError,
            this->_outerColorError,
            this->_innerWeightError,
            this->_outerWeightError,
            checkInfo.reason,
            this->_verboseFrame);
        }
    }
}

void Polyhuco::drawMarkersNormal(cv::Mat &image, std::vector<std::vector<cv::Point2f>> markersAxes){      
    for(int i = 0; i < this->_foundInformation.detectionInfo.detectedMarkers.size(); i++){    
        // Marker
        general_marker::GeneralMarker marker = this->_foundInformation.detectionInfo.detectedMarkers[i];
        checkMarkerInfo checkInfo = this->checkMarker(marker);
        // Printing
        if(checkInfo.correct){
            marker.drawMarkerNormal(image, markersAxes[i],
            this->_innerColor,
            this->_outerColor,
            this->_innerWeight,
            this->_outerWeight,
            this->_normalColor,
            "",
            this->_verboseFrame);
        }
        else{
            marker.drawMarkerNormal(image, markersAxes[i],
            this->_innerColorError,
            this->_outerColorError,
            this->_innerWeightError,
            this->_outerWeightError,
            this->_normalColor,
            checkInfo.reason,
            this->_verboseFrame);
        }
    }
}


void Polyhuco::drawGuessedMarkers(bool wRefinement){
    // Pose to use
    cv::Mat rvec;
    cv::Mat tvec;
    if(wRefinement){
        rvec = this->_foundInformation.detectionInfo.wRefinement.rvec.clone();
        tvec = this->_foundInformation.detectionInfo.wRefinement.tvec.clone();
    }
    else{
        rvec = this->_foundInformation.detectionInfo.woRefinement.rvec.clone();
        tvec = this->_foundInformation.detectionInfo.woRefinement.tvec.clone();
    }

    // // Processing
    // for(int i = 0; i < this->_foundInformation.guessedInfo.guessedMarkers.size(); i++){
    //     // Variables
    //     int noDetectedId = this->_foundInformation.guessedInfo.guessedMarkers[i].getId();
    //     std::vector<cv::Point3f> corners3D = this->_foundInformation.guessedInfo.guessedMarkers[i].getCorners3D();
    //     double angle = this->_foundInformation.guessedInfo.guessedMarkers[i].getAngle();
    //     // Processing
    //     if(this->_foundInformation.lookForMarkers != 2){
    //         cv::Scalar guessedColor; cv::Scalar guessedColorFrame;
    //         guessedColor = cv::Scalar(3, 252, 173);
    //         guessedColorFrame = cv::Scalar(2, 201, 138);
    //         bool draw3D = false;
    //         if(this->_foundInformation.lookForMarkers == 3){
    //             draw3D = true;
    //         }
    //         drawGuessedMarker(this->_foundInformation.outputFrame, noDetectedId, angle, corners3D, 
    //             rvec, 
    //             tvec, 
    //             this->_foundInformation.detectionInfo.K, 
    //             this->_foundInformation.detectionInfo.D, 
    //             guessedColor, guessedColorFrame, draw3D);
    //     }
    //     else if(this->_foundInformation.lookForMarkers == 2){ // BAD MARKER TO GUESS
    //         cv::Scalar guessedColor; cv::Scalar guessedColorFrame;
    //         guessedColor = cv::Scalar(72, 2, 247);
    //         guessedColorFrame = cv::Scalar(55, 2, 186);
    //         bool draw3D = false;
    //         drawGuessedMarker(this->_foundInformation.outputFrame, noDetectedId, angle, corners3D, 
    //             rvec, 
    //             tvec, 
    //             this->_foundInformation.detectionInfo.K, 
    //             this->_foundInformation.detectionInfo.D, 
    //             guessedColor, guessedColorFrame, draw3D);
    //     }
    // }

    // New painting process for guessed markers
    for(int i = 0; i < this->_foundInformation.guessedInfo.guessedMarkers.size(); i++){
        // Marker
        general_marker::GeneralMarker marker = this->_foundInformation.guessedInfo.guessedMarkers[i];

        // 3D
        bool draw3D = false;
        if(this->_foundInformation.lookForMarkers == 3){
            draw3D = true;
        }
        cv::Point2f elevatedCenter;
        if(draw3D){
            cv::Point3f elevatedCenter3D = obtainCenter3D(marker.getCorners3D());
            elevatedCenter3D = elevatedCenter3D*1.5;
            std::vector<cv::Point3f> elevatedCenter3DVector = {elevatedCenter3D};
            std::vector<cv::Point2f> elevatedCenter2DVector;
            cv::projectPoints(elevatedCenter3DVector, rvec, tvec, this->_foundInformation.detectionInfo.K, this->_foundInformation.detectionInfo.D, elevatedCenter2DVector);
            elevatedCenter = elevatedCenter2DVector[0];
        }

        // Correct/Incorrect
        checkMarkerInfo checkInfo = this->checkMarker(marker);
        // Printing
        if(checkInfo.correct){
            if(draw3D){
                marker.drawMarker3D(this->_foundInformation.outputFrame, elevatedCenter,
                this->_innerColorGuessed,
                this->_outerColorGuessed,
                this->_innerWeightGuessed,
                this->_outerWeightGuessed,
                "",
                this->_verboseFrame);
            }
            else{
                marker.drawMarker(this->_foundInformation.outputFrame,
                this->_innerColorGuessed,
                this->_outerColorGuessed,
                this->_innerWeightGuessed,
                this->_outerWeightGuessed,
                "",
                this->_verboseFrame);
            }
        }
        else{
            if(draw3D){
                marker.drawMarker3D(this->_foundInformation.outputFrame, elevatedCenter,
                this->_innerColorErrorGuessed,
                this->_outerColorErrorGuessed,
                this->_innerWeightErrorGuessed,
                this->_outerWeightErrorGuessed,
                checkInfo.reason,
                this->_verboseFrame);
            }
            else{
                marker.drawMarker(this->_foundInformation.outputFrame,
                this->_innerColorErrorGuessed,
                this->_outerColorErrorGuessed,
                this->_innerWeightErrorGuessed,
                this->_outerWeightErrorGuessed,
                checkInfo.reason,
                this->_verboseFrame);
            }
        }
    }
}




void Polyhuco::drawDetections(bool wRefinement){
    // Pose to use
    cv::Mat rvec;
    cv::Mat tvec;
    if(wRefinement){
        rvec = this->_foundInformation.detectionInfo.wRefinement.rvec.clone();
        tvec = this->_foundInformation.detectionInfo.wRefinement.tvec.clone();
    }
    else{
        rvec = this->_foundInformation.detectionInfo.woRefinement.rvec.clone();
        tvec = this->_foundInformation.detectionInfo.woRefinement.tvec.clone();
    }

    // Centre of the polyhedron
    std::vector<cv::Point3f> centre3D = {cv::Point3f(0,0,0)};
    std::vector<cv::Point2f> centre2D;
    cv::projectPoints(centre3D, 
        rvec,
        tvec,
        this->_foundInformation.detectionInfo.K, 
        this->_foundInformation.detectionInfo.D, centre2D);
    // Draw axes
    std::vector<cv::Point3f> exes3D = {cv::Point3f(0,this->_markersSize,0), cv::Point3f(this->_markersSize,0,0), cv::Point3f(0,0,this->_markersSize)};
    std::vector<cv::Point2f> exes2D;
    cv::projectPoints(exes3D,
        rvec,
        tvec,
        this->_foundInformation.detectionInfo.K,
        this->_foundInformation.detectionInfo.D, exes2D);
    cv::line(this->_foundInformation.outputFrame, exes2D[0], centre2D[0], cv::Scalar(0,0,255), 2);
    cv::line(this->_foundInformation.outputFrame, exes2D[1], centre2D[0], cv::Scalar(0,255,0), 2);
    cv::line(this->_foundInformation.outputFrame, exes2D[2], centre2D[0], cv::Scalar(255,0,0), 2);
    // Draw each marker on the frame
    if(this->_foundInformation.drawingType == 0){
        // No drawing
    }
    else if(this->_foundInformation.drawingType == 1){
        // 2D drawing
        this->drawMarkers(this->_foundInformation.outputFrame);
    }
    else if(this->_foundInformation.drawingType == 2){
        // 3D Drawing, with cucomarker 3d corners
        if(this->_foundInformation.detectionInfo.detectionType == 1){
            std::vector<cv::Point2f> elevatedCenters;
            for(int m = 0; m < this->_foundInformation.detectionInfo.detectedMarkers.size(); m++){
                std::vector<cv::Point2f> corners2D = this->_foundInformation.detectionInfo.detectedMarkers[m].getCorners(); 
                std::vector<cv::Point3f> corners3D = this->_foundInformation.detectionInfo.detectedMarkers[m].getCorners3D();
                cv::Point3f centre3D = this->_foundInformation.detectionInfo.detectedMarkers[m].getCenter3D();
                cv::Mat rvec, tvec;
                cv::solvePnP(corners3D, corners2D, this->_foundInformation.detectionInfo.K, this->_foundInformation.detectionInfo.D, rvec, tvec);
                // Estimate pose of elevated center
                std::vector<cv::Point3f> elevatedCentre3D = {cv::Point3f(centre3D.x*1.5, centre3D.y*1.5, centre3D.z*1.5)};
                std::vector<cv::Point2f> elevatedCentre2D;
                cv::projectPoints(elevatedCentre3D, rvec, tvec, this->_foundInformation.detectionInfo.K, this->_foundInformation.detectionInfo.D, elevatedCentre2D);
                elevatedCenters.push_back(elevatedCentre2D[0]);
            }
            // Drawing
            this->drawMarkers3D(this->_foundInformation.outputFrame, elevatedCenters);
        }
        // Aruco detections drawing, single piramid, local pose
        if(this->_foundInformation.detectionInfo.detectionType == 2){
            // Get elevated centers
            std::vector<cv::Point2f> elevatedCenters;
            for(int m = 0; m < this->_foundInformation.detectionInfo.detectedMarkers.size(); m++){
                std::vector<cv::Point2f> corners2D = this->_foundInformation.detectionInfo.detectedMarkers[m].getCorners(); 
                std::vector<cv::Point3f> corners3D = this->_foundInformation.detectionInfo.detectedMarkers[m].getCorners3D();
                cv::Point3f centre3D = this->_foundInformation.detectionInfo.detectedMarkers[m].getCenter3D();
                cv::Mat rvec, tvec;
                cv::solvePnP(corners3D, corners2D, this->_foundInformation.detectionInfo.K, this->_foundInformation.detectionInfo.D, rvec, tvec);
                // Estimate pose of elevated center
                std::vector<cv::Point3f> elevatedCentre3D = {cv::Point3f(centre3D.x*1.5, centre3D.y*1.5, centre3D.z*1.5)};
                std::vector<cv::Point2f> elevatedCentre2D;
                cv::projectPoints(elevatedCentre3D, rvec, tvec, this->_foundInformation.detectionInfo.K, this->_foundInformation.detectionInfo.D, elevatedCentre2D);
                elevatedCenters.push_back(elevatedCentre2D[0]);
            }
            // Drawing
            this->drawMarkers3D(this->_foundInformation.outputFrame, elevatedCenters);
        }
    }
    else if(this->_foundInformation.drawingType == 3){
        std::vector<std::vector<cv::Point2f>> markersAxesPoints2D;
        // 3D Drawing, with global figure exes
        for(int m = 0; m < this->_foundInformation.detectionInfo.detectedMarkers.size(); m++){
            // Extract corners
            map_processor::MappedMarkerInformation mappedMarkerInfo = this->_mapProcessor.getMarkerInformation(this->_foundInformation.detectionInfo.detectedMarkers[m].getId());
            std::vector<cv::Point3f> corners3D;
            for(int c = 0; c < mappedMarkerInfo.corners.size(); c++){
                corners3D.push_back(mappedMarkerInfo.corners[c]);
            }
            /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                --- Computing the perpendicular, then move it ---
            ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
            // Perpendicular
            // centre of marker * 2
            cv::Point3f markerCenter = obtainCenter3D(mappedMarkerInfo.corners);
            cv::Point3f elevatedMarkerCenter = cv::Point3f(markerCenter.x*1.5, markerCenter.y*1.5, markerCenter.z*1.5);
            cv::Point3f axis = elevatedMarkerCenter;
            std::cout<<"Axis: "<<axis<<std::endl;
            // Computing the pose
            std::vector<cv::Point3f> axesPoints3D; axesPoints3D.push_back(axis);
            std::vector<cv::Point2f> axesPoints2D;
            cv::projectPoints(axesPoints3D,
                rvec,
                tvec,
                this->_foundInformation.detectionInfo.K,
                this->_foundInformation.detectionInfo.D,
                axesPoints2D);
            markersAxesPoints2D.push_back(axesPoints2D);
        }
        // Drawing the exes on each marker
        this->drawMarkersNormal(this->_foundInformation.outputFrame, markersAxesPoints2D);
    }
    else if(this->_foundInformation.drawingType == 4){
        std::vector<cv::Point2f> elevatedCenters;
        // 3D Drawing, with global figure exes
        for(int m = 0; m < this->_foundInformation.detectionInfo.detectedMarkers.size(); m++){
            // Extract corners
            map_processor::MappedMarkerInformation mappedMarkerInfo = this->_mapProcessor.getMarkerInformation(this->_foundInformation.detectionInfo.detectedMarkers[m].getId());
            std::vector<cv::Point3f> corners3D;
            for(int c = 0; c < mappedMarkerInfo.corners.size(); c++){
                corners3D.push_back(mappedMarkerInfo.corners[c]);
            }
            /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                Computing the perpendicular
            ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
            cv::Point3f markerCenter = obtainCenter3D(mappedMarkerInfo.corners);
            cv::Point3f elevatedMarkerCenter = cv::Point3f(markerCenter.x*1.5, markerCenter.y*1.5, markerCenter.z*1.5);
            std::vector<cv::Point3f> elevatedCentre3D = {elevatedMarkerCenter};
            std::vector<cv::Point2f> elevatedCentre2D;
            cv::projectPoints(elevatedCentre3D,
                rvec,
                tvec,
                this->_foundInformation.detectionInfo.K,
                this->_foundInformation.detectionInfo.D,
                elevatedCentre2D);
            elevatedCenters.push_back(elevatedCentre2D[0]);
        }
        // Drawing
        this->drawMarkers3D(this->_foundInformation.outputFrame, elevatedCenters);
    }
}



void Polyhuco::drawGuessedMarker(cv::Mat &frame, int noDetectedId, double angle, std::vector<cv::Point3f> corners3D, cv::Mat rvec, cv::Mat tvec, cv::Mat K, cv::Mat D, cv::Scalar guessedColor, cv::Scalar guessedColorFrame, bool draw3D){
    // Guess center
    std::vector<cv::Point2f> guessedCorners2D; cv::projectPoints(corners3D, rvec, tvec, K, D, guessedCorners2D);
    cv::Point2f center2D = obtainCenter2D(guessedCorners2D);
    if(this->_verboseDebug){
        std::cout<<"Guessed center 2d: "<<center2D<<std::endl;
    }
    cv::Point2f elevatedCenter;
    if(draw3D){
        cv::Point3f elevatedCenter3D = obtainCenter3D(corners3D);
        elevatedCenter3D = elevatedCenter3D*1.5;
        std::vector<cv::Point3f> elevatedCenter3DVector = {elevatedCenter3D};
        std::vector<cv::Point2f> elevatedCenter2DVector;
        cv::projectPoints(elevatedCenter3DVector, rvec, tvec, K, D, elevatedCenter2DVector);
        elevatedCenter = elevatedCenter2DVector[0];
    }
    // Start drawing
    std::vector<cv::Point2f> corners2D;
    cv::projectPoints(corners3D, rvec, tvec, K, D, corners2D);

    general_marker::GeneralMarker guessedMarker(noDetectedId, corners2D);
        guessedMarker.setName("Guessed marker");
        guessedMarker.setAngle(getAngleFromCorners3D(corners3D, rvec, tvec));
    checkMarkerInfo checkInfo = checkMarker(guessedMarker);
    if(draw3D){
        guessedMarker.drawMarker3D(frame, elevatedCenter,
            this->_innerColorGuessed,
            this->_outerColorGuessed,
            this->_innerWeightGuessed,
            this->_outerWeightGuessed,
            "Guessed marker",
            this->_verboseFrame);
    }
    else{
        if(checkInfo.correct){
            guessedMarker.drawMarker(frame,
            this->_innerColorGuessed,
            this->_outerColorGuessed,
            this->_innerWeightGuessed,
            this->_outerWeightGuessed,
            "",
            this->_verboseFrame);
        }
        else{
            guessedMarker.drawMarker(frame,
            this->_innerColorErrorGuessed,
            this->_outerColorErrorGuessed,
            this->_innerWeightErrorGuessed,
            this->_outerWeightErrorGuessed,
            checkInfo.reason,
            this->_verboseFrame);
        }
    }
}


}

#endif