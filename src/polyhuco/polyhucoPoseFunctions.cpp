#ifndef _POLYHUCO_POSE_FUNCTIONS_CPP_
#define _POLYHUCO_POSE_FUNCTIONS_CPP_

#include "polyhucoPoseFunctions.hpp"

cv::Vec3f applyPoseToVec(cv::Vec3f point, cv::Mat rvec, cv::Mat tvec){
    cv::Mat pointMat = cv::Mat(point); pointMat.convertTo(pointMat, CV_64F); //std::cout<<"pointMat: "<<pointMat<<std::endl;
    cv::Mat sum = pointMat+tvec; //std::cout<<"Sum: "<<sum<<std::endl;
    cv::Mat rotationMatrix; cv::Rodrigues(rvec, rotationMatrix); //std::cout<<"rotationMatrix: "<<rotationMatrix<<std::endl;
    cv::Mat mul = rotationMatrix*sum; //std::cout<<"Mul: "<<mul<<std::endl;
    point = cv::Vec3f(mul);
    return point;
}

cv::Vec3f worldToCamera(cv::Vec3f vecc, cv::Mat rvec, cv::Mat tvec){

    //
    cv::Vec4f vec(vecc[0], vecc[1], vecc[2], 1);
    std::cout<<"V: "<<vecc<<std::endl;
    cv::Mat pointMat = cv::Mat(vec); pointMat.convertTo(pointMat, CV_64F);
    std::cout<<"PM: "<<pointMat<<std::endl;

    //
    cv::Mat m33; cv::Rodrigues(rvec, m33);
    cv::Mat m(cv::Size(4,4), CV_64F);
    m.at<double>(0,0) = m33.at<double>(0,0);
    m.at<double>(0,1) = m33.at<double>(0,1);
    m.at<double>(0,2) = m33.at<double>(0,2);
    m.at<double>(0,3) = tvec.at<double>(0,0);
    m.at<double>(1,0) = m33.at<double>(1,0);
    m.at<double>(1,1) = m33.at<double>(1,1);
    m.at<double>(1,2) = m33.at<double>(1,2);
    m.at<double>(1,3) = tvec.at<double>(0,1);
    m.at<double>(2,0) = m33.at<double>(2,0);
    m.at<double>(2,1) = m33.at<double>(2,1);
    m.at<double>(2,2) = m33.at<double>(2,2);
    m.at<double>(2,3) = tvec.at<double>(0,2);
    m.at<double>(3,0) = 0.0;
    m.at<double>(3,1) = 0.0;
    m.at<double>(3,2) = 0.0;
    m.at<double>(3,3) = 1.0;
    m = m.inv();
    std::cout<<"M: "<<m<<std::endl;

    cv::Mat mul = m*pointMat;
    std::cout<<"Mul: "<<mul<<std::endl;
    
    cv::Vec3f res = cv::Vec3f(mul.at<double>(0), mul.at<double>(1), mul.at<double>(2));
    std::cout<<"Res: "<<res<<std::endl;

    return res;
}


double getAngleFromCorners3D(std::vector<cv::Point3f> corners3D, cv::Mat rvec, cv::Mat tvec){
    // Normal
        cv::Point3f center = obtainCenter3D(corners3D);
        cv::Vec3f markerCenter = cv::Vec3f(center.x, center.y, center.z);
        cv::Vec3f markerCenterPosed = applyPoseToVec(markerCenter, rvec, tvec);
        cv::Vec3f elevatedMarkerCenter = cv::Vec3f(markerCenter[0]*1.5, markerCenter[1]*1.5, markerCenter[2]*1.5);
        cv::Vec3f perpendicularPosed = applyPoseToVec(elevatedMarkerCenter, rvec, tvec);
        cv::Vec3f perpendicularPosedAndMoved = cv::Vec3f(perpendicularPosed[0]-markerCenterPosed[0],perpendicularPosed[1]-markerCenterPosed[1],perpendicularPosed[2]-markerCenterPosed[2]);
        // Normalise
        cv::Vec3f normalisedPerpendicular; cv::normalize(perpendicularPosedAndMoved, normalisedPerpendicular);
        //normalisedPerpendicular = worldToCamera(normalisedPerpendicular, rvec, tvec);
    // Vector of the camera
        cv::Vec3f cameraVector = cv::Vec3f(0.0, 0.0, 1.0);
    // Angle
        double angle = normalisedPerpendicular.dot(cameraVector);
        angle = angle/(cv::norm(normalisedPerpendicular)*cv::norm(cameraVector));
        angle = acos(angle);
        angle = (angle*180)/3.141516;
        angle -= 90;
    return angle;
}



#endif
