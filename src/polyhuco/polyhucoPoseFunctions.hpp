#ifndef _POLYHUCO_POSE_FUNCTIONS_HPP_
#define _POLYHUCO_POSE_FUNCTIONS_HPP_

#include "polyhuco.h"

cv::Vec3f applyPoseToVec(cv::Vec3f point, cv::Mat rvec, cv::Mat tvec);

cv::Vec3f worldToCamera(cv::Vec3f vecc, cv::Mat rvec, cv::Mat tvec);

double getAngleFromCorners3D(std::vector<cv::Point3f> corners3D, cv::Mat rvec, cv::Mat tvec);

#endif