#ifndef _UTILS_HPP_
#define _UTILS_HPP_

/*~~~~~~~~~~~~~~~
    Includes
~~~~~~~~~~~~~~~*/  
// External includes
#include <vector>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/highgui.hpp"
// Internal includes
#include "colors.hpp"



/*~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Geometry operations
~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
// obtainCenter2D
cv::Point2f obtainCenter2D(std::vector<cv::Point2f> corners);
// obtainCenter3D
cv::Point3f obtainCenter3D(std::vector<cv::Point3f> corners);
// applyRvecToPoint
cv::Point3f applyRvecToPoint(cv::Mat rvec, cv::Point3f point);
// movePointToCamera
cv::Point3f movePointToCamera(cv::Mat rvec, cv::Mat tvec, cv::Point3f point);


// poseLine
std::string getPoseLine(int id, cv::Mat rvec, cv::Mat tvec, bool verboseDebug);



double computePercentageDifference(double a, double b);

void fiducialObjectConfigGeneration(std::vector<std::string> fo_markers_files,std::string fo_map_path,std::string fo_output_path);
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Polyhuco config file
~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
struct polyhucoConfig{
    std::string name;
    double markerSize;
    std::string markerConfig;
    std::string markerMap;
    std::string detectionMode;
    int minId;
    int maxId;
    std::string commentaries;
};
polyhucoConfig readPolyhucoConfig(std::string configFileFolder);
void printPolyhucoConfig(polyhucoConfig config);




#endif
