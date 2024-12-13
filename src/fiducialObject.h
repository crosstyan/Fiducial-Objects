/**
 *
 *
 *
 *\code
 *
 *\code
 */
#ifndef __FIDUCIAL_OBJECT_H_
#define __FIDUCIAL_OBJECT_H_

#include <vector>
#include <opencv2/core/core.hpp>
#include <unordered_map>
#include <opencv2/highgui.hpp>
#include "fiducialobject_exports.h"

namespace fiducialObject{

//A fiducial object
class FIDUCIALOBJECT_API Object{
    friend class Detector;
    std::vector<std::vector<std::string>> markersConfig;
    std::string _map_dict;
    int _map_mInfoType;
public:
    std::unordered_map<int,std::vector<cv::Point3f>> markers; //map with <markerId,3d points of the marker>
    void loadFromFile(std::string path);
    void saveToFile(std::string path);
    void scale(float scale=1);
};

// A detection of an object in a image
struct FIDUCIALOBJECT_API Detection: public Object{
    std::unordered_map<int,std::vector<cv::Point2f>> observations; //observation of the markers in the image. map with <key,2d points>
    cv::Mat rvec,tvec;//matrix rotation and translation vectors
    //draws the detection the image passed
    //If the camera parmeters are indicated (K,D) it also draws a 3D axis showing the orientation
    void draw(const cv::Mat &img, cv::Mat K = {}, cv::Mat D = {});
    bool isVadid()const;
 };



// Detects Objects in images and returns them
class FIDUCIALOBJECT_API Detector {
    public:
    //Looks for the objets passed in the image. If the camera parameters (K,D) are passed, then the detections are provided with
    //the estiamted pose w.r.t. the camera (rvec and tvec)
    static std::vector<Detection> detect(const std::vector<Object> &obj, const cv::Mat  &image,const cv::Mat K={},const cv::Mat D={});
};

}


#endif
