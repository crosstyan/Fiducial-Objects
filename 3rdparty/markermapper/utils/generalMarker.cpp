#ifndef _GeneralMarker_CPP_
#define _GeneralMarker_CPP_

#include <fstream>

#include "generalMarker.hpp"


// GMobtainCenter2D
cv::Point2f GMobtainCenter2D(std::vector<cv::Point2f> corners){
    double acc_x, acc_y;
    acc_x = 0.0; acc_y = 0.0;
    for(uint h = 0; h < corners.size(); h++){
        acc_x += corners[h].x;
        acc_y += corners[h].y;
    }
    cv::Point2f center2D = cv::Point2f(acc_x/corners.size(), acc_y/corners.size());
    return center2D;
}


// GMobtainCenter3D
cv::Point3f GMobtainCenter3D(std::vector<cv::Point3f> corners){
    double acc_x = 0;
    double acc_y = 0;
    double acc_z = 0;
    for(uint c = 0; c < corners.size(); c++){
        acc_x += corners[c].x;
        acc_y += corners[c].y;
        acc_z += corners[c].z;
    }
    cv::Point3f center3D = cv::Point3f(acc_x/corners.size(), acc_y/corners.size(), acc_z/corners.size());
    return center3D;
}

namespace general_marker {


GeneralMarker::GeneralMarker(){
    // Default values
    this->id = -1;
    this->_verbose = false;
    this->_verboseDebug = false;
    this->_angle = -1.0;
    this->_name = "";
    this->_hasContours = false;
    this->ssize = -1;
    this->_corners3D.clear();
    this->_area = 1;
    this->_config = -1;
}


GeneralMarker::GeneralMarker(int id, bool verbose, bool verboseDebug){
    this->id = id;
    // Verboses
    this->_verbose = verbose;
    this->_verboseDebug = verboseDebug;
    // Default values
    this->_angle = -1.0;
    this->_name = "";
    this->_hasContours = false;
    this->ssize = -1;
    this->_corners3D.clear();
    this->_area = 1;
    this->_config = -1;
}

GeneralMarker::GeneralMarker(int id, float size, bool verbose, bool verboseDebug){
    this->id = id;
    this->ssize = size;
    // Verboses
    this->_verbose = verbose;
    this->_verboseDebug = verboseDebug;
    // Default values
    this->_angle = -1.0;
    this->_name = "";
    this->_hasContours = false;
    this->_corners3D.clear();
    this->_area = 1;
    this->_config = -1;
}


GeneralMarker::GeneralMarker(int id, cv::Mat rt, float size, bool verbose, bool verboseDebug){
    this->id = id;
    //this->rt_g2m = rt.clone();
    rt.copyTo(this->rt_g2m);
    this->ssize = size;
    // Verboses
    this->_verbose = verbose;
    this->_verboseDebug = verboseDebug;
    // Default values
    this->_angle = -1.0;
    this->_name = "";
    this->_hasContours = false;
    this->_corners3D.clear();
    this->_area = 1;
    this->_config = -1;
}


GeneralMarker::GeneralMarker(const GeneralMarker& mdetection): std::vector<cv::Point2f>(mdetection){
    mdetection.copyTo(*this);
}


GeneralMarker::GeneralMarker(const std::vector<cv::Point2f>& corners, int newId): std::vector<cv::Point2f>(corners){
    id = newId;
    // Default values
    this->_verbose = false;
    this->_verboseDebug = false;
    this->_angle = -1.0;
    this->_name = "";
    this->_hasContours = false;
    this->ssize = -1;
    this->_corners3D.clear();
    this->_area = 1;
    this->_config = -1;
}


GeneralMarker::GeneralMarker(int id, std::vector<cv::Point2f> corners, std::vector<cv::Point3f> corners3D, double area, bool verbose, bool verboseDebug){
    this->_verbose = verbose;
    this->_verboseDebug = verboseDebug;
    this->id = id;
    this->_area = area;
    if(corners.size() != 0){
        for(uint i = 0; i < corners.size(); i++){
            (*this).push_back(corners[i]);
        }
        this->computeCenter();
    }
    if(corners3D.size() != 0){
        for(uint i = 0; i < corners3D.size(); i++){
            this->_corners3D.push_back(corners3D[i]);
        }
        this->computeCenter3D();
    }
    this->_angle = -1.0;
    this->_name = "";
    this->_hasContours = false;
    this->_config = -1;
}



void  GeneralMarker::copyTo(GeneralMarker &m)const{
    // Info
    m._verbose = _verbose;
    m._verboseDebug = _verboseDebug;
    m.id=id;
    m.ssize=ssize;
    m._angle = _angle;
    m._name = _name;
    m.rt_g2m = rt_g2m.clone();
    m._config = _config;
    // Corners
    m.resize(size());
    for(size_t i=0;i<size();i++)
        m.at(i)=at(i);
    m.computeCenter();
    // Corners 3D
    m._corners3D.clear();
    for(auto c: _corners3D){
        m._corners3D.push_back(c);
    }
    m.computeCenter3D();
    // Contours
    m._hasContours = _hasContours;
    m.contourPoints=contourPoints;
}



std::vector<cv::Point2f> GeneralMarker::getAsMinimun4Corners2D() const{
    std::vector<cv::Point2f> corners2D = this->getCorners();

    // Normal markers
    if(corners2D.size() >= 4){
        // Nothing to do
        // ...
    }


    // Triangle markers
    if(corners2D.size() == 3){
        // We compute the center and push it as a new "corner"
        cv::Point2f center;
        for(auto c: corners2D){
            center += c;
        }
        center = center/3;
        corners2D.push_back(center);
    }

    // Default case
    if(corners2D.size() == 0){
        // Nothing to do
        // ...
    }

    return corners2D;
}



std::vector<cv::Point3f> GeneralMarker::getAsMinimun4Corners3D() const{
    std::vector<cv::Point3f> corners3D = this->getCorners3D();

    // Normal markers
    if(corners3D.size() >= 4){
        // Nothing to do
        // ...
    }


    // Triangle markers
    if(corners3D.size() == 3){
        // We compute the center and push it as a new "corner"
        cv::Point3f center;
        for(auto c: corners3D){
            center += c;
        }
        center = center/3;
        corners3D.push_back(center);
    }

    // Default case
    if(corners3D.size() == 0){
        // Nothing to do
        // ...
    }

    return corners3D;
}




void GeneralMarker::setContours(std::vector<cv::Point> contours){
    this->_hasContours = true;
    this->contourPoints.clear();
    for(uint i = 0; i < contours.size(); i++){
        this->contourPoints.push_back(contours[i]);
    }
}



void GeneralMarker::setCorners2D(std::vector<cv::Point2f> corners2D){
    (*this).clear();
    for(auto c: corners2D){
        (*this).push_back(c);
    }
    this->computeCenter();
};


void GeneralMarker::setCorners3D(std::vector<cv::Point3f> corners3D){
    this->_corners3D.clear();// = corners3D;
    for(uint i = 0; i < corners3D.size(); i++){
        this->_corners3D.push_back(corners3D[i]);
    }
    this->computeCenter3D();
};



// Return 3D corners in different reference system
std::vector<cv::Point3f> GeneralMarker::get3dPoints(bool applyRt) const{
    std::vector<cv::Point3f> newCorners3D;
    for(auto c: this->_corners3D){
        newCorners3D.push_back(c);
    }

    if (!this->rt_g2m.empty() && applyRt){
        cv::Mat m = this->rt_g2m.clone();
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

    return newCorners3D;
}

// Return resized 3D corners
std::vector<cv::Point3f> GeneralMarker::get3dPoints(double size) const{

    // Rescaling
    std::vector<cv::Point3f> newCorners3D;
    float scale = cv::norm(this->_corners3D[0]-this->_corners3D[1])/size;
    for(auto c: this->_corners3D){
        newCorners3D.push_back(cv::Point3f(c.y, c.x, 0)/scale);
    }

    return newCorners3D;
}




void GeneralMarker::draw(cv::Mat& in,  cv::Scalar color, int lineWidth){
    this->drawMarker(in, color, color, lineWidth, lineWidth);
}


void GeneralMarker::drawMarker(cv::Mat &image, cv::Scalar innerColor, cv::Scalar outerColor, double innerWeight, double outerWeight, std::string message, bool verbose){
    // Borders
    for(uint i = 0; i < (*this).size(); i++){
        if (i != (*this).size()-1)
            cv::line(image, (*this)[i], (*this)[i+1], outerColor, outerWeight);
        else
            cv::line(image, (*this)[i], (*this)[0], outerColor, outerWeight);
        cv::line(image, (*this)[i], this->_center, innerColor, innerWeight);
    }
    // ID
    double step = 0; double stepJump = 15;
    std::string markerId = std::to_string(this->id);
    int baseline = 0;
    cv::Size textSize = cv::getTextSize(markerId, cv::FONT_HERSHEY_PLAIN, 2, 2, &baseline);
    cv::rectangle(image, cv::Rect(this->_center.x-5, this->_center.y-textSize.height-5, textSize.width+5, textSize.height+5), innerColor, -1);
    cv::putText(image, markerId, this->_center, cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255,255,255), 2, false);
    step += stepJump;
    // Extra information?
    if(verbose){
        // Name ?
        if(this->_name != ""){
            baseline = 0; 
            textSize = cv::getTextSize(this->_name, cv::FONT_HERSHEY_PLAIN, 1.2, 1, &baseline);
            cv::Point2f startPoint = cv::Point2f(this->_center.x, this->_center.y+step+2);
            cv::rectangle(image, cv::Rect(startPoint.x-5, startPoint.y-textSize.height-5, textSize.width+10, textSize.height+10), innerColor, -1);
            cv::putText(image, this->_name, startPoint, cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(255,255,255), 1, false);
            step += (stepJump*1.2);   
        }
        // Angle ?
        if(this->_angle != -1.0){
            baseline = 0; std::string angleStr = std::to_string(this->_angle).substr(0,5); angleStr = "- Angle: "+angleStr;
            textSize = cv::getTextSize(angleStr, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
            cv::Point2f startPoint = cv::Point2f(this->_center.x, this->_center.y+step);
            cv::rectangle(image, cv::Rect(startPoint.x-5, startPoint.y-textSize.height-5, textSize.width+10, textSize.height+10), innerColor, -1);
            cv::putText(image, angleStr, startPoint, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 1, false);
            step += stepJump;
        }
        // Message ?
        if(message != ""){
            baseline = 0; message = "- "+message;
            textSize = cv::getTextSize(message, cv::FONT_HERSHEY_PLAIN, 0.75, 1, &baseline);
            cv::Point2f startPoint = cv::Point2f(this->_center.x, this->_center.y+step);
            cv::rectangle(image, cv::Rect(startPoint.x-5, startPoint.y-textSize.height-5, textSize.width+10, textSize.height+10), innerColor, -1);
            cv::putText(image, message, startPoint, cv::FONT_HERSHEY_PLAIN, 0.75, cv::Scalar(255,255,255), 1, false);
            step += stepJump;
        }
    }
}

void GeneralMarker::drawMarker3D(cv::Mat &image, cv::Point2f elevatedCenter, cv::Scalar innerColor, cv::Scalar outerColor, double innerWeight, double outerWeight, std::string message, bool verbose){
    // Standard drawing
    this->drawMarker(image, innerColor, outerColor, innerWeight, outerWeight, message, verbose);
    // Borders
    for(uint i = 0; i < (*this).size(); i++){
        cv::line(image, (*this)[i], elevatedCenter, outerColor, outerWeight);
    }
    cv::line(image, elevatedCenter, this->_center, innerColor, innerWeight);
}

void GeneralMarker::drawMarkerNormal(cv::Mat & image, std::vector<cv::Point2f> axisPoints,  cv::Scalar innerColor, cv::Scalar outerColor, double innerWeight, double outerWeight, cv::Scalar normalColor, std::string message, bool verbose){
    // Standard drawing
    this->drawMarker(image, innerColor, outerColor, innerWeight, outerWeight, message, verbose);
    // Axis drawing
    for(uint i = 0; i < axisPoints.size(); i++){
        cv::circle(image, axisPoints[0], 1, normalColor, 3);
        cv::line(image, this->_center, axisPoints[i], normalColor, 1);
    }
}




GeneralMarker &  GeneralMarker::operator=(const GeneralMarker& m){
    m.copyTo(*this);
    return *this;
}


void GeneralMarker::computeCenter(){
    this->_center = GMobtainCenter2D(*this);
}


void GeneralMarker::computeCenter3D(){
    this->_center3D = GMobtainCenter3D(this->_corners3D);
}


}
#endif
