#include "cucomarkerMarkerDetector.hpp"

namespace aruco_mm {


// Class [Aruco Marker Detector] signature
const double signature = 1664895083006;





/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                                Public
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/



CucomarkerMarkerDetector::CucomarkerMarkerDetector(double markerSize, bool verbose, bool verboseDebug):GeneralMarkerDetector(markerSize, verbose, verboseDebug){
    this->_markersParamsPath = "_";
    this->_colorDetectionMode = "_";
};


std::vector<general_marker::GeneralMarker> CucomarkerMarkerDetector::detect(cv::Mat frame){

    // Erase previous detections
    this->_detectedMarkersIds.clear();
    this->_detectedMarkers.clear();

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~
        Cucomarker detection
    ~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    // Detection of the markers
    std::vector<cucomarker::Marker> markers = this->_cucomarkerDetector.detect(frame);
    for(uint i = 0; i < markers.size(); i++){
        std::vector<cv::Point2f> corners;
        std::vector<cv::Point3f> corners3D; // = markers[i].get3DCorners(this->_markerSize);
        for(uint j = 0; j < markers[i].size(); j++){
            corners.push_back(markers[i][j]);
        }
        general_marker::GeneralMarker newDetectedMarker = general_marker::GeneralMarker(markers[i].id, corners, corners3D, markers[i].getArea());
        newDetectedMarker.setVerboses(this->_verbose, this->_verboseDebug);
        newDetectedMarker.setContours(markers[i].contours);

        // Correct size
        newDetectedMarker.ssize = this->_markerSize;

        this->_detectedMarkersIds.push_back(newDetectedMarker.getId());
        this->_detectedMarkers.push_back(general_marker::GeneralMarker(newDetectedMarker));
    }


    // Computing 3D coordinates for each marker
    for(uint i = 0; i < this->_detectedMarkers.size(); i++){

        // For Customized markers we use get3DCorners()
        this->_detectedMarkers[i].setCorners3D( this->getMarkers3DCornersSchema( -1, cv::Mat(), markers[i].getConfigurationSchema()) );

        // Save configuration information
        this->_detectedMarkers[i].setConfig(markers[i].getConfigurationSchema());

        // Corrected size



//       // Compute pose plus draw axes
//       auto fs = cv::FileStorage();
//       fs.open("../../DATA/4_Prism/Map/data/cameraCalibration.yml", cv::FileStorage::READ);
//       cv::Mat K, D;
//       fs["camera_matrix"] >> K;
//       fs["distortion_coefficients"] >> D;

//       // Compute pose plus draw axes
//       cv::Mat rvec, tvec;
//       bool good = cv::solvePnP( this->_detectedMarkers[i].getAsMinimun4Corners3D() , this->_detectedMarkers[i].getAsMinimun4Corners2D(), K, D, rvec, tvec);
//       std::cout<<"Pose estimation: "<<good<<std::endl;


//       // Draw detection
//       this->_detectedMarkers[i].draw(frame, cv::Scalar(255,0,0), 1);

//       // Draw axes
//       std::vector<cv::Point3f> proj3d = {cv::Point3f(0,0,0),cv::Point3f(0,0,1)};
//       std::vector<cv::Point2f> proj2d;
//       cv::projectPoints(proj3d, rvec, tvec, K, D, proj2d);
//       //std::cout<<">>>: "<<proj2d[0]<<"  |  "<<proj2d[1]<<std::endl;


//       cv::line(frame, proj2d[0], proj2d[1], cv::Scalar(0,0,255), 2);
//       cv::imshow("Drawing axe", frame);
//       cv::waitKey();



    }



    return _detectedMarkers;
}





void CucomarkerMarkerDetector::print(){
    std::cout<<BMAGENTA<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~["<<RESET<<" Cucomarker Marker Detector "<<BMAGENTA<<"]~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<RESET<<std::endl;
    std::cout<<MAGENTA<<"\t- verbose: "<<RESET<<this->_verbose<<std::endl;
    std::cout<<MAGENTA<<"\t- verboseDebug: "<<RESET<<this->_verboseDebug<<std::endl;
    std::cout<<MAGENTA<<"\t- markerSize: "<<RESET<<this->_markerSize<<std::endl;
    std::cout<<MAGENTA<<"\t- detectedMarkers: "<<RESET<<std::endl;
        for(auto m: this->_detectedMarkers){
            std::cout<<"\t\t> "<<m<<std::endl;
        }
    std::cout<<CYAN<<"\t- markersParamsPath: "<<RESET<<this->_markersParamsPath<<std::endl;
    std::cout<<CYAN<<"\t- colorDetectionMode: "<<RESET<<this->_colorDetectionMode<<std::endl;
    std::cout<<BMAGENTA<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<RESET<<std::endl;
}




/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  FILES  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Files:
        // Specific
            double signature

        // General
            bool _verbose;
            bool _verboseDebug;
            double _markerSize;
            std::vector<int> _detectedMarkersIds;
            std::vector<general_marker::GeneralMarker> _detectedMarkers;

        // Specific
            // Detector
            cucomarker::MarkerDetector _cucomarkerDetector;
                // Config
                std::string _markersParamsPath;
                std::string _colorDetectionMode;
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

// Save to file
void CucomarkerMarkerDetector::saveToFile(std::string filePath) const{
    cv::FileStorage fs(filePath, cv::FileStorage::WRITE);
    fs << "signature" << signature;
    // General
    fs << "verbose" << this->_verbose; 
    fs << "verboseDebug" << this->_verboseDebug;
    fs << "markerSize" << this->_markerSize;
    // Specific
    fs << "markersParamsPath" << cv::String(this->_markersParamsPath);
    fs << "colorDetectionMode" << cv::String(this->_colorDetectionMode);
    fs.release();
}

// Read from file
std::shared_ptr<CucomarkerMarkerDetector> CucomarkerMarkerDetector::readFromFile(std::string filePath){

    std::shared_ptr<CucomarkerMarkerDetector> result;

    cv::FileStorage fs(filePath, cv::FileStorage::READ);
    if((double)fs["signature"] == signature){

        // General
        bool verbose = false; if((int)fs["verbose"] == 1){ verbose = true; }
        bool verboseDebug = false; if((int)fs["verboseDebug"] == 1){ verboseDebug = true; }
        result = std::make_shared<aruco_mm::CucomarkerMarkerDetector>((double)fs["markerSize"], verbose, verboseDebug);

        // Specific
        if((std::string)fs["markersParamsPath"] != "_")
            result->setParams((std::string)fs["markersParamsPath"]);
        if((std::string)fs["colorDetectionMode"] != "_")
            result->setColorDetectionMode((std::string)fs["colorDetectionMode"]);

    }else{
        // Exception
        // ...
    }
    fs.release();

    return result;
}



void CucomarkerMarkerDetector::setParams(std::string markersParamsPath){
    this->_markersParamsPath = markersParamsPath;


    // Are there multiple configurations?
    std::string configurationsLine = markersParamsPath;
    std::string comma = ",";

    std::vector<std::string> configurations = splitConfigurationsString(markersParamsPath);

    for(auto config: configurations){
        this->_cucomarkerDetector.loadConfigFile(config);
    }


}

void CucomarkerMarkerDetector::setColorDetectionMode(std::string colorDetectionMode){
    this->_colorDetectionMode = colorDetectionMode;
    this->_cucomarkerDetector.setColorDetectionMode(colorDetectionMode);
}


std::vector<std::string> CucomarkerMarkerDetector::splitConfigurationsString(std::string markersParamsPath){

    std::string comma = ",";
    int pos = 0;
    std::vector<std::string> configurations;
    while(std::string::npos != (pos = markersParamsPath.find(comma))){
        std::string configuration = markersParamsPath.substr(0,pos);
        configurations.push_back(configuration);
//        std::cout<<"Configuration: "<<configuration<<std::endl;
        markersParamsPath.erase(0,pos+comma.length());
    }
    configurations.push_back(markersParamsPath);
//    std::cout<<"Configuration: "<<configurationsLine<<std::endl;

    return configurations;
}




/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                            Protected
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
std::vector<cv::Point3f> CucomarkerMarkerDetector::compute3DCornersSchema(int config){

    // Split configurations if needed
    std::vector<std::string> configurations = splitConfigurationsString(this->_markersParamsPath);

    if(config == -1){
        std::cout<<"Invalid config: "<<config<<std::endl;
        config = 0;
    }

    std::string configurationToUse = configurations[config];

    // Load 3D corners schema
    cv::FileStorage fs(configurationToUse, cv::FileStorage::READ);
//    cv::FileStorage fs(configurationToUse, cv::FileStorage::READ);
    cv::FileNode corners = fs["marker"]["corners"];
    cv::FileNodeIterator it = corners.begin(), it_end = corners.end(); // Go through the node

    std::vector<cv::Point3f> corners3D;
    for (; it != it_end; ++it){
        // Read each point
        cv::FileNode pt = *it;
        cv::Point3d point;
        cv::FileNodeIterator itPt = pt.begin();
        point.x = *itPt; ++itPt;
        point.y = *itPt;
        corners3D.push_back(point);

        //std::cout<<"Point: "<<point<<std::endl;
    }

    /*
        Set the point (0,0) in the center
    */
    // Move the points to the top left corner
        // Obtain quantity
        float min_x = 0;
        float min_y = 0;
        for(auto c: corners3D){
            if((min_x == 0) || (c.x < min_x)){
                min_x = c.x;
            }
            if((min_y == 0) || (c.y < min_y)){
                min_y = c.y;
            }
        }
        // Move
        std::vector<cv::Point3f> newCorners3D;
        for(auto c: corners3D){
            newCorners3D.push_back(cv::Point3f(c.x - min_x, c.y - min_y, 0));
        }
        corners3D = newCorners3D;
    // Obtain center point now
    float max_x = -1;
    float max_y = -1;
    for(auto c: corners3D){
        if(c.x > max_x){
            max_x = c.x;
        }
        if(c.y > max_y){
            max_y = c.y;
        }
    }
    // Rest quantities to move every point
    newCorners3D.clear();
    for(auto c: corners3D){
        newCorners3D.push_back(cv::Point3f(c.y - max_y/2., c.x - max_x/2., 0));
    }

    return newCorners3D;
}


std::vector<cv::Point3f> CucomarkerMarkerDetector::getMarkers3DCornersSchema(float size, cv::Mat RT, int config){


    std::vector<cv::Point3f> corners3D, newCorners3D;
    corners3D = compute3DCornersSchema(config);


    // Size to use
    float sizeToUse = size;
    if (size == -1){
        sizeToUse = this->_markerSize;
       
    }



    // Rest quantities to move every point
    newCorners3D.clear();
    float scale = cv::norm(corners3D[0]-corners3D[1])/sizeToUse;
    for(auto c: corners3D){
        newCorners3D.push_back(c/scale);
    }


    // APPLY RT?
    if (!RT.empty()){
        cv::Mat m = RT.clone();
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



    corners3D = newCorners3D;

//    for(auto c: corners3D){
//        std::cout<<c<<std::endl;
//    }

    return corners3D;
}

}
