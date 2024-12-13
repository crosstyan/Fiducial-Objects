#include "arucoMarkerDetector.hpp"
#include "../colors.hpp"

namespace aruco_marker_detector {


// Class [Aruco Marker Detector] signature
const double signature = 1664877145730;




/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                                Public
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/



ArucoMarkerDetector::ArucoMarkerDetector(double markerSize, bool verbose, bool verboseDebug):GeneralMarkerDetector(markerSize, verbose, verboseDebug){
    this->_markersParamsPath = "_";
    this->_markersDictPath = "_";
};



std::vector<general_marker::GeneralMarker> ArucoMarkerDetector::detect(cv::Mat frame){

    // Erase previous detections
    this->_detectedMarkersIds.clear();
    this->_detectedMarkers.clear();

    /*~~~~~~~~~~~~~~~~~~~~~
        Aruco detection
    ~~~~~~~~~~~~~~~~~~~~~*/
    // Detection of the markers
    std::vector<aruco::Marker> markers = this->_arucoDetector.detect(frame);
        //std::vector<aruconano::Marker> markers = arucoDetector.detect(frame);
    for(uint i = 0; i < markers.size(); i++){
        std::vector<cv::Point2f> corners;
        for(uint j = 0; j < markers[i].size(); j++){
            corners.push_back(markers[i][j]);
        }
        general_marker::GeneralMarker newDetectedMarker = general_marker::GeneralMarker(markers[i].id, corners);
        newDetectedMarker.setVerboses(this->_verbose, this->_verboseDebug);
        this->_detectedMarkersIds.push_back(newDetectedMarker.getId());
        this->_detectedMarkers.push_back(general_marker::GeneralMarker(newDetectedMarker));
    }


    // Computing 3D coordinates for each marker
    for(uint i = 0; i < this->_detectedMarkers.size(); i++){
        // For Aruco markers we use get3DCorners()
        this->_detectedMarkers[i].setCorners3D( this->getMarkers3DCornersSchema(this->_markerSize) );
    }


    // Return detected markers
    return _detectedMarkers;
}




void ArucoMarkerDetector::print(){
    std::cout<<BMAGENTA<<"~~~~~~~~~~~~~~~~~~~~~~~~["<<RESET<<" Aruco Marker Detector "<<BMAGENTA<<"]~~~~~~~~~~~~~~~~~~~~~~~~"<<RESET<<std::endl;
    std::cout<<MAGENTA<<"\t- verbose: "<<RESET<<this->_verbose<<std::endl;
    std::cout<<MAGENTA<<"\t- verboseDebug: "<<RESET<<this->_verboseDebug<<std::endl;
    std::cout<<MAGENTA<<"\t- markerSize: "<<RESET<<this->_markerSize<<std::endl;
    std::cout<<MAGENTA<<"\t- detectedMarkers: "<<RESET<<std::endl;
        for(auto m: this->_detectedMarkers){
            std::cout<<"\t\t> "<<m<<std::endl;
        }
    std::cout<<CYAN<<"\t- markersParamsPath: "<<RESET<<this->_markersParamsPath<<std::endl;
    std::cout<<CYAN<<"\t- markersDictPath: "<<RESET<<this->_markersDictPath<<std::endl;
    std::cout<<BMAGENTA<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<RESET<<std::endl;
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
            aruco::MarkerDetector _arucoDetector;
                // Config
                std::string _markersParamsPath;
                std::string _markersDictPath;
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

// Save to file
void ArucoMarkerDetector::saveToFile(std::string filePath) const{
    cv::FileStorage fs(filePath, cv::FileStorage::WRITE);
    fs << "signature" << signature;
    // General
    fs << "verbose" << this->_verbose; 
    fs << "verboseDebug" << this->_verboseDebug;
    fs << "markerSize" << this->_markerSize;
    // Specific
    fs << "markersParamsPath" << cv::String(this->_markersParamsPath);
    fs << "markersDictPath" << cv::String(this->_markersDictPath);
    fs.release();
}

// Read from file
std::shared_ptr<ArucoMarkerDetector> ArucoMarkerDetector::readFromFile(std::string filePath){

    std::shared_ptr<ArucoMarkerDetector> result;

    cv::FileStorage fs(filePath, cv::FileStorage::READ);
    if((double)fs["signature"] == signature){

        // General
        bool verbose = false; if((int)fs["verbose"] == 1){ verbose = true; }
        bool verboseDebug = false; if((int)fs["verboseDebug"] == 1){ verboseDebug = true; }
        result = std::make_shared<aruco_marker_detector::ArucoMarkerDetector>((double)fs["markerSize"], verbose, verboseDebug);

        // Specific
        if((std::string)fs["markersParamsPath"] != "_")
            result->setParams((std::string)fs["markersParamsPath"]);
        if((std::string)fs["markersDictPath"] != "_")
            result->setDict((std::string)fs["markersDictPath"]);

    }else{
        // Exception
        // ...
    }
    fs.release();

    return result;
}




void ArucoMarkerDetector::setParams(std::string markersParamsPath){
    this->_markersParamsPath = markersParamsPath;
    this->_arucoDetector.loadParamsFromFile(markersParamsPath);
}

void ArucoMarkerDetector::setDict(std::string markersDictPath){
    this->_markersDictPath = markersDictPath;
    this->_arucoDetector.setDictionary(markersDictPath);
}





/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                            Protected
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

std::vector<cv::Point3f> ArucoMarkerDetector::getMarkers3DCornersSchema(float size, cv::Mat RT, int config)
{

    float sizeToUse = size;
    if (size == -1){
        sizeToUse = this->_markerSize;
    }


    float size_2 = sizeToUse/2.;
    std::vector<cv::Point3f> points = { cv::Point3f ( -size_2, size_2,0 ),
                                        cv::Point3f ( size_2, size_2 ,0 ),
                                        cv::Point3f ( size_2, -size_2,0 ),
                                        cv::Point3f ( -size_2, -size_2,0 )  };


    // APPLY RT?
    if (!RT.empty()){
        cv::Mat m = RT.clone();
        //aruco_mm::mult<cv::Point3f>(rt_g2m,newCorners3D);
        if ( m.type() ==CV_32F ) {
            const float *ptr=m.ptr<float> ( 0 );
            cv::Point3f res;
            for ( auto & p :  points ) {
                res.x= ptr[0]*p.x +ptr[1]*p.y +ptr[2]*p.z+ptr[3];
                res.y= ptr[4]*p.x +ptr[5]*p.y +ptr[6]*p.z+ptr[7];
                res.z= ptr[8]*p.x +ptr[9]*p.y +ptr[10]*p.z+ptr[11];
                p=res;
            }
        } else 	if ( m.type() ==CV_64F ) {
            const double *ptr=m.ptr<double> ( 0 );
            cv::Point3f res;
            for ( auto & p :  points ) {
                res.x= ptr[0]*p.x +ptr[1]*p.y +ptr[2]*p.z+ptr[3];
                res.y= ptr[4]*p.x +ptr[5]*p.y +ptr[6]*p.z+ptr[7];
                res.z= ptr[8]*p.x +ptr[9]*p.y +ptr[10]*p.z+ptr[11];
                p=res;
            }
        }
    }


    return points;
}







}
