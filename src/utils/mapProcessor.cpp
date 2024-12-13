#include "mapProcessor.hpp"
#include "colors.hpp"

#include <vector>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include "opencv2/highgui.hpp"

namespace map_processor {

// Constructor
MapProcessor::MapProcessor(cv::String mapPath, bool verbose, bool verboseDebug, double scaleForMap){
    this->mapPath = mapPath;
    this->verbose = verbose;
    this->verboseDebug = verboseDebug;
    this->scale = scaleForMap;
    this->processMap();
}

// Constructor
MapProcessor::MapProcessor(std::unordered_map<int,std::vector<cv::Point3f>> &mapContent, bool verbose, bool verboseDebug, double scaleForMap){
    this->mapPath = "LOADED";
    this->_mapContent = mapContent;
    this->verbose = verbose;
    this->verboseDebug = verboseDebug;
    this->scale = scaleForMap;
    this->processMap();
}

void MapProcessor::printInfo(){
    std::cout<<std::endl;
    std::cout<<BGREEN<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~ Map information: ~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<RESET<<std::endl;
    std::cout<<CYAN<<"- Verbose: "<<RESET<<this->verbose<<std::endl;
    std::cout<<CYAN<<"- Verbose debug: "<<RESET<<this->verboseDebug<<std::endl;
    std::cout<<GREEN<<"- Map path: "<<RESET<<this->mapPath<<std::endl;
    std::cout<<GREEN<<"- Dict: "<<RESET<<this->_dict<<std::endl;
    std::cout<<GREEN<<"- Number of mapped markers (nmakers): "<<RESET<<this->_nmarkers<<std::endl;
    std::cout<<GREEN<<"- Markers info type: "<<RESET<<this->_mInfoType<<std::endl;
    std::cout<<GREEN<<"- Number of mapped markers (processed): "<<RESET<<this->mappedMarkers.size()<<std::endl;
    this->printMarkersInfo();
    std::cout<<GREEN"- Computed center: "<<RESET<<this->_center<<std::endl;
    this->printMarkersInfo(true);
    std::cout<<BGREEN<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<RESET<<std::endl<<std::endl;
}



MappedMarkerInformation MapProcessor::getMarkerInformation(int id, bool centered){
    // Centered?
    std::vector<MappedMarkerInformation> markers;
    if(centered){
        markers = this->mappedCenteredMarkers;
    }
    else{
        markers = this->mappedMarkers;
    }
    // Marker
    for(int i = 0; i < this->_ids.size(); i++){
        if(this->_ids[i] == id){
            return markers[i];
        }
    }
    // No exists in figure
    MappedMarkerInformation defaultMarker;
    defaultMarker.id = -1;
    return defaultMarker;
}


void MapProcessor::computeCenter(){
    /*~~~~~~~~~~~~~~~~~~~~~~
        Compute center
    ~~~~~~~~~~~~~~~~~~~~~~*/
    double accumulate_x, accumulate_y, accumulate_z;
    accumulate_x = 0.0;
    accumulate_y = 0.0;
    accumulate_z = 0.0;
    double numberOfPoints = 0.0;
    for(int i = 0; i < this->mappedMarkers.size(); i++){
        std::vector<cv::Point3f> corners = this->mappedMarkers[i].corners;
        for(int j = 0; j < corners.size(); j++){
            numberOfPoints += 1.0;
            accumulate_x += corners[j].x;
            accumulate_y += corners[j].y;
            accumulate_z += corners[j].z;
        }
    }
    // Compute mean
    this->_center = cv::Point3f(accumulate_x/numberOfPoints, accumulate_y/numberOfPoints, accumulate_z/numberOfPoints);
    /*~~~~~~~~~~~~~~~~~~~~~~~~~
        Move mapped corners
    ~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    for(int i = 0; i < this->mappedMarkers.size(); i++){
        MappedMarkerInformation newMarker;
        newMarker.id = this->mappedMarkers[i].id;
        for(int j = 0; j < this->mappedMarkers[i].corners.size(); j++){
            cv::Point3f oldCorner = this->mappedMarkers[i].corners[j];
            cv::Point3f newCorner;
            newCorner = cv::Point3f((oldCorner.x - this->_center.x), (oldCorner.y - this->_center.y), (oldCorner.z - this->_center.z));
            newMarker.corners.push_back(newCorner);
        }
        this->mappedCenteredMarkers.push_back(newMarker);
    }
}



// Process map file
void MapProcessor::processMap(std::vector<std::string> mapContent){

    if(this->mapPath != "LOADED"){


        auto mappedPolyhedronFile = cv::FileStorage();
        
        if(mapContent.size() == 0){
            mappedPolyhedronFile.open(this->mapPath, cv::FileStorage::READ);
        }
        else{
            std::string yml_content;
            for (const auto& line : mapContent) {
                yml_content += line + "\n";
            }
            // Luego, crea un FileStorage a partir de este string.
            mappedPolyhedronFile.open(yml_content, cv::FileStorage::READ | cv::FileStorage::MEMORY);
        }


        if(this->verboseDebug){
            std::cout<<YELLOW<<"> Read map: "<<RESET<<std::endl;
        }
        mappedPolyhedronFile["aruco_bc_dict"] >> this->_dict;
        mappedPolyhedronFile["aruco_bc_nmarkers"] >> this->_nmarkers;
        mappedPolyhedronFile["aruco_bc_mInfoType"] >> this->_mInfoType;
        cv::FileNode markers = mappedPolyhedronFile["aruco_bc_markers"];
        cv::FileNodeIterator it = markers.begin(), it_end = markers.end();
        int idx = 0;
        for(; it != it_end; ++it, idx++){
            MappedMarkerInformation mappedMarker;

            // Id
            int _id; (*it)["id"] >> _id;
            if(this->verboseDebug){
                std::cout<<YELLOW<<"\t- Id = "<<_id<<RESET<<std::endl;
            }
            mappedMarker.id = _id;
            this->_ids.push_back(_id);

            // FileNode
            cv::FileNode item = *it;
            //std::cout<<"Type of the node: "<<item["corners"].size()<<std::endl;
            
            // Corners
            std::vector<cv::Point3f> mappedMarkerCorners;
            cv::FileNodeIterator cornersIterator = item["corners"].begin(), cornersIterator_end = item["corners"].end();
            int idx2 = 0;
            if(this->verboseDebug){
                std::cout<<YELLOW<<"\t- Corners: "<<RESET<<std::endl;
            }
            for(; cornersIterator != cornersIterator_end; ++cornersIterator, idx2++){
                if(this->verboseDebug){
                    std::cout<<YELLOW<<"\t\t> Punto "<<idx2<<" -> "<<RESET;
                }
                double x = (double)(*cornersIterator)[0];
                double y = (double)(*cornersIterator)[1];
                double z = (double)(*cornersIterator)[2];
                cv::Point3f point = cv::Point3f(x, y, z);
                mappedMarkerCorners.push_back(point);
                if(this->verboseDebug){
                    std::cout<<YELLOW<<point<<RESET<<std::endl;
                }
            }
            mappedMarker.corners = mappedMarkerCorners;
            this->mappedMarkers.push_back(mappedMarker);
        }
        mappedPolyhedronFile.release();
    }
    else{
        // Use the already loaded items mapContent
        for (const auto& pair : this->_mapContent) {
            int key = pair.first;
            const std::vector<cv::Point3f>& values = pair.second;
            MappedMarkerInformation mappedMarker;
            mappedMarker.id = key;
            this->_ids.push_back(key);
            for (const cv::Point3f& point : values) {
                mappedMarker.corners.push_back(point);
            }
            this->mappedMarkers.push_back(mappedMarker);
        }
    }

    // Scale the map information?
    if(this->scale != 1){
        applyScale();
    }
}

void MapProcessor::printMarkersInfo(bool centered){
    // Centered?
    std::vector<MappedMarkerInformation> markers;
    cv::String markersName;
    if(centered){
        markers = this->mappedCenteredMarkers;
        markersName = "- Centered markers: ";
    }
    else{
        markers = this->mappedMarkers;
        markersName = "- Markers: ";
    }
    // Print information
    std::cout<<BMAGENTA<<markersName<<RESET<<std::endl;
    for(int i = 0; i < markers.size(); i++){
        std::cout<<BMAGENTA<<"\t- Id: "<<RESET<<markers[i].id<<std::endl;
        for(int j = 0; j < markers[i].corners.size(); j++){
            std::cout<<MAGENTA<<"\t\t- Corner "<<j<<": "<<RESET<<markers[i].corners[j]<<std::endl;
        }
    }
}



void MapProcessor::applyScale(){

    // Apply scale?
    if(this->scale == 1){
        return;
    }


    // The scale is applied on the mapped markers corners
    std::vector<map_processor::MappedMarkerInformation> scaledMappedMarkers;
    for(auto marker: this->mappedMarkers){

        map_processor::MappedMarkerInformation scaledMarker;
        scaledMarker.id = marker.id;

        for(auto corner: marker.corners){
            cv::Point3f scaledCorner = corner*this->scale;
            scaledMarker.corners.push_back(scaledCorner);
        }

        scaledMappedMarkers.push_back(scaledMarker);
    }
        // Push again
        this->mappedMarkers.clear();
        for(auto marker: scaledMappedMarkers){
            this->mappedMarkers.push_back(marker);
        }

}



}
