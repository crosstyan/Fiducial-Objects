#ifndef __FIDUCIAL_OBJECT_CPP__
#define __FIDUCIAL_OBJECT_CPP__

#include <fstream>
#include <string>
#include <vector>
#include <stdexcept>
#include <sstream>
#include <numeric>
#include <filesystem>
#include "fiducialObject.h"
#include "polyhuco/polyhuco.h"

namespace fiducialObject{

/*~~~~~~~~~~~
    Utils
~~~~~~~~~~~*/

// readFile
std::vector<std::string> readFile(const std::string& filePath) {
    std::vector<std::string> lines;
    std::ifstream file(filePath);

    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file");
    }

    std::string line;
    while (std::getline(file, line)) {
        lines.push_back(line);
    }

    file.close();
    return lines;
}

// splitsString
std::vector<std::string> splitString(const std::string& str, const std::string& delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(str);

    while (std::getline(tokenStream, token, delimiter[0])) {
        tokens.push_back(token);
    }

    return tokens;
}

// findStringPositions
std::vector<int> findStringPositions(const std::vector<std::string>& vec, const std::string& str) {
    std::vector<int> positions;
    for (int i = 0; i < vec.size(); ++i) {
        if (vec[i].find(str) != std::string::npos) {
            positions.push_back(i);
        }
    }
    return positions;
}

// addToFile
void addToFile(const std::vector<std::string>& lines, const std::string& filePath) {
    std::ofstream file;

    file.open(filePath, std::ios::app); // Open the file in append mode

    // Check if the file is open
    if (!file) {
        std::cerr << "Failed to open the file.";
        return;
    }

    // Write each line from the vector to the file
    for (const auto& line : lines) {
        file << line << "\n";
    }

    file.close(); // Close the file
}

// addMapToFile
void addMapToFile(const std::string& filename, const std::string& aruco_bc_dict, const int& aruco_bc_nmarkers, const int& aruco_bc_mInfoType, const std::unordered_map<int, std::vector<cv::Point3f>>& aruco_bc_markers) {
    std::string aux_file = "aux.yml";
    {
        cv::FileStorage fs(aux_file, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_YAML);

        fs << "aruco_bc_dict" << aruco_bc_dict;
        fs << "aruco_bc_nmarkers" << aruco_bc_nmarkers;
        fs << "aruco_bc_mInfoType" << aruco_bc_mInfoType;
        fs << "aruco_bc_markers" << "[";

        // Extract keys into a vector
        std::vector<int> keys;
        for (const auto& marker : aruco_bc_markers) {
            keys.push_back(marker.first);
        }

        // Sort the keys in ascending order
        std::sort(keys.begin(), keys.end());

        // Iterate through the sorted keys
        for (const auto& key : keys) {
            fs << "{:" << "id" << key;
            fs << "corners" << "[:";
            for (const auto& corner : aruco_bc_markers.at(key)) {
                fs << corner;
            }
            fs << "]";
            fs << "}";
        }

        fs << "]";
    }

    // Append auxiliary file content to the actual file
    std::ifstream src(aux_file, std::ios::binary);
    std::ofstream dest(filename, std::ios::app | std::ios::binary);
    dest << src.rdbuf();

    // Delete the auxiliary file
    if (remove(aux_file.c_str()) != 0) {
        std::cerr << "Error deleting file " << aux_file << std::endl;
    }
}

/*~~~~~~~~~~~~~~~~~~~~~~
    Functionalities
~~~~~~~~~~~~~~~~~~~~~~*/

// loadFromFile
void Object::loadFromFile(std::string path){
    // Read file
    std::vector<std::string> fileContent = readFile(path);
    // Read markers configuration content
    int numOfMarkersSchemas = std::stoi(splitString(fileContent[0],"_")[3]);
    for(int i = 0; i < numOfMarkersSchemas; i++){
        int iniPos = findStringPositions(fileContent, "_MARKER_"+std::to_string(i)+"_BEGIN_")[0]+1;
        int endPos = findStringPositions(fileContent, "_MARKER_"+std::to_string(i)+"_END_")[0];
        std::vector<std::string> markerContent;
        for(int j = iniPos; j < endPos; j++){
            markerContent.push_back(fileContent[j]);
        }
        markersConfig.push_back(markerContent);
    }
    // Read map
        // Points
        int initialLine = findStringPositions(fileContent, "__MAP__")[0]+1;
        int endLine = fileContent.size();
        std::vector<std::string> mapFileContent;
        for(int j = initialLine; j < endLine; j++){
            mapFileContent.push_back(fileContent[j]);
        }
        map_processor::MapProcessor mapProcessor; mapProcessor.scale=1; mapProcessor.processMap(mapFileContent); mapProcessor.computeCenter();
        for(auto id: mapProcessor.getIds()){
            std::vector<cv::Point3f> corners = mapProcessor.getMarkerInformation(id).corners;
            for(auto corner: corners)
                markers[id].push_back(corner);
        }
        // Dict
        _map_dict = mapProcessor._dict;
        // mInfoType
        _map_mInfoType = mapProcessor._mInfoType;
}

void Object::saveToFile(std::string path){
    // Remove file if already existed
    if (std::filesystem::exists(path)) {
        std::filesystem::remove(path);
    }
    // Save markers information
    int numOfMarkersSchemas = markersConfig.size();
    addToFile({"__MARKERS_"+std::to_string(numOfMarkersSchemas)+"__"}, path);
    for(int i = 0; i < markersConfig.size(); i++){
        addToFile({"_MARKER_"+std::to_string(i)+"_BEGIN_"}, path);
        addToFile(markersConfig[i], path);
        addToFile({"_MARKER_"+std::to_string(i)+"_END_"}, path);
    }
    // Save map information
    addToFile({"__MAP__"}, path);
    addMapToFile(path, _map_dict, markersConfig.size(), _map_mInfoType, markers);
}

void Object::scale(float scale){
    for(auto &[id,v]:markers)for(auto &p:v){p*=scale;}
}

bool Detection::isVadid()const
{
    return observations.size()!=0;
}

// draw
void Detection::draw(const cv::Mat &img, cv::Mat K, cv::Mat D){
    cv::Mat overlay;
    img.copyTo(overlay);
    
    // Define your color in BGR format
    cv::Scalar markerColor = cv::Scalar(255, 0, 128); // Blue and red mixed to give a purplish color

    // Draw markers on image
    for (const auto& pair : observations) {
        int key = pair.first;
        const std::vector<cv::Point2f>& corners = pair.second;

        // Print ID with customized font and color
        cv::Point2f midPoint = std::accumulate(corners.begin(), corners.end(), cv::Point2f(0, 0)) * (1.f / corners.size());
        
        // Draw a background box for the ID text
        int baseline;
        std::string id_text = std::to_string(key);
        cv::Size textSize = cv::getTextSize(id_text, cv::FONT_HERSHEY_SIMPLEX, 1, 2, &baseline);
        cv::Point2f boxTopLeft = midPoint - cv::Point2f(textSize.width/2, textSize.height/2);
        cv::Point2f boxBottomRight = midPoint + cv::Point2f(textSize.width/2, textSize.height/2 + baseline);
        cv::rectangle(overlay, boxTopLeft, boxBottomRight, markerColor, cv::FILLED);
        // cv::putText(overlay, id_text, midPoint + cv::Point2f(-textSize.width/2, textSize.height/2), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,255,255), 2, cv::LINE_AA);

        // Print corners and lines with semi-transparent purplish color
        for (size_t i = 0; i < corners.size(); i++) {
            cv::line(overlay, corners[i], corners[(i+1) % corners.size()], markerColor, 2, cv::LINE_AA);
            cv::circle(overlay, corners[i], 5, markerColor, -1);
        }
    }

    // Blend the overlay with the original image
    double alpha = 0.5; // Define your transparency level
    cv::addWeighted(overlay, alpha, img, 1 - alpha, 0, img);


    // Draw ids on image
    for (const auto& pair : observations) {
        int key = pair.first;
        std::string id_text = std::to_string(key);
        const std::vector<cv::Point2f>& corners = pair.second;
        cv::Point2f midPoint = std::accumulate(corners.begin(), corners.end(), cv::Point2f(0, 0)) * (1.f / corners.size());
        int baseline;
        cv::Size textSize = cv::getTextSize(id_text, cv::FONT_HERSHEY_SIMPLEX, 1, 2, &baseline);
        cv::putText(img, id_text, midPoint + cv::Point2f(-textSize.width/2, textSize.height/2), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,255,255), 2, cv::LINE_AA);
    }


    // Draw pose (if available)
    if(!tvec.empty() && !rvec.empty() && !K.empty() && !D.empty()){
        float axisLength = 0.01; // This value might need to be adjusted depending on your use case
        std::vector<cv::Point3f> axisPoints;
        axisPoints.push_back(cv::Point3f(0, 0, 0));
        axisPoints.push_back(cv::Point3f(axisLength, 0, 0));
        axisPoints.push_back(cv::Point3f(0, axisLength, 0));
        axisPoints.push_back(cv::Point3f(0, 0, axisLength));

        std::vector<cv::Point2f> imagePoints;
        cv::projectPoints(axisPoints, rvec, tvec, K, D, imagePoints);

        // Draw lines between the points with customized line style and color
        cv::line(img, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        cv::putText(img, "X", imagePoints[1], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255), 2, cv::LINE_AA);
        cv::line(img, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        cv::putText(img, "Y", imagePoints[2], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0), 2, cv::LINE_AA);
        cv::line(img, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
        cv::putText(img, "Z", imagePoints[3], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,0,0), 2, cv::LINE_AA);
    }
}






// detect
std::vector<Detection> Detector::detect(const std::vector<Object> &obj, const cv::Mat  &image,const cv::Mat CameraParams,const cv::Mat CameraDistortion){
    std::vector<Detection> detectionInfo;
    for(auto o: obj){
        // Creation
        Detection detInfo;
        detInfo.markers = o.markers;
        detInfo.markersConfig = o.markersConfig;
        // Create polyhuco
        polyhuco::Polyhuco poly("",1,"Cucomarker",1,1,"LOADED","LOADED");
        poly.loadFromContent(o.markersConfig, o.markers);
        poly.setThresAngle(10);
        poly.setThresNDet(0);
        poly.setThresArea(0);
        // Process frame
        polyhuco::foundInFrame foundInfo = poly.findInFrame(image, "-", 1, 1, true, CameraParams, CameraDistortion);
        for(auto m: foundInfo.detectionInfo.detectedMarkers){
            detInfo.observations[m.id] = m.getCorners();
        }
        for(auto m: foundInfo.guessedInfo.guessedMarkers){
            detInfo.observations[m.id] = m.getCorners();
        }
        if(foundInfo.detectionInfo.poseIsSelected){
            detInfo.tvec = foundInfo.detectionInfo.selectedPose.tvec.clone();
            detInfo.rvec = foundInfo.detectionInfo.selectedPose.rvec.clone();
        }
        // Push
        detectionInfo.push_back(detInfo);
    }
    return detectionInfo;
}

}


#endif
