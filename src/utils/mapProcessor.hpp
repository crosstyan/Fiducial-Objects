#ifndef _MAP_PROCESSOR_
#define _MAP_PROCESSOR_

#include <vector>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include "opencv2/highgui.hpp"

namespace map_processor {

/**
 * @brief Auxiliar structure in order to facilitate the use of the infomration
 * related to the mapped markers
 * 
 */
struct MappedMarkerInformation{
    int id;
    std::vector<cv::Point3f> corners;
};

/*~~~~~~~~~~~~~~~~~~
    MAP PROCESSOR
~~~~~~~~~~~~~~~~~~*/
/**
 * @brief Map processor class perform the reading of the mapped 3d object in order to
 * make the pose estimation of the entire object.
 * 
 */
class MapProcessor {
    public:
        // Attributes
        cv::String mapPath;
        std::unordered_map<int,std::vector<cv::Point3f>> _mapContent;
        bool verbose;
        bool verboseDebug;
        double scale;
        // Methods
        inline MapProcessor(){};
        MapProcessor(cv::String mapPath, bool verbose = false, bool verboseDebug = false, double scaleForMap = 1);
        MapProcessor(std::unordered_map<int,std::vector<cv::Point3f>> &mapContent, bool verbose = false, bool verboseDebug = false, double scaleForMap = 1);
        void printInfo(); // Print mapped markers information
        MappedMarkerInformation getMarkerInformation(int id, bool centered=true); // Get the mapped information about a certain marker
        void computeCenter();  // Compute the center of the points and move them to mappedCenteredMarkers
        inline std::vector<int> getIds(){ return this->_ids; };
        void processMap(std::vector<std::string> mapContent = {}); // Process map file
        // Attributes
        cv::String _dict;
        int _nmarkers;
        int _mInfoType;
    private:
        // Attributes
        std::vector<int> _ids;
        std::vector<MappedMarkerInformation> mappedMarkers;
        cv::Point3f _center;
        std::vector<MappedMarkerInformation> mappedCenteredMarkers;
        // Methods
        void printMarkersInfo(bool centered = false);
        void applyScale();
};



}

#endif
