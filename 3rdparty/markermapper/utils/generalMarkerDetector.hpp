#ifndef _GeneralMarkerDetector_H_
#define _GeneralMarkerDetector_H_

#include<cstdlib>

#include "generalMarker.hpp"
#include "colors.hpp"

namespace general_marker_detector {


/** @brief
    * --- ! --- 
    *     Clase interfaz para ser implementada por completo en un detector particular.
    *    Permite el manejo de diferentes tipos de marcadores de forma comoda.
    * --- ! ---
*/
class GeneralMarkerDetector {
    public:
        inline GeneralMarkerDetector(){};

        /**
         * @brief Construct a new General Marker Detector object
         * 
         * @param markerSize ...
         * @param verbose ...
         * @param verboseDebug ...
         */
        GeneralMarkerDetector(double markerSize, bool verbose = false, bool verboseDebug = false);


        // To IMPLEMENT (Mandatory)
            // Print
            virtual inline void print(){};
            /* ~~~~ Detection ~~~~
            Detection consists in getting the corners 2D of the markers in the image.
                + Computing the 3D corners of each marker independently
            ~~~~~~~~~~~~~~~~~~~ */
            virtual std::vector<general_marker::GeneralMarker> detect(cv::Mat frame) = 0;
            /*  ~~~~ 3D Corners ~~~~
            3D Corners configuration are needed to be accesible as well as modified for
            every detector. Each type of marker will use one different schema.
            ~~~~~~~~~~~~~~~~~~~~ */
            virtual std::vector<cv::Point3f> getMarkers3DCornersSchema(float size = -1, cv::Mat RT = cv::Mat(), int config = -1) = 0;
            // Save to file
            virtual void saveToFile(std::string filePath) const = 0;



        // To MODIFY (Optional)
            // Name for the marker labeler dict
            virtual inline std::string getMarkerLabelerDict(){return ""; };
            /* ~~~~ Estimated poses threshold ~~~~
                Threshold to prove if there is significant differences between both estimated poses
                of the markers asociated to the detector
            ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
            virtual inline float getEstimatedPosesDifferenceThreshold(){ return 0.6; };


        // Already IMPLEMENTED
            inline std::vector<int> getDetectedIds(){ return this->_detectedMarkersIds; };
            inline void setDetectedMarkers(std::vector<general_marker::GeneralMarker> newMarkers){ this->_detectedMarkers = newMarkers; };
            inline std::vector<general_marker::GeneralMarker> getDetectedMarkers(){ return this->_detectedMarkers; };
            general_marker::GeneralMarker getGeneralMarkerById(int id);
            // Minimun of 4 corners 3D in schema getting
            std::vector<cv::Point3f> getMinimum4Corners3DSchema(float size = -1, cv::Mat RT = cv::Mat(), int config = -1);
            // Reading object from file
            std::shared_ptr<GeneralMarkerDetector> readFromFile(std::string filePath);
            // Markers - config
            inline int getMarkerConfigById(int id){
                for(auto marker: this->_detectedMarkers){
                    if(marker.getId() == id){
                        return marker.getConfig();
                    }
                }
                return -1;
            };
    

    protected:
        bool _verbose;
        bool _verboseDebug;
        double _markerSize;
        std::vector<int> _detectedMarkersIds;
        std::vector<general_marker::GeneralMarker> _detectedMarkers;
};


}

#endif
