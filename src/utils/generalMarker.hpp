#ifndef _GeneralMarker_H_
#define _GeneralMarker_H_

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "../fiducialobject_exports.h"

namespace general_marker {

/**
 * @brief Class that represents the related information of a single detected marker in a single moment
 * 
 */
class FIDUCIALOBJECT_API GeneralMarker : public std::vector<cv::Point2f> {
    public:

        /*~~~~~~~~~~~~~~~~~~~
            Constructors
        ~~~~~~~~~~~~~~~~~~~*/
        GeneralMarker();
        GeneralMarker(int id, bool verbose = false, bool verboseDebug = false);
        GeneralMarker(int id, float size, bool verbose = false, bool verboseDebug = false);
        GeneralMarker(int id, cv::Mat rt, float size, bool verbose = false, bool verboseDebug = false);
        GeneralMarker(const GeneralMarker& mdetection);
        GeneralMarker(const std::vector<cv::Point2f>& corners, int newId = -1);
        GeneralMarker(int id,  std::vector<cv::Point2f> corners, std::vector<cv::Point3f> corners3D = std::vector<cv::Point3f>(), double area = 0.0, bool verbose = false, bool verboseDebug = false);
        ~GeneralMarker(){}
        

        /*~~~~~~~~~~~~~~~~~~~~~~~~~
            Marker information
        ~~~~~~~~~~~~~~~~~~~~~~~~~*/
        int id;
        float ssize;
        inline void setName(std::string name){this->_name = name;};
        inline void setVerboses(bool verbose, bool verboseDebug){this->_verbose = verbose; this->_verboseDebug = verboseDebug; };
        inline int getId() const{ return this->id; };
        void copyTo(GeneralMarker &mdetection) const;
        // Config
            inline void setConfig(int config){this->_config = config; };
            inline int getConfig(){return this->_config; };

        
        /*~~~~~~~~~~~~~~~~~
            Properties
        ~~~~~~~~~~~~~~~~~*/
        // Centers
            // 2D
            inline cv::Point2f getCenter() const{ return this->_center; };
            inline void setCenter(cv::Point2f newCenter){ this->_center = newCenter; };
            // 3D
            inline cv::Point3f getCenter3D() const{ return this->_center3D; };
            inline void setCenter3D(cv::Point3f newCenter){ this->_center3D = newCenter; };
        // Area
            inline double getArea() const{ return this->_area; };
        // Angle
            inline void setAngle(double angle){ this->_angle = angle; };
            inline double getAngle() const{ return this->_angle; };



        /*~~~~~~~~~~~~~~~~~~~~~~~
            Pose estimation
        ~~~~~~~~~~~~~~~~~~~~~~~*/
        cv::Mat rt_g2m; // Mat to transform points from global ref sys to marker ref sys
        inline cv::Mat getMat(){return this->rt_g2m;};
        // Corners getting wrappers where a minimun of 4 corners are needed
            std::vector<cv::Point2f> getAsMinimun4Corners2D() const;
            std::vector<cv::Point3f> getAsMinimun4Corners3D() const;


        /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            Contours and corners
        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
        // Contours
            std::vector<cv::Point> contourPoints;
            void setContours(std::vector<cv::Point> contours); 
            inline std::vector<cv::Point> getContours() const{ return this->contourPoints; };
        // Corners
            // 2D
            inline std::vector<cv::Point2f> getCorners() const{ return (*this); };
            void setCorners2D(std::vector<cv::Point2f> corners2D);
            // 3D
            inline std::vector<cv::Point3f> getCorners3D() const{ return this->_corners3D; };
            void setCorners3D(std::vector<cv::Point3f> corners3D);
        // Handlers for getting special 3d corners format
            // Old versions
            // Return 3D corners in different reference system
            std::vector<cv::Point3f> get3dPoints(bool applyRt = true) const;
            // Return resized 3D corners
            std::vector<cv::Point3f> get3dPoints(double size) const;




        /*~~~~~~~~~~~~~~~~~~~~~~~~~
            Drawing functions
        ~~~~~~~~~~~~~~~~~~~~~~~~~*/
        void draw(cv::Mat& in, cv::Scalar color=cv::Scalar(0,0,255), int lineWidth = -1); // Simplest method to draw a marker
        void drawMarker(cv::Mat &image, cv::Scalar innerColor, cv::Scalar outerColor, double innerWeight, double outerWeight, std::string message = "", bool verbose = false);
        void drawMarker3D(cv::Mat &image, cv::Point2f elevatedCenter, cv::Scalar innerColor , cv::Scalar outerColor, double innerWeight, double outerWeight, std::string message = "", bool verbose = false);
        void drawMarkerNormal(cv::Mat & image, std::vector<cv::Point2f> axisPoints,  cv::Scalar innerColor, cv::Scalar outerColor, double innerWeight, double outerWeight, cv::Scalar normalColor, std::string message = "", bool verbose = false);
    
        
        /*~~~~~~~~~~~~~~~~
            Operators
        ~~~~~~~~~~~~~~~~*/
        // [ = ]
        GeneralMarker & operator=(const GeneralMarker& m);
        // [ << ]
        friend std::ostream& operator<<(std::ostream& str, const GeneralMarker& M){
            str << M.id << " = ";
            for (uint i = 0; i < M.size(); i++)
                str << "(" << M[i].x << "," << M[i].y << ") ";
            return str;
        }


    private:
        // Verboses
        bool _verbose;
        bool _verboseDebug;
        // Marker information
        std::string _name;
        double _area;
        double _angle;
        int _config = -1;
        // Contours and corners
        bool _hasContours;
        std::vector<cv::Point3f> _corners3D;
        // Centers
        cv::Point2f _center;
        cv::Point3f _center3D;
        cv::Point3f _elevatedCenter;
        void computeCenter();
        void computeCenter3D();
};


}

#endif
