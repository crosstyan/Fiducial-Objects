#ifndef _MarkerMapper_H
#define _MarkerMapper_H
#include <cmath>
#include <memory>
#include <exception>
#include <opencv2/core/core.hpp>
#include <aruco/cameraparameters.h>
#include <aruco/markermap.h>
#include "marker_mapper_exports.h"
#include "mapper_types.h"
#include <thread>

// Detectors
    //#include <aruco/markerdetector.h>
    //#include "utils/markerDetector.h"
    #include "utils/generalMarkerDetector.hpp"

namespace aruco_mm{


/**
 * @brief The MarkerMapper class is the base class for all mappers.
 * It implements the factory pattern using getAvailableMarkerMappers() and  create()
 *
 */
class  MARKERMAPPER_API  MarkerMapper {


public:
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        Factory implementation
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

    // Reads from file
    static std::shared_ptr<MarkerMapper> readFromFile ( std::string fname ) ;
    
    // Return the possible mappers.
    // Currently: global_graph
    static std::vector <std::string> getAvailableMarkerMappers( ) { return {"global_graph"};}

    //Returns the mapper with the name indicated. If not specified returns "global_graph"
    static std::shared_ptr< MarkerMapper> create(std::string name="");

    /**
     * @brief setParams  sets the important params
     * @param cam_p initial camera parameters
     * @param defaultMarkerSize in meters
     * @param originMarkerId  marker that is set as origin of the reference system. If not set, the first one found is used
     * @param optimizeCameraIntrinsics ...
     */
    void setParams (const aruco::CameraParameters & cam_p, float defaultMarkerSize, int originMarkerId=-1,bool optimizeCameraIntrinsics = true) ;
    
    // Process next frame
    bool process (const cv::Mat &image,int frame_idx ,bool forceConnectedComponent=false ) ;
    
    // Draws detected markers in last call to process
    virtual void drawDetectedMarkers (cv::Mat &imshow , int border_width=1)=0;
    
    // Finalizes the process.
    void optimize ( bool blocking=true   )  ;
    bool isOptimizationFinished() ;//only when call finalize in non-blocking mode
    void waitForOptimizationFinished() ;//waits until optimization is finished. Call when isOptimizationFinished()==true non-blocking mode

    //for factory pourposes
    virtual std::string getName()const {return "base";}
    /**
     * @brief saveToPcd
     * @param fpath
     * @param addViewLocations whether to add or not the camera locations
    */
    void saveToPcd(std::string fpath,bool addViewLocations=false);//saves the MarkerSet to pcd
    
    // Saves in binary format for later use
    void saveToFile ( std::string fname ) ;

    // Save the set of poses to a file with tum rgbd format
    void saveFrameSetPosesToFile(std::string filepath );

    // Returns the markermap that can be used with aruco library for tracking
    aruco::MarkerMap getMarkerMap()  ;




    /*~~~~~~~~~~~~~~~~
        Accessors
    ~~~~~~~~~~~~~~~~*/

    // Returns the aruco detector
    inline general_marker_detector::GeneralMarkerDetector & getGeneralMarkerDetector(){return *_mdetector;};
    inline void setGeneralMarkerDetector(general_marker_detector::GeneralMarkerDetector *detector){this->_mdetector = detector; };

    // Returns the frame set
    FrameSet &getFrameSet(){return _frameSet;}

    // Returns the marker sets
    MarkerSet &getMarkerSet(){return _markerSet;}

    // Returns the camera parameters
    aruco::CameraParameters &getCameraParams(){return _cam_params;}

    // Returns the marker that is set as origin of the reference system
    uint32_t getOriginMarkerId() const{return _originMarkerId;}

    // Returns the marker indicated.
    MarkerInfo & operator[](int id);

    // Returns the frame indicated.
    FrameInfo & operator()(int frame_idx);

    // Returns the internal variable _defaultMarkerSize
    float getDefaultMarkerSize()const {return _defaultMarkerSize;}



    //-------------------------------------------------------
    //           YOU SHOULD NOT USE BELOW THIS POINT
    //-------------------------------------------------------
    
    //Process the markers in detected_markers
    virtual bool process  (  const markerSet &detected_markers,int frame_idx =-1 , bool forceConnectedComponent=false) =0;

    void print_debug_info();


    // Markers - Configuration
    std::map<int, int> _markersConfiguration;
    inline void setConfigByMarkerId(int id, int config){
      this->_markersConfiguration[id] = config;
    };
    inline int getConfigFromMarkerId(int id){
        return this->_markersConfiguration[id];
    };


protected:

    /*~~~~~~~~~~~~~~~~~
        Data types
    ~~~~~~~~~~~~~~~~~*/
    MarkerSet _markerSet;
    FrameSet _frameSet;
    aruco::CameraParameters _cam_params;
    general_marker_detector::GeneralMarkerDetector* _mdetector;
    float _defaultMarkerSize;
    int _originMarkerId;//id  of the marker that is fixed setting the ref of global system
    bool _optimizeCameraIntrinsics;
    int _frameCounter=0;
    bool _finishedProcessing=true;




    /*~~~~~~~~~~~~~~~~
        Methods
    ~~~~~~~~~~~~~~~~*/

    void optimize_wrap ( )  {
        optimize_impl();
        _finishedProcessing=true;
    }

    virtual void optimize_impl()=0;
    virtual void toStream( std::ostream & fname ) {(void)fname;}
    virtual void fromStream( std::istream & fname )  {(void)fname;}


    //other methods
    double bundleAdjustment();

    double estimateCurrentLocation(std::vector<cv::Point3f> &objPoints,std::vector<cv::Point2f> &imgPoints,cv::Mat &R,cv::Mat &T,bool useRansac);

    void computeFrameSetPoses();

    void computeFrameSetPoses( FrameSet &fset_io, MarkerSet &mset,aruco::CameraParameters cam_params,bool onlyUnkonwn=true);

    markerSet deleteAllBut(  markerSet &in,const std::set<uint32_t> &tokeep,const std::set<uint32_t> &tokeep2={});

    double bundleAdjustment(MarkerSet & MarkerSet, FrameSet & FrameSet,  const std::set<uint32_t> &fixedMarkers,aruco::CameraParameters &camp, bool fix_cameraparams);



    //io
    void  base_fromStream ( std::istream &str )  ;
    void  base_toStream ( std::ostream &str )  ;



    std::vector<double> getRprjErr(const FrameInfo &fi,     MarkerSet &mi, const aruco::CameraParameters &cp);
    double getRprjErr(const std::vector<cv::Point3f> &p3d,const  std::vector<cv::Point2f> &p2d ,aruco::CameraParameters &cam, const cv::Mat &rt);


    void getQuaternionAndTranslationfromMatrix44(const cv::Mat &M_in ,float &qx,float &qy,float &qz,float &qw,float &tx,float &ty,float &tz);

    std::thread _optimizingThread;

};
};

#endif

