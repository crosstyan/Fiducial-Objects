#ifndef ArucoCalibrate_H
#define ArucoCalibrate_H
#include <string>
#include <aruco/aruco.h>
aruco::CameraParameters arucoCalibrate(std::string path2FolderWithImages,std::function<void(std::string msg)> onMessage={} );

#endif
