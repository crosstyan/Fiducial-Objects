#~~~~~~~~~~~~~~~~~~~~~~~~
#  General information
#~~~~~~~~~~~~~~~~~~~~~~~~
cmake_minimum_required(VERSION 3.3)
project(cucomarker VERSION 2.3.0 LANGUAGES CXX C)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 17)





#~~~~~~~~~~~~~~~~~~~
#   Dependencies
#~~~~~~~~~~~~~~~~~~~
# OpenCV
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})





# -~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-
#           APPLICATIONS
# -~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-



#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#   2.  MARKER DETECTOR
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#[[ Creating the detector program only requires the file cucomarker.h and OpenCV. ]]
add_executable(markerdetector
    marker_detector/marker_detector.cpp
)
target_link_libraries(markerdetector
    opencv_highgui
    opencv_calib3d
)
# DUAL
#[[ Creating the detector program only requires the file cucomarker.h and OpenCV. ]]
add_executable(dual
    marker_detector/marker_detector_multi.cpp
)
target_link_libraries(dual
    opencv_highgui
    opencv_calib3d
)



#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#   1.  MARKER PRINTER
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#[[ Creating the program that reads the svg and creates the markers requires
    dependencies to read SVG files. However ]]
    include_directories(print_marker/3rdparty/tinyxml2)
add_subdirectory(print_marker/3rdparty/plutovg)
INCLUDE_DIRECTORIES(
    ${CMAKE_SOURCE_DIR}/print_marker/3rdparty/plutovg
    ${CMAKE_SOURCE_DIR}/print_marker/3rdparty/tinyxml2
)
add_executable(printmarker
    print_marker/print_marker.cpp
    print_marker/svgreader.h
    print_marker/svgreader.cpp
    print_marker/parser.h
    print_marker/3rdparty/tinyxml2/tinyxml2.cpp
)
target_link_libraries(printmarker
    opencv_highgui
    opencv_calib3d
    plutovg
)
