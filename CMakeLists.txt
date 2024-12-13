# Minimun version for cmake
CMAKE_MINIMUM_REQUIRED(VERSION 3.5)

project(fiducial_object VERSION "1.0.0" LANGUAGES CXX C)
set(PROJECT_SOVERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS ON) #...with compiler extensions like gnu++11
set(CMAKE_CXX_STANDARD 20)

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#      DLL posfix and other OS related stuff
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
if(WIN32)
    # Postfix of DLLs:
    SET(PROJECT_DLLVERSION "${PROJECT_VERSION_MAJOR}${PROJECT_VERSION_MINOR}${PROJECT_VERSION_PATCH}")
    SET(RUNTIME_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin CACHE PATH "Directory for dlls and binaries")
    SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin CACHE PATH "Directory for binaries")
    SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin CACHE PATH "Directory for dlls")
else()
    # Postfix of so's:
    set(PROJECT_DLLVERSION)
endif()
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/libs)
link_directories(${CMAKE_BINARY_DIR}/libs)
OPTION(BUILD_SHARED_LIBS 	"Set to OFF to build static libraries" ON)



# Options
OPTION(BUILD_UTILS "Set to off to not build utils" ON)
IF(NOT WIN32)
OPTION(BUILD_UTILS_DEVELOPER "Set to off to not build developer utils" ON)
OPTION(BUILD_TESTS "Set to off to not build tests" OFF)
ENDIF()
OPTION(BUILD_MARKERMAPPER "Set to off to not build marker mapper" ON)
OPTION(BUILD_GUI  "Set to off to not build utils" ON)

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#      Dependencies
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# OPENCV
find_package(OpenCV REQUIRED)
MESSAGE( [Main] " OPENCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
include_directories( ${OpenCV_INCLUDE_DIRS} )
set(REQUIRED_LIBRARIES ${REQUIRED_LIBRARIES}  opencv_core opencv_imgproc opencv_calib3d)
# EIGEN
include_directories( "3rdparty/" )

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#      Library Compiling
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

include_directories( "3rdparty/eigen3" )
ADD_SUBDIRECTORY(3rdparty/aruco)
include_directories( "3rdparty" )
ADD_SUBDIRECTORY(3rdparty/markermapper)
ADD_SUBDIRECTORY(3rdparty/cucomarker)
add_subdirectory(src)

IF (BUILD_UTILS)
    add_subdirectory(utils)
ENDIF()

#IF (BUILD_TESTS)
#    add_subdirectory(tests)
#ENDIF()


IF (BUILD_UTILS_DEVELOPER)
    add_subdirectory(utils_developer)
ENDIF()

#################################
#  Find and Configure Qt5
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
SET(BUILD_QT ON)
# -DQt5_DIR=$(brew --prefix qt5)/lib/cmake/Qt5
# https://stackoverflow.com/questions/40081327/macos-sierra-cmake-failed-with-qt5
if (APPLE)
    set(CMAKE_PREFIX_PATH "/opt/homebrew/opt/qt@5/lib/cmake/Qt5")
endif()

find_package(Qt5 COMPONENTS Widgets Network)
include_directories(${Qt5Widgets_INCLUDE_DIRS} ${QtQml_INCLUDE_DIRS})
add_definitions(${Qt5Widgets_DEFINITIONS})

IF(BUILD_GUI)
    IF(Qt5_FOUND)
        ADD_SUBDIRECTORY(qt)
    ELSE()
        MESSAGE(WARNING "Qt5 not found. GUI will not be built")
    ENDIF()
ENDIF()
# ----------------------------------------------------------------------------
#   Uninstall target, for "make uninstall"
# ----------------------------------------------------------------------------
CONFIGURE_FILE( "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" IMMEDIATE @ONLY)
ADD_CUSTOM_TARGET(uninstall "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")
# ----------------------------------------------------------------------------
# create configuration file from .in file (If you use windows take care with paths)
# ----------------------------------------------------------------------------
CONFIGURE_FILE("${PROJECT_SOURCE_DIR}/config.cmake.in" "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake")
INSTALL(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" DESTINATION share/${PROJECT_NAME}  )
