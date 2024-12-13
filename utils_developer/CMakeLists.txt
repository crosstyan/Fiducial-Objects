include_directories(${PROJECT_SOURCE_DIR}/src )

link_libraries(${REQUIRED_LIBRARIES} opencv_highgui )

# processVideo
add_executable(processPose processPose/processPose.cpp utils/utils.cpp)
target_link_libraries(processPose fiducial_object)

# Markers detection test
add_executable(detectionTest detectionTest/detectionTest.cpp utils/utils.cpp)
target_link_libraries(detectionTest fiducial_object) # Necessary for VideoCapture

# Fiducial Object: Generate config
add_executable(generateConfig generateConfig/generateConfig.cpp)
target_link_libraries(generateConfig fiducial_object)

# Generate object map
add_executable(generateMapObject mapperObject/mapperObject.cpp)
target_link_libraries(generateMapObject fiducial_object ${EXTRALIBNAME}aruco ${EXTRALIBNAME}marker_mapper)