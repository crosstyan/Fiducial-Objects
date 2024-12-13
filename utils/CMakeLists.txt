include_directories(${PROJECT_SOURCE_DIR}/src )

# simpleProcessing
add_executable(simpleProcessing fobject_process.cpp )
target_link_libraries(simpleProcessing fiducial_object ${REQUIRED_LIBRARIES} opencv_highgui)
