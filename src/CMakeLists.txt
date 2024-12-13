INCLUDE_DIRECTORIES(.)


SET(    srcs
    fiducialObject.cpp
        polyhuco/polyhuco.cpp
        polyhuco/polyhucoPoseFunctions.cpp
        polyhuco/polyhucoDrawingFunctions.cpp
        utils/mapProcessor.cpp
        utils/frameDetector.cpp
        utils/generalMarker.cpp
        utils/generalMarkerDetector.cpp
        utils/markerDetectors/cucomarkerMarkerDetector.cpp
        utils/utils.cpp

)


INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR} ${libdbus_INCLUDE_DIRS})
SET(hdrs      fiducialobject_exports.h fiducialObject.h polyhuco/levmarq.h  polyhuco/polyhuco.h  polyhuco/polyhucoPoseFunctions.hpp utils/utils.hpp)

ADD_LIBRARY(${PROJECT_NAME} ${srcs} ${hdrs})

SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES 				# create *nix style library versions + symbolic links
        DEFINE_SYMBOL FIDUCIALOBJECT_DSO_EXPORTS
        VERSION ${PROJECT_VERSION}
        SOVERSION ${PROJECT_SOVERSION}
        CLEAN_DIRECT_OUTPUT 1 										# allow creating static and shared libs without conflicts
        OUTPUT_NAME "${PROJECT_NAME}${PROJECT_DLLVERSION}" 	# avoid conflicts between library and binary target names
)

Find_Package(OpenMP)
if (OPENMP_FOUND)
        set(OPENMP_LOCAL_LIBS ${OpenMP_CXX_LIBRARIES})
endif()

TARGET_LINK_LIBRARIES(${PROJECT_NAME}  ${REQUIRED_LIBRARIES} ${REQUIRED_BOOST_LIBRARIES} ${libhal_LIBS} ${OPENMP_LOCAL_LIBS})

INSTALL(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin COMPONENT main
        LIBRARY DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE COMPONENT main
        ARCHIVE DESTINATION lib COMPONENT main)

INSTALL(FILES ${hdrs}
        DESTINATION include/${PROJECT_NAME}
        COMPONENT main)

