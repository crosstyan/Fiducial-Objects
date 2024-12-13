add_subdirectory(tools/qtgl)
qt5_add_resources( RCC fobject.qrc )
include_directories(${PROJECT_SOURCE_DIR}/3rdparty/cucomarker/print_marker/3rdparty/tinyxml2)
SET(print_marker_SRCS
    ${PROJECT_SOURCE_DIR}/3rdparty/cucomarker/print_marker/svgreader.h
${PROJECT_SOURCE_DIR}/3rdparty/cucomarker/print_marker/svgreader.cpp
${PROJECT_SOURCE_DIR}/3rdparty/cucomarker/print_marker/parser.h
${PROJECT_SOURCE_DIR}/3rdparty/cucomarker/print_marker/3rdparty/tinyxml2/tinyxml2.cpp
)
SET(uis_src_fobject   fiducialobjectqt.h fiducialobjectqt.cpp arucocalibrate.cpp arucocalibrate.h drawers3d.h)
QT5_WRAP_UI(uis_fobject     fiducialobjectqt.ui)
add_executable(fobject_qt main.cpp    ${uis_src_fobject} ${uis_fobject} ${print_marker_SRCS} ${RCC} )
qt5_use_modules(fobject_qt   Widgets  Network  )

target_link_libraries(fobject_qt   ${OpenCV_LIBS} aruco marker_mapper fiducial_object plutovg qtgl)



INSTALL(TARGETS fobject_qt     RUNTIME DESTINATION bin)
IF(WIN32)
     CONFIGURE_FILE( "${CMAKE_CURRENT_SOURCE_DIR}/installer_script.iss.in" "${CMAKE_CURRENT_SOURCE_DIR}/installer_script.iss" IMMEDIATE @ONLY)
   INSTALL(FILES "program_icon.ico" DESTINATION bin/ )
elseif(UNIX)

ENDIF()
