cmake_minimum_required( VERSION 3.5.1 )
set( CMAKE_CXX_STANDARD 11 )

add_library( Calibration stereo_camera.cpp )

find_package( OpenCV REQUIRED )

target_link_libraries( Calibration ${OpenCV_LIBS} CameraHandler FaceDetection)

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/../.. )

target_include_directories( Calibration
    INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR/build}>
    $<INSTALL_INTERFACE:include>
    )

install( TARGETS Calibration DESTINATION bin )
install( FILES stereo_camera.hpp DESTINATION include )
