cmake_minimum_required( VERSION 3.5.1 )
set( CMAKE_CXX_STANDARD 11 )

add_library( ProjectorCalibration projector_calibration.cpp )

find_package( OpenCV REQUIRED )

target_link_libraries( ProjectorCalibration ${OpenCV_LIBS})
