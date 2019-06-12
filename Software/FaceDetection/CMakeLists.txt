cmake_minimum_required( VERSION 3.5.1 )
set( CMAKE_CXX_STANDARD 11 )

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

add_library( FaceDetection face_detection.cpp )

add_subdirectory( ${CMAKE_CURRENT_SOURCE_DIR}/dlib dlib_build )

find_package( OpenCV REQUIRED )
target_link_libraries( FaceDetection ${OpenCV_LIBS} dlib::dlib)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)

