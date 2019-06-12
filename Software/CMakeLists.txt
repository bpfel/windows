cmake_minimum_required( VERSION 3.5.1 )
project( windows )
set( CMAKE_CXX_STANDARD 11 )
set( SOURCE_FILES windows.cpp )

add_executable( windows ${SOURCE_FILES} )

target_compile_definitions( windows PUBLIC RENDER_SCENE )
#target_compile_definitions( windows PUBLIC SHOW_FACEDETECTION )
#target_compile_definitions( windows PUBLIC CHECK_TIME )

# add the local camera handler library
include_directories( "${PROJECT_SOURCE_DIR}/CameraHandler" )
link_directories( "${PROJECT_SOURCE_DIR}/CameraHandler" )
add_subdirectory ( CameraHandler )

# add the local face detection library
include_directories( "${PROJECT_SOURCE_DIR}/FaceDetection" )
link_directories( "${PROJECT_SOURCE_DIR}/FaceDetection" )
add_subdirectory ( FaceDetection )

# add the local calibration library
include_directories( "${PROJECT_SOURCE_DIR}/Calibration/cpp" )
link_directories( "${PROJECT_SOURCE_DIR}/Calibration/cpp" )
add_subdirectory( Calibration/cpp )

# add the local calibration library
include_directories( "${PROJECT_SOURCE_DIR}/ProjectorCalibration" )
link_directories( "${PROJECT_SOURCE_DIR}/ProjectorCalibration" )
add_subdirectory( ProjectorCalibration )

# same for the scene
include_directories( "${PROJECT_SOURCE_DIR}/Scene" "${PROJECT_SOURCE_DIR}/Scene/includes")
link_directories( "${PROJECT_SOURCE_DIR}/Scene" "${PROJECT_SOURCE_DIR}/Scene/includes" )
add_subdirectory ( Scene )

find_package( OpenCV REQUIRED )

target_link_libraries( windows
    ${OpenCV_LIBS}
    FaceDetection
    Calibration
    WindowScene
    ${SCENELIBS} GLAD STB_IMAGE dl
    CameraHandler
    ProjectorCalibration
  )

#choose what kind of output you'd like
target_compile_definitions( windows PUBLIC RENDER_SCENE )
#target_compile_definitions( windows PUBLIC SHOW_FACEDETECTION )
target_compile_definitions( windows PUBLIC CHECK_TIME )

#enter resolution specification form among 1080p, 720p, 480p, 360p as an integer
set( RES 360 )
target_compile_definitions(CameraHandler PUBLIC RESOLUTION=${RES})
target_compile_definitions(Calibration PUBLIC RESOLUTION=${RES})

#choose the calibration pattern
target_compile_definitions(Calibration PUBLIC CALIB_A3_40MM=TRUE )
#target_compile_definitions(Calibration PUBLIC CALIB_A4_25MM=TRUE )

#choose the camera
target_compile_definitions(Calibration PUBLIC CAM_LOGITECH_C920PRO=TRUE )

#decide whether the cameras are mounted inverted
target_compile_definitions(CameraHandler PUBLIC CMOUNT_INVERTED )
#decide whether the cameras are mounted horizontally or at an angle
target_compile_definitions(Calibration PUBLIC REDEFINE_HORIZONTAL_DIRECTION )

#define the distance from camera frame 1 (on the right) to the beamer centered frame
target_compile_definitions(Calibration PUBLIC DISTANCE_FROM_CENTER_TO_F1=550 )

