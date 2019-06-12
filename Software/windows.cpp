/**
 \file windows.cpp
 \brief Main file
 */

#include "projector_calibration.hpp"
#include "src/windowscene.h"
#include "stereo_camera.hpp"
#include <dlib/ref.h>
#include <dlib/threads.h>
#include <time.h>

using namespace std;

int main(int argc, char *argv[]) {
  // Check runtime arguments
  if (argc < 7) {
    cerr << "Usage: ./windows <cameraIndex1> <cameraIndex2> <cameraIndex3> "
            "<fixed_config 0 or 1> <fullscreen 0 or 1> <frame 0 or 1>"
         << endl;
    return -1;
  }
  /// Read runtime arguments
  int camera_indices[3] = {atoi(argv[1]), atoi(argv[2]), atoi(argv[3])};
  bool fixed_config = atoi(argv[4]);
  bool fullscreen = atoi(argv[5]);
  bool render_frame = atoi(argv[6]);

  /// some doxydoc
  ///
  Mat cameraDistance;
  Mat cameraOrientation;
  Mat intrinsics[2];
  Mat distCoeffs[2];
  pair<Matx34d, Matx34d> projectionMatrices;
  Mat R_1 = Mat::eye(3, 3, CV_64F);
  Mat T_1 = Mat(3, 1, CV_64F, double(0));

  /// Calibration
  StereoCamera stereo_camera(camera_indices, fixed_config);
  cameraDistance = stereo_camera.camera_2_position_;
  cameraOrientation = stereo_camera.camera_2_orientation_;
  intrinsics[0] = stereo_camera.camera_parameters_[0].intrinsics_;
  intrinsics[1] = stereo_camera.camera_parameters_[1].intrinsics_;
  distCoeffs[0] = stereo_camera.camera_parameters_[0].distortion_coefficients_;
  distCoeffs[1] = stereo_camera.camera_parameters_[1].distortion_coefficients_;
  projectionMatrices = stereo_camera.ProjectionMatrices();

  /// CameraHandler
  CameraHandler cameraHandlers_one = CameraHandler(true, true);
  CameraHandler cameraHandlers_two = CameraHandler(true, true);
  /// Open the cameras
  cameraHandlers_one.OpenCamera(camera_indices[0]);
  cameraHandlers_two.OpenCamera(camera_indices[1]);

  /// Face Detection
  FaceDetection faceDetectCamera_one = FaceDetection();
  FaceDetection faceDetectCamera_two = FaceDetection();

  /// Pass the camera internal parameters
  faceDetectCamera_one.SetCameraInternals(intrinsics[0], distCoeffs[0]);
  faceDetectCamera_two.SetCameraInternals(intrinsics[1], distCoeffs[1]);

  // Allocate necessary output variables
  pair<cv::Point3d, cv::Point2d> face_location_one;
  pair<cv::Point3d, cv::Point2d> face_location_two;
  Mat frameModified_one;
  Mat frameModified_two;

  // Calculate beamer to wall distance
  float wall_distance = get_distance_to_wall(camera_indices[2], fixed_config);

#ifdef RENDER_SCENE
  WindowScene scene;
  if (scene.initialize(fullscreen) != 0) {
    cout << "Failed to initialize scene" << endl;
    return -1;
  }
  scene.passProjectorPosition(0.0f, 0.0f, wall_distance);
  scene.passProjectedWindow(get_frame_width(), get_frame_height());
  if (render_frame)
    scene.toggleFrame();

  while (scene.isActive()) {
#else

  while (true) {
#endif

    thread_function t1(DetectFace, cameraHandlers_one.ObtainNextFrame(),
                       dlib::ref(face_location_one),
                       dlib::ref(faceDetectCamera_one));
    thread_function t2(DetectFace, cameraHandlers_two.ObtainNextFrame(),
                       dlib::ref(face_location_two),
                       dlib::ref(faceDetectCamera_two));
    t1.wait();
    t2.wait();

    // Perspective Update
    Mat FaceLocation = LinearLSTriangulation(
        face_location_one.first, projectionMatrices.first,
        face_location_two.first, projectionMatrices.second);
    // Calculate the face position in the beamer fixed frame
    Point3d wall_distance(0.0, 0.0, 0.756);
    Point3d face_location_bf =
        stereo_camera.TransformToBeamerFixedFrame(FaceLocation);
    // Get FaceLocation in br-frame, convert from mm to m and add wall distance
    // in m
    Point3d face_location_wf =
        stereo_camera.TransformToBeamerFixedFrame(FaceLocation) / 1000 +
        wall_distance;

#ifdef RENDER_SCENE
    // eye position
    scene.passEyePosition(face_location_wf.x, face_location_wf.y,
                          face_location_wf.z);
    scene.render();
#else

#ifdef SHOW_FACEDETECTION
    // View the face detections
    frameModified_one =
        cameraHandlers_one.VisualizePoint(face_location_one.second);
    frameModified_two =
        cameraHandlers_two.VisualizePoint(face_location_two.second);
    DisplayTwoFrames(frameModified_one, frameModified_two);
#endif

    if (cv::waitKey(30) == 27)
      break;
#endif
  }

#ifdef RENDER_SCENE
  scene.terminate();
#endif
  return 0;
}
