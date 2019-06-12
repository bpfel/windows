#ifndef STEREO_CAMERA_H
#define STEREO_CAMERA_H

#include "opencv2/opencv.hpp"
#include <camera_handler.hpp>
#include <cassert>
#include <ctime>
#include <face_detection.hpp>
#include <iostream>
#include <opencv2/core/utility.hpp>
#include <string.h>

/**
 * @brief Saves the camera parameters
 */
struct CameraParameters {
  /**
   * @brief Saves the index of the device
   */
  int camera_index_ = -1;
  /**
   * @brief 3x3 matrix containing the camera intrinsics
   */
  cv::Mat intrinsics_;
  /**
   * @brief Matrix containing the distortion coefficients
   */
  cv::Mat distortion_coefficients_;
  /**
   * @brief True if the additional parameters have been calculated. This can
   * only be done when the size of the sensor at the current resolution is
   * known.
   */
  bool additional_parameters_set_ = false;
  /**
   * @brief Field of view in x direction
   */
  double field_of_view_x_ = -1;
  /**
   * @brief Field of view in y direction
   */
  double field_of_view_y_ = -1;
  /**
   * @brief Focal length of the camera in the same units chosen for the
   * chessboard pitch in the camera calibration.
   */
  double focal_length_ = -1;
  /**
   * @brief Aspect Ratio of the frame
   */
  double aspect_ratio_ = -1;
  /**
   * @brief Width of the camera sensor
   */
  double aperture_width_ = 0;
  /**
   * @brief Heigth of the camera sensor
   */
  double aperture_height_ = 0;
  /**
   * @brief Principal point of the sensor given in the same units chosen for the
   * chessboard pitch in the camera calibration.
   */
  cv::Point2d principal_point_ = Point2d(-1, -1);
};

/**
 * @brief Saves the parameters of the checkerboard pattern used for calibration.
 */
struct Checkerboard {
  /**
   * @brief Constructor. Defines the checkerboard parameters as hardcoded if
   * either CALIB_A3_40MM or CALIB_A4_25MM are set. Otherwise a dialog is
   * invoked.
   */
  Checkerboard();
  /**
   * @brief Returns the width and height of the board.
   *
   * @return Variable of type Size containing width and height of the board.
   */
  cv::Size BoardSize();
  /**
   * @brief Saves the number of internal coners in horizontal direction.
   */
  int num_corners_hor_ = -1;
  /**
   * @brief Saves the number of internal corners in vertical direction.
   */
  int num_corners_ver_ = -1;
  /**
   * @brief Saves the width/height of a checkerboard square in millimeters.
   */
  double field_width_ = -1;
  /**
   * @brief Saves the number of images taken for a single calibration.
   */
  int num_boards_ = -1;

  /**
   * @brief Saves the total number of corners
   */
  int num_corners_ = -1;

  std::vector<cv::Point3f> corner_locations_;
};

class StereoCamera {
private:
  /**
   * @brief Checkerboard saving the parameters of the used calibration pattern.
   */
  Checkerboard checkerboard_;
  /**
   * @brief Camera handlers used to access cameras.
   */
  CameraHandler cameras_[2];
  /**
   * @brief If set the calibration will be read from file if existing.
   */
  bool fixed_config_;
  /**
   * @brief CalibrateTwoCameras captures two video streams and regocnizes a
   * checkerboard calibration pattern. From that it extracts the corners and
   * feeds them to calibrateCamera from openCV.
   */
  void CalibrateTwoCameras();
  /**
   * @brief This function performs a camera calibration for a single camera
   *
   * @param camera_index Describes which camera to calibrate. Put 0 for the
   * first camera and 1 for the second camera.
   */
  void CalibrateCamera(int camera_index);
  /**
  * @brief Perform a camera calibration for each camera, then calibrate the two
  * as a stereo camera and return their relative position and orientation.
  */
  void DualCalibration();
  /**
   * @brief Sets the aperture width and the aperture height for both cameras
   * according to the camera specified in the CMakeLists.
   *
   * Supported cameras: CAM_LOGITECH_C920PRO
   *
   * @return Returns true if the values were set successfully
   */
  bool AcquireCameraParameters();

public:
  /**
   * @brief Constructor. Sets up both camera handlers, executes the calibration
   * of both cameras and of the resulting stereo camera.
   *
   * @param camera_indices Indices of the cameras to be used for calibration.
   */
  StereoCamera(int camera_indices[2], bool fixed_config);
  /**
   * @brief Calculates the projection matrix of the selected camera.
   *
   * @return Returns the projection matrices as a pair of Mat.
   */
  std::pair<cv::Mat, cv::Mat> ProjectionMatrices();
  /**
  * @brief  Calculates the relative orientation of the second camera in euler
  * angles, based on an already performed calibration.
  *
  * source: https://www.learnopencv.com/rotation-matrix-to-euler-angles/
  *
  * @return Returns the rotations about x, y and z
  */
  cv::Vec3f EulerAngles();
  /**
   * @brief Saves the parameters of both cameras. The coordinate system of the
   * first camera is used as the reference frame.
   */
  CameraParameters camera_parameters_[2];
  /**
   * @brief A rotation matrix bringing a vector represented in frame 1 to its
   * representation in frame 2.
   */
  cv::Mat camera_2_orientation_;
  /**
   * @brief A connection vector from frame 2 to frame 1, represented in frame 2
   * coordinates.
   */
  cv::Mat camera_2_position_;

  /**
   * @brief A homogeneous transformation matrix bringing a vector from a
   * representation in the frame of camera 1 to the representation in the beamer
   * fixed frame.
   */
  cv::Mat homo_trans_from_f1_to_bf_;
  /**
   * @brief Populates the homogeneous transformation matrix that brings vectors
   * from camera 1 frame to the beamer fixed frame.
   *
   * @param distance_from_center_to_f1 Describes the distance from camera 1 to
   * the beamer center in mm.
   */
  void SetupBeamerFixedFrame();

  /**
   * @brief Represents a vector in the camera 1 frame in the beamer fixed frame
   *
   * @param input Input vector.
   *
   * @return Output vector.
   */
  Point3d TransformToBeamerFixedFrame(cv::Point3d input);
  Point3d TransformToBeamerFixedFrame(cv::Mat input);

  /**
   * @brief Shows both camera views detecting a face and registers the current
   * face position on cue (spacebar) to define the horizontal direction, when
   * the cameras are tilted upwards.
   */
  cv::Point3d GetHorizontalDirection();
};

// Helper Functions
/**
 * @brief Displays two frames side by side
 *
 * @param frame1
 * @param frame2
 */
void DisplayTwoFrames(cv::Mat frame1, cv::Mat frame2);
#endif
