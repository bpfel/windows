#ifndef FACE_DETECTION_H
#define FACE_DETECTION_H

#include <dlib/gui_widgets.h>
#include <dlib/image_processing.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/opencv.h>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

using namespace dlib;
using namespace std;

/**
 * @brief Detects a single face within a camera capture and passes the position
 * and orientation of the face.
 */

class FaceDetection {
private:
  /**
   * @brief Intrinsic camera matrix
   */
  cv::Mat camera_matrix;

  /**
   * @brief Distant coefficients of the camera
   */
  cv::Mat dist_coeffs;

  /**
   * @brief Face detection object
   */
  frontal_face_detector detector;

  /**
   * @brief Vector with detected faces
   */
  std::vector<rect_detection> detected_faces;

  /**
   * @brief Contains the 3D position on camera coordinates and the 2D position
   * over the image of the triangulation point
   */
  pair<cv::Point3d, cv::Point2d> lastTriansgulationPoints;

public:
  /**
   * @brief Face Detection constructor
   */
  FaceDetection();

  /**
   * @brief Detects all the faces of a given image
   *
   * @param Image where the detection is made
   */
  void DetectFacesInImage(cv::Mat frame);

  /**
   * @brief Obtains the point (in 2D and 3D) where the triangulation will be
   * done for the main detected face
   *
   * @return 2D and 3D coordinates of the point for triangulation
   */
  pair<cv::Point3d, cv::Point2d> ObtainPointForTriangulation();

  /**
   * @brief Sets the internals of the camera
   *
   * @param camera_matrix intrinsic camera matrix
   *
   * @param dist_coeffs distant coefficients of the camera
   */
  void SetCameraInternals(cv::Mat camera_matrix, cv::Mat dist_coeffs);
};

/**
 * @brief Performs the triangulation given two points
 *
 * @param u Position of the point in the first camera coordinates
 *
 * @param P Projection matrix of the first camera
 *
 * @param u1 Position of the point in the second camera coordinates
 *
 * @param P1 Projection matrix of the second camera
 *
 * @return returns the 3D face location
 */
cv::Mat_<double> LinearLSTriangulation(cv::Point3d u, cv::Matx34d P,
                                       cv::Point3d u1, cv::Matx34d P1);

/**
 * @brief Performs the whole face detection.
 *
 * @param frame A camera frame in which the face is detected
 *
 * @param returned_values Pair containing the triangulation point
 *
 * @param face_detection Face detection object that is used to detect the images
 */
void DetectFace(cv::Mat frame, pair<cv::Point3d, cv::Point2d> &returned_values,
                FaceDetection &face_detection);
#endif
