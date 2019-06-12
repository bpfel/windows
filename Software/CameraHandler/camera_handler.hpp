#ifndef CAMERA_HANDLER_H
#define CAMERA_HANDLER_H

#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

/**
 * @brief CameraHandler can be used to stream video from a connected camera.
 */
class CameraHandler {
protected:
  /**
   * @brief When set, the point representing the face position is shown in the
   * output image.
   */
  bool showTriangulationPoint;
  /**
   * @brief When set, the line representing face orientation is shown in the
   * output image.
   */
  bool showFaceOrientation;

  /**
   * @brief Stores one frame of the capture.
   */
  cv::Mat frame;

  /**
   * @brief Rotates the frames by 180 in case CMOUNT_INVERTED is set.
   */
  void OrientView();

  /**
  * @brief Set the resolution of the handed camera to the chosen parameters
  * REGULAR_WIDTH and REGULAR_HEIGHT
   */
  void SetResolution();
  /**
   * @brief Sets the camera focus to infinity and turns autofocus off
   */
  void SetFocus();
  /**
   * @brief Stores the video capture of the camera handler.
   */
  cv::VideoCapture cap;

public:
  /**
   * @brief Default constructor, sets showTriangulationPoint and
   * showFaceOrientation to false.
   */
  CameraHandler();
  /**
   * @brief Constructor
   *
   * @param showTriangulation When set, the point representing the face position
   is shown in the output image.

   * @param showFaceOrientation When set, the line representing face orientation
   is shown in the output image.

   */
  CameraHandler(bool showTriangulation, bool showFaceOrientation);
  /**
   * @brief Initializes the capture by handing it a VideoCapture of a certain
   * cameraID and checks whether the given ID corresponds to an existing device.
   *
   * @param cameraID ID of a connected device.
   *
   * @return True if connection was successful.
   */
  bool OpenCamera(int cameraID);
  /**
   * @brief Tries to obtain the next frame and if successful, orients it
   * correctly.
   *
   * @return Mat containing the next frame.
   */
  Mat ObtainNextFrame();
  /**
   * @brief Adds markings for face orientation and face position on frame if
   * showFaceOrientation/showTriangulationPoint are set.
   *
   * @param faceOrientationOrigin Represents the projection of the beginning of
   * the face orientation vector.
   * @param faceOrientationEnd Represents the projection of the end of the face
   * orientation vector.
   * @param triangPoint Represent the position of the face.
   *
   * @return Mat containing the updated frame.
   */
  Mat VisualizePointAndOrientation(Point2d faceOrientationOrigin,
                                   Point2d faceOrientationEnd,
                                   Point2d triangPoint);
  /**
   * @brief Adds markings for  face position on frame if shoTriangulationPoint
   * are set.
   *
   * @param triangPoint Represent the position of the face.
   *
   * @return Mat containing the updated frame.
   */
  Mat VisualizePoint(Point2d triangPoint);

  /**
   * @brief Destructor. Releases the capture.
   */
  ~CameraHandler();
};
#endif
