#include "face_detection.hpp"


//Constructor of the face detection
FaceDetection::FaceDetection(){
  //Initialize frontal_face_detector
  detector = get_frontal_face_detector();
  //Initialize variables with default values
  cv::Point3d triangulationPoint = cv::Point3d(0,0,1);
  cv::Point2d triangShowPoint = cv::Point2d(0,0);
  lastTriansgulationPoints = make_pair(triangulationPoint, triangShowPoint);
}

//Sets the camera internals
void FaceDetection::SetCameraInternals(cv::Mat camera_matrix, cv::Mat dist_coeffs){
  this->camera_matrix = camera_matrix;
  this->dist_coeffs = dist_coeffs; // Assuming no lens distortion
}

void FaceDetection::DetectFacesInImage(cv::Mat frame){
  // Turn OpenCV's Mat into something dlib can deal with
  cv_image<bgr_pixel> cimg(frame);
  // Detect faces
  detector(cimg, detected_faces);
}

//Return 2D and 3D position of center of the bounding box of the detected person
pair<cv::Point3d, cv::Point2d> FaceDetection::ObtainPointForTriangulation(){
  //Initialize with default values
  cv::Point3d triangulationPoint = cv::Point3d(0,0,1);
  cv::Point2d triangShowPoint = cv::Point2d(0,0);
  //Obtain the center of the bounding box
  //Compute the new triangulation point if at least one person is detected
  if(detected_faces.size() > 0){
    //Obtain the center of the bounding box
	  rectangle const& rect = detected_faces[0].rect;
    double x = (rect.left() + rect.right()) / 2;
    double y = (rect.top() + rect.bottom()) / 2;
    triangulationPoint = cv::Point3d(x,y,1);
    triangShowPoint = cv::Point2d(x,y);
    lastTriansgulationPoints = make_pair(triangulationPoint, triangShowPoint);
  }
  return lastTriansgulationPoints;
}

//Wraps the triangulation and the face detection
void DetectFace(cv::Mat frame, pair<cv::Point3d, cv::Point2d> &returned_values, FaceDetection & face_detection){
    //Execute the face detection
    face_detection.DetectFacesInImage(frame);
    //obtain triangulation point
    returned_values = face_detection.ObtainPointForTriangulation();
}

/**
 From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
 Adapted by Team 7
 */
cv::Mat_< double > LinearLSTriangulation(
        cv::Point3d u,       //homogenous image point (u,v,1)
        cv::Matx34d P,       //camera 1 matrix K_1[I|0]
        // P contains the projection matrix of camera 1 composed of the intrinsics and the homogeneous transform describing the position of camera 1 which in this case is the identity.
        cv::Point3d u1,      //homogenous image point in 2nd camera
        cv::Matx34d P1       //camera 2 matrix K_2[R|T]
        // P1 containts the projection matrix of camera 2 composed of the intrinsics and the homogeneous transform describing the position of camera 2 w.r.t. camera 1.
        )
{
    //build matrix A for homogenous equation system Ax = 0
    //assume X = (x,y,z,1), for Linear-LS method
    //which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1
    cv::Matx43d A(u.x*P(2,0)-P(0,0),    u.x*P(2,1)-P(0,1),      u.x*P(2,2)-P(0,2),
          u.y*P(2,0)-P(1,0),    u.y*P(2,1)-P(1,1),      u.y*P(2,2)-P(1,2),
          u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),   u1.x*P1(2,2)-P1(0,2),
          u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),   u1.y*P1(2,2)-P1(1,2)
              );
    cv::Mat_<double> B = (cv::Mat_<double>(4,1) <<    -(u.x*P(2,3)    -P(0,3)),
                      -(u.y*P(2,3)  -P(1,3)),
                      -(u1.x*P1(2,3)    -P1(0,3)),
                      -(u1.y*P1(2,3)    -P1(1,3)));

    cv::Mat_<double> X;
    cv::solve(A,B,X,cv::DECOMP_SVD);

    return X;
}
