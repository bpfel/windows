#include "camera_handler.hpp"

//Constructor
CameraHandler::CameraHandler(){
  this->showTriangulationPoint = false;
  this->showFaceOrientation = false;
  frame = cv::Mat::zeros(1,1,cv::DataType<double>::type);
}

//COnstructor for visualization purposes
CameraHandler::CameraHandler(bool showTriangulation, bool showFaceOrientation){
  this->showTriangulationPoint = showTriangulation;
  this->showFaceOrientation = showFaceOrientation;
  frame = cv::Mat::zeros(1,1,cv::DataType<double>::type);
}

//Rotate the image so that it is correctly positioned
void CameraHandler::OrientView (){
  #ifdef CMOUNT_INVERTED
      flip(frame, frame, -1);
  #endif
}

//Open the camera, set resolution and focus with the given ID
bool CameraHandler::OpenCamera(int cameraID){
  try
  {
    cap = VideoCapture(cameraID);
    if (!cap.isOpened())
    {
      cerr << "Unable to connect to camera" << endl;
      return false;
    }
      SetResolution();
      SetFocus();
      return true;
    }
    catch(exception& e)
    {
      cout << e.what() << endl;
      return false;
    }
}

//Return the next camera frame in the correct orientation
Mat CameraHandler::ObtainNextFrame(){
  if(cap.isOpened()){
    // Grab a frame
    if ( !cap.read(frame) ){  // the camera might need some warmup
      std::cout<<"No read frame"<<std::endl;
    }
    else{
      //Rotate the frame by 180deg
      OrientView();
    }
  }
  else{
    std::cout<<"Camera not opened"<<std::endl;
  }
  return frame;
}

//Returns the image with the visualized face orientation and triangulation point
Mat CameraHandler::VisualizePointAndOrientation(Point2d faceOrientationOrigin, Point2d faceOrientationEnd, Point2d triangPoint){
  if(showFaceOrientation){
    cv::line(frame,faceOrientationOrigin, faceOrientationEnd, cv::Scalar(255,0,0), 2);
  }
  if(showTriangulationPoint){
    cv::circle(frame, triangPoint, 3, cv::Scalar(0,0,255), -1);
  }
  return frame;
}

//Returns the image with the visualized triangulation point
Mat CameraHandler::VisualizePoint(Point2d triangPoint){
  if(showTriangulationPoint){
    cv::circle(frame, triangPoint, 3, cv::Scalar(0,0,255), -1);
  }
  return frame;
}

//Sets the camera resolution
void CameraHandler::SetResolution(){
    //if defined set the camera to the desired resolution
#if RESOLUTION==1080
    this->cap.set(CAP_PROP_FRAME_WIDTH, 1920);
    this->cap.set(CAP_PROP_FRAME_HEIGHT, 1080);
#elif RESOLUTION==720
    this->cap.set(CAP_PROP_FRAME_WIDTH, 1280);
    this->cap.set(CAP_PROP_FRAME_HEIGHT, 720);
#elif RESOLUTION==480
    this->cap.set(CAP_PROP_FRAME_WIDTH, 858);
    this->cap.set(CAP_PROP_FRAME_HEIGHT, 480);
#elif RESOLUTION==360
    this->cap.set(CAP_PROP_FRAME_WIDTH, 480);
    this->cap.set(CAP_PROP_FRAME_HEIGHT, 360);
#endif
}

//Sets the camera focus
void CameraHandler::SetFocus(){
    this->cap.set(CAP_PROP_AUTOFOCUS, false);
    this->cap.set(CAP_PROP_FOCUS, 0);
}

//Release the oppened camera
CameraHandler::~CameraHandler(){
    this->cap.release();
}
