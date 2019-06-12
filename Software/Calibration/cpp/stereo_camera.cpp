#include "stereo_camera.hpp"

Checkerboard::Checkerboard() {
#ifdef CALIB_A3_40MM
  this->num_corners_hor_ = 8;
  this->num_corners_ver_ = 6;
  this->field_width_ = 40;
  this->num_boards_ = 10;
#elif defined(CALIB_A4_25MM)
  this->num_corners_hor_ = 8;
  this->num_corners_ver_ = 6;
  this->field_width_ = 25;
  this->num_boards_ = 10;
#else
  printf("Enter number of corners along width: ");
  scanf("%d", &this->num_corners_hor_);

  printf("Enter number of corners along height: ");
  scanf("%d", &this->num_corners_ver_);

  printf("Enter  number of boards: ");
  scanf("%d", &this->num_boards_);

  printf("Enter width of a field: ");
  scanf("%d", &this->field_width_);
#endif
  // calculate the total number of corners
  this->num_corners_ = this->num_corners_hor_ * this->num_corners_ver_;
  // define the 3D locations of all corners
  for (int i = 0; i < this->num_corners_; i++) {
    this->corner_locations_.push_back(
        Point3f(i / this->num_corners_hor_ * this->field_width_,
                i % this->num_corners_hor_ * this->field_width_, 0.0f));
  }
}

cv::Size Checkerboard::BoardSize() {
  return Size(this->num_corners_hor_, this->num_corners_ver_);
}

StereoCamera::StereoCamera(int camera_indices[2], bool fixed_config) {
  this->fixed_config_ = fixed_config;
  // Set up both camera captures
  this->cameras_[0] = CameraHandler(false, false);
  this->cameras_[1] = CameraHandler(false, false);
  this->cameras_[0].OpenCamera(camera_indices[0]);
  this->cameras_[1].OpenCamera(camera_indices[1]);
  // Initialize both sets of camera parameters
  this->camera_parameters_[0].camera_index_ = camera_indices[0];
  this->camera_parameters_[1].camera_index_ = camera_indices[1];
  // Make the stereo system ready for use with a dual camera calibration
  DualCalibration();
  // Set up the beamer fixed frame
  SetupBeamerFixedFrame();
  // Explicitly release the captures
  this->cameras_[0].~CameraHandler();
  this->cameras_[1].~CameraHandler();
}

void StereoCamera::CalibrateCamera(int camera_index) {
  // Basic input check - not the real indices but the indices corresponding to
  // the storage array for the two cameras being available
  assert(camera_index == 0 || camera_index == 1);
  // Initialize storage for 2D-3D correspondences
  // first storage for the whole set of boards
  std::vector<std::vector<cv::Point3f>> object_points;
  std::vector<std::vector<cv::Point2f>> image_points;
  // second storage for a single realization
  std::vector<cv::Point2f> image_corners;

  // for counting the number of successfully detected checkerboards
  int successes = 0;

  cv::Mat image = this->cameras_[camera_index].ObtainNextFrame();
  cout << "The current frame resolution is: " << image.size() << endl;
  cv::Mat gray_image;

  while (successes < checkerboard_.num_boards_) {
    // conver the image to grayscale
    cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

    // try to find a checkerboard pattern
    bool found = findChessboardCorners(
        gray_image, this->checkerboard_.BoardSize(), image_corners,
        CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);
    // if found, show the corners on the image
    // does
    if (found) {
      cornerSubPix(
          gray_image, image_corners, Size(11, 11), Size(-1, -1),
          TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 30, 0.1));
      drawChessboardCorners(gray_image, this->checkerboard_.BoardSize(),
                            image_corners, found);
    }

    // Window setup
    DisplayTwoFrames(image, gray_image);

    image = this->cameras_[camera_index].ObtainNextFrame();
    int key = waitKey(1);

    if (key == ' ' && found) {
      image_points.push_back(image_corners);
      object_points.push_back(this->checkerboard_.corner_locations_);

      fprintf(stderr, "Snap stored! ");

      successes++;

      if (successes == this->checkerboard_.num_boards_)
        break;
    }
  }

  std::vector<cv::Mat> rotation_matrices;
  std::vector<cv::Mat> translation_vectors;

  // Use aspect ratio of 1 as a guess for the intrinsics
  this->camera_parameters_[camera_index].intrinsics_ = cv::Mat(3, 3, CV_32FC1);
  this->camera_parameters_[camera_index].intrinsics_.ptr<float>(0)[0] = 1;
  this->camera_parameters_[camera_index].intrinsics_.ptr<float>(1)[1] = 1;

  // Call opencv calibration function
  double rms_reprojection_error = calibrateCamera(
      object_points, image_points, image.size(),
      this->camera_parameters_[camera_index].intrinsics_,
      this->camera_parameters_[camera_index].distortion_coefficients_,
      rotation_matrices, translation_vectors);

  std::cout << "Camera calibration for Camera " << camera_index
            << " done with a reprojection error of " << rms_reprojection_error
            << endl;

  // Exectued if AcquireCameraParameters is able to obtain the sensor size. Note
  // that this only works if the given sensor size directly corresponds to the
  // resolution chosen for image capturing
  if (AcquireCameraParameters()) {
    calibrationMatrixValues(
        this->camera_parameters_[camera_index].intrinsics_, image.size(),
        this->camera_parameters_[camera_index].aperture_width_,
        this->camera_parameters_[camera_index].aperture_height_,
        this->camera_parameters_[camera_index].field_of_view_x_,
        this->camera_parameters_[camera_index].field_of_view_y_,
        this->camera_parameters_[camera_index].focal_length_,
        this->camera_parameters_[camera_index].principal_point_,
        this->camera_parameters_[camera_index].aspect_ratio_);
  }
}

void StereoCamera::CalibrateTwoCameras() {

  // Initialize storage for 2D-3D correspondences
  // first storage for the whole set of boards
  std::vector<std::vector<cv::Point3f>> object_points;
  std::vector<std::vector<cv::Point2f>> image_points[2];

  // second storage for a single realization
  std::vector<cv::Point2f> image_corners[2];

  int successes = 0;

  cv::Mat image[2];
  cv::Mat gray_image[2];
  image[0] = this->cameras_[0].ObtainNextFrame();
  image[1] = this->cameras_[1].ObtainNextFrame();

  while (successes < this->checkerboard_.num_boards_) {
    // convert the image to grayscale
    cv::cvtColor(image[0], gray_image[0], cv::COLOR_BGR2GRAY);
    cv::cvtColor(image[1], gray_image[1], cv::COLOR_BGR2GRAY);

    // try to find a checkerboard pattern
    bool found1 = findChessboardCorners(
        gray_image[0], this->checkerboard_.BoardSize(), image_corners[0],
        CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);
    bool found2 = findChessboardCorners(
        gray_image[1], this->checkerboard_.BoardSize(), image_corners[1],
        CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);

    // if found show the corners on the images
    if (found1) {
      cornerSubPix(
          gray_image[0], image_corners[0], Size(11, 11), Size(-1, -1),
          TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 30, 0.1));
      drawChessboardCorners(gray_image[0], this->checkerboard_.BoardSize(),
                            image_corners[0], found1);
    }
    if (found2) {
      cornerSubPix(
          gray_image[1], image_corners[1], Size(11, 11), Size(-1, -1),
          TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 30, 0.1));
      drawChessboardCorners(gray_image[1], this->checkerboard_.BoardSize(),
                            image_corners[1], found1);
    }

    // Window Setup
    DisplayTwoFrames(gray_image[0], gray_image[1]);
    int key = waitKey(1);

    image[0] = this->cameras_[0].ObtainNextFrame();
    image[1] = this->cameras_[1].ObtainNextFrame();

    if (key == ' ' && found1 && found2) {
      image_points[0].push_back(image_corners[0]);
      image_points[1].push_back(image_corners[1]);
      object_points.push_back(this->checkerboard_.corner_locations_);

      fprintf(stderr, "Snap stored! ");

      successes++;

      if (successes == this->checkerboard_.num_boards_)
        break;
    }
  }

  cv::Mat essential_matrix, fundamental_matrix;

  double rms_reprojection_error = stereoCalibrate(
      object_points, image_points[0], image_points[1],
      this->camera_parameters_[0].intrinsics_,
      this->camera_parameters_[0].distortion_coefficients_,
      this->camera_parameters_[1].intrinsics_,
      this->camera_parameters_[1].distortion_coefficients_, image[0].size(),
      this->camera_2_orientation_, this->camera_2_position_, essential_matrix,
      fundamental_matrix,
      CALIB_FIX_ASPECT_RATIO + CALIB_ZERO_TANGENT_DIST + CALIB_FIX_INTRINSIC +
          CALIB_SAME_FOCAL_LENGTH + CALIB_RATIONAL_MODEL + CALIB_FIX_K3 +
          CALIB_FIX_K4 + CALIB_FIX_K5,
      TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));
  cout << "Stereo Camera Calibration done with rms: " << rms_reprojection_error
       << endl;
}

void StereoCamera::DualCalibration() {

  // Single camera calibrations first, try and read established calibrations
  // if not successful calibrate
  std::string filename = "../calibration_data/calib";
  filename.append(to_string(RESOLUTION));
  filename.append(".yaml");
  cv::FileStorage calibrations(filename, FileStorage::READ);
  if (calibrations.FileStorage::isOpened()) {
    // if file exists read intrinsics from file
    calibrations["K_one"] >> camera_parameters_[0].intrinsics_;
    calibrations["K_two"] >> camera_parameters_[1].intrinsics_;
    calibrations["D_one"] >> camera_parameters_[0].distortion_coefficients_;
    calibrations["D_two"] >> camera_parameters_[1].distortion_coefficients_;
    std::cout << "Calibrations read from file successifully." << std::endl;
  } else {
    // Perform 2 single camera calibrations
    CalibrateCamera(0);
    CalibrateCamera(1);
    // Write to file
    FileStorage file(filename, FileStorage::WRITE);
    file << "K_one" << this->camera_parameters_[0].intrinsics_;
    file << "D_one" << this->camera_parameters_[0].distortion_coefficients_;
    ;
    file << "K_two" << this->camera_parameters_[1].intrinsics_;
    ;
    file << "D_two" << this->camera_parameters_[1].distortion_coefficients_;
    if (camera_parameters_[0].additional_parameters_set_) {
      file << "fovx_one" << this->camera_parameters_[0].field_of_view_x_;
      file << "fovy_one" << this->camera_parameters_[0].field_of_view_y_;
      file << "focalLength_one" << this->camera_parameters_[0].focal_length_;
      file << "aspectRatio_one" << this->camera_parameters_[0].aspect_ratio_;
      file << "principalPoint_one"
           << this->camera_parameters_[0].principal_point_;
      file << "fovx_two" << this->camera_parameters_[1].field_of_view_x_;
      file << "fovy_two" << this->camera_parameters_[1].field_of_view_y_;
      ;
      file << "focalLength_two" << this->camera_parameters_[1].focal_length_;
      file << "aspectRatio_two" << this->camera_parameters_[1].aspect_ratio_;
      file << "principalPoint_two"
           << this->camera_parameters_[1].principal_point_;
    }
    file.release();
  }
  calibrations.release();

  // Stereo camera calibration second
  std::string filename_2 = "../calibration_data/calibstereo";
  filename_2.append(to_string(RESOLUTION));
  filename_2.append(".yaml");

  if (this->fixed_config_) {
    cv::FileStorage calibration_2(filename_2, FileStorage::READ);
    if (calibration_2.FileStorage::isOpened()) {
      calibration_2["R"] >> this->camera_2_orientation_;
      calibration_2["T"] >> this->camera_2_position_;
    } else {
      std::cerr << "No stereo calibration file found, please restart with "
                   "fixed_config = 0."
                << std::endl;
    }
    calibration_2.release();
  } else {
    CalibrateTwoCameras();

    // Write to file
    FileStorage calibration_2(filename_2, FileStorage::WRITE);
    calibration_2 << "R" << this->camera_2_orientation_;
    calibration_2 << "T" << this->camera_2_position_;
    calibration_2.release();
  }
  std::cout << std::endl
            << "The distance between the cameras is estimated as:\n"
            << this->camera_2_position_ << std::endl;
  std::cout << "This amounts to : " << norm(this->camera_2_position_) / 1000
            << " m" << std::endl;
  std::cout << endl
            << "The relative orientation has been estimated as:\n"
            << this->camera_2_orientation_ << std::endl;
  std::cout << std::endl
            << "Written in euler angles this would be:\n"
            << this->EulerAngles() << std::endl;
}

bool StereoCamera::AcquireCameraParameters() {
// Define the camera aperture
#ifdef CAM_LOGITECH_C920PRO
  this->camera_parameters_[0].aperture_width_ = 4.80;  // mm
  this->camera_parameters_[1].aperture_width_ = 4.80;  // mm
  this->camera_parameters_[0].aperture_height_ = 3.60; // mm
  this->camera_parameters_[1].aperture_height_ = 3.60; // mm
  return true;
#endif
  // Implement different cameras here if needed
  return false;
}

std::pair<cv::Mat, cv::Mat> StereoCamera::ProjectionMatrices() {
  // Allocate the homogeneous transformation matrix
  cv::Mat homogeneous_transformation_matrix[2];
  // Identity / no rotation
  cv::Mat rotation_matrix = cv::Mat::eye(3, 3, CV_64F);
  // Zero vector, no translation
  cv::Mat translation_vector = cv::Mat(3, 1, CV_64F, double(0));
  // Build the homogeneous transformation matrix for the first camera
  hconcat(rotation_matrix, translation_vector,
          homogeneous_transformation_matrix[0]);
  // Build the homogeneous transformation matrix for the second camera
  hconcat(this->camera_2_orientation_, this->camera_2_position_,
          homogeneous_transformation_matrix[1]);

  return std::make_pair(
      camera_parameters_[0].intrinsics_ * homogeneous_transformation_matrix[0],
      camera_parameters_[1].intrinsics_ * homogeneous_transformation_matrix[1]);
}

Vec3f StereoCamera::EulerAngles() {
  cv::Mat R = this->camera_2_orientation_;

  float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) +
                  R.at<double>(1, 0) * R.at<double>(1, 0));

  bool singular = sy < 1e-6; // If

  float x, y, z;
  if (!singular) {
    x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
    y = atan2(-R.at<double>(2, 0), sy);
    z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
  } else {
    x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
    y = atan2(-R.at<double>(2, 0), sy);
    z = 0;
  }
  return Vec3f(x, y, z);
}

void StereoCamera::SetupBeamerFixedFrame() {
#ifdef DISTANCE_FROM_CENTER_TO_F1
  // Calculation of the first direction
  cv::Mat inverted_rotation;
  transpose(this->camera_2_orientation_, inverted_rotation);
  cv::Mat translation_vector_f1_to_f2 =
      -inverted_rotation * this->camera_2_position_;
  // std::cout << "Translation vector is: " << translation_vector_f1_to_f2
  //          << std::endl;

  // Calculate first direction as normalized translation vector
  cv::Mat direction_one_mat =
      translation_vector_f1_to_f2 / norm(translation_vector_f1_to_f2);
  // Conversion from matrix to point
  cv::Point3d direction_one(direction_one_mat.at<double>(0),
                            direction_one_mat.at<double>(1),
                            direction_one_mat.at<double>(2));

// Define the horizontal direction. If the cameras are not mounted horizontally
// but at an angle set REDEFINE_HORIZONTAL_DIRECTION in the CMakeLists
#ifdef REDEFINE_HORIZONTAL_DIRECTION
  Point3d old_z_direction = this->GetHorizontalDirection();
#else
  cv::Point3d old_z_direction(0, 0, 1);
#endif
  // Calculate second direction as cross product between z direction (assumed
  // horizontal and new x direction.
  cv::Point3d direction_two = old_z_direction.cross(direction_one);
  // Calculate third direction as cross product between direction one and
  // direction two
  cv::Point3d direction_three = direction_one.cross(direction_two);

  // cout<<"directions:
  // "<<direction_one<<endl<<direction_two<<endl<<direction_three<<endl;
  // If there is such a thing as elegent opencv matrix manipulation tell me
  // please
  cv::Mat rotation_matrix(3, 3, CV_64F);
  rotation_matrix.at<double>(0, 0) = direction_one.x;
  rotation_matrix.at<double>(0, 1) = direction_one.y;
  rotation_matrix.at<double>(0, 2) = direction_one.z;
  rotation_matrix.at<double>(1, 0) = direction_two.x;
  rotation_matrix.at<double>(1, 1) = direction_two.y;
  rotation_matrix.at<double>(1, 2) = direction_two.z;
  rotation_matrix.at<double>(2, 0) = direction_three.x;
  rotation_matrix.at<double>(2, 1) = direction_three.y;
  rotation_matrix.at<double>(2, 2) = direction_three.z;
  // Add another rotation about the new z-axis such that the y-axis is pointing
  // upwards and the x-axis is pointing to the right
  cv::Mat rotation_about_z(3, 3, CV_64F);
  rotation_about_z.at<double>(0, 0) = -1;
  rotation_about_z.at<double>(0, 1) = 0;
  rotation_about_z.at<double>(0, 2) = 0;
  rotation_about_z.at<double>(1, 0) = 0;
  rotation_about_z.at<double>(1, 1) = -1;
  rotation_about_z.at<double>(1, 2) = 0;
  rotation_about_z.at<double>(2, 0) = 0;
  rotation_about_z.at<double>(2, 1) = 0;
  rotation_about_z.at<double>(2, 2) = 1;
  // cout<<"matrix: "<<rotation_matrix<<endl;
  cv::Mat translation_vector(3, 1, CV_64F, double(0));
  translation_vector.at<double>(0) = 1;
  translation_vector *= DISTANCE_FROM_CENTER_TO_F1;
  // cout<<"The translation vector is :"<<translation_vector<<endl;
  // build the homogeneous transformation, leaving away the bottom row
  hconcat(rotation_about_z * rotation_matrix, translation_vector,
          this->homo_trans_from_f1_to_bf_);
// cout<<"The homogeneous transformation matrix is:
// "<<endl<<this->homo_trans_from_f1_to_bf_<<endl;
#else
  std::cerr << "Please define the distance between the beamer fixed frame and "
               "the frame of camera 1 in the CMakeLists of "
               "'dual_calibration_class'"
            << std::endl;
#endif
}

Point3d StereoCamera::TransformToBeamerFixedFrame(Point3d input) {
  // Bring input into homogeneous form
  cv::Mat input_homogeneous(4, 1, CV_64F, double(1));
  input_homogeneous.at<double>(0) = input.x;
  input_homogeneous.at<double>(1) = input.y;
  input_homogeneous.at<double>(2) = input.z;
  // Calculate output
  cv::Mat result_temp = this->homo_trans_from_f1_to_bf_ * input_homogeneous;
  return Point3d(result_temp.at<double>(0), result_temp.at<double>(1),
                 result_temp.at<double>(2));
}

cv::Point3d StereoCamera::TransformToBeamerFixedFrame(cv::Mat input) {
  // Bring input into homogeneous form
  cv::Mat input_homogeneous(4, 1, CV_64F, double(1));
  input_homogeneous.at<double>(0) = input.at<double>(0);
  input_homogeneous.at<double>(1) = input.at<double>(1);
  input_homogeneous.at<double>(2) = input.at<double>(2);
  // Calculate output
  cv::Mat result_temp = this->homo_trans_from_f1_to_bf_ * input_homogeneous;
  // cout<<"Calculation being done is:
  // "<<endl<<this->homo_trans_from_f1_to_bf_<<"\ntimes\n"<<input_homogeneous<<"\nand
  // the resulting matrix is\n"<<result_temp<<endl;
  return cv::Point3d(result_temp.at<double>(0), result_temp.at<double>(1),
                     result_temp.at<double>(2));
}

cv::Point3d StereoCamera::GetHorizontalDirection() {
  cout << "Configuration of the horizontal direction.\n"
       << "Please position your head level with the cameras, faceing them "
          "directly\n";
  cout << "Once your face represents the horizontal direction press <space> to "
          "confirm\n";
  cv::Mat image[2];
  cv::Mat image_modified[2];
  FaceDetection faceDetectCamera[2] = {FaceDetection(), FaceDetection()};
  std::pair<cv::Point3d, cv::Point2d> triangPoint[2];
  std::pair<cv::Matx34d, cv::Matx34d> projectionMatrices;
  projectionMatrices = this->ProjectionMatrices();
  int key = 0;
  while (key != ' ') {
    image[0] = this->cameras_[0].ObtainNextFrame();
    image[1] = this->cameras_[1].ObtainNextFrame();
    faceDetectCamera[0].DetectFacesInImage(image[0]);
    faceDetectCamera[1].DetectFacesInImage(image[1]);
    triangPoint[0] = faceDetectCamera[0].ObtainPointForTriangulation();
    triangPoint[1] = faceDetectCamera[1].ObtainPointForTriangulation();
    image_modified[0] = this->cameras_[0].VisualizePoint(triangPoint[0].second);
    image_modified[1] = this->cameras_[1].VisualizePoint(triangPoint[1].second);
    DisplayTwoFrames(image_modified[0], image_modified[1]);
    key = waitKey(1);
  }
  cv::Mat FaceLocation =
      LinearLSTriangulation(triangPoint[0].first, projectionMatrices.first,
                            triangPoint[1].first, projectionMatrices.second);
  std::cout << "Face location for horizontal direction\n"
            << FaceLocation << std::endl;
  cv::Mat direction_one_mat = FaceLocation / norm(FaceLocation);
  cv::Point3d direction_one(direction_one_mat.at<double>(0),
                            direction_one_mat.at<double>(1),
                            direction_one_mat.at<double>(2));
  std::cout << "Direction one:\n" << direction_one << std::endl;
  cv::destroyAllWindows();
  return direction_one;
}

void DisplayTwoFrames(cv::Mat frame1, cv::Mat frame2) {
  // Open the two named windows
  cv::namedWindow("window1", WINDOW_NORMAL);
  cv::namedWindow("window2", WINDOW_NORMAL);
  // Resize windows to standard 1080p screen
  cv::resizeWindow("window1", 960, 540);
  cv::resizeWindow("window2", 960, 540);
  cv::imshow("window1", frame1);
  cv::imshow("window2", frame2);
  moveWindow("window2", 960, 0);
}
