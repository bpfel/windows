#include "projector_calibration.hpp"

double distance_to_wall(int cam_3) {

  // Project four images with the resolution of the projector (1280x800px) where
  // one corner pixel is white and the rest black.
  // and capture it with the camera and save it as a gray image.

  // Get the images
  std::cout << "folder is" << folder + "topleft.jpg" << std::endl;
  cv::Mat image1 = cv::imread(folder + "topleft.jpg", 0);
  cv::Mat image2 = cv::imread(folder + "topright.jpg", 0);
  cv::Mat image3 = cv::imread(folder + "bottomleft.jpg", 0);
  cv::Mat image4 = cv::imread(folder + "bottomright.jpg", 0);
  cv::Mat image5 = cv::imread(folder + "background.jpg", 0);

  if (!image1.data || !image2.data || !image3.data || !image4.data ||
      !image5.data) {
    std::cerr << "Could not open or find the image" << std::endl;
    return -1;
  }

  // set up video capture
  cv::VideoCapture cap(cam_3);
  if (!cap.isOpened()) {
    std::cerr << "Could not open the camera\n";
    return -1;
  }

  // Allocate the needed images
  cv::Mat topleft, topleft_gray, topright, topright_gray, bottomleft,
      bottomleft_gray, bottomright, bottomright_gray, background,
      background_gray;

  // 1) white pixel top left
  cv::namedWindow("white pixel top left", cv::WINDOW_NORMAL);
  cv::setWindowProperty("white pixel top left", cv::WND_PROP_FULLSCREEN,
                        cv::WINDOW_FULLSCREEN);
  cv::imshow("white pixel top left", image1);
  std::cout << "Press <space> to capture the next image.\n";
  cv::waitKey(0);
  cap >> topleft;
  cv::cvtColor(topleft, topleft_gray, cv::COLOR_BGR2GRAY);
  // Empty buffer forcefully
  cap.release();
  cap.open(cam_3);

  // 2) white pixel top right
  cv::namedWindow("white pixel top right", cv::WINDOW_NORMAL);
  cv::setWindowProperty("white pixel top right", cv::WND_PROP_FULLSCREEN,
                        cv::WINDOW_FULLSCREEN);
  cv::imshow("white pixel top right", image2);
  std::cout << "Press <space> to capture the next image.\n";
  cv::waitKey(0);
  cap.grab();
  cap >> topright;
  cv::cvtColor(topright, topright_gray, cv::COLOR_BGR2GRAY);
  // Empty buffer forcefully
  cap.release();
  cap.open(cam_3);

  // 3) white pixel bottom left
  cv::namedWindow("white pixel bottom left", cv::WINDOW_NORMAL);
  cv::setWindowProperty("white pixel bottom left", cv::WND_PROP_FULLSCREEN,
                        cv::WINDOW_FULLSCREEN);
  cv::imshow("white pixel bottom left", image3);
  std::cout << "Press <space> to capture the next image.\n";
  cv::waitKey(0);
  cap.grab();
  cap >> bottomleft;
  cv::cvtColor(bottomleft, bottomleft_gray, cv::COLOR_BGR2GRAY);
  // Empty buffer forcefully
  cap.release();
  cap.open(cam_3);

  // 4) white pixel bottom right
  cv::namedWindow("white pixel bottom right", cv::WINDOW_NORMAL);
  cv::setWindowProperty("white pixel bottom right", cv::WND_PROP_FULLSCREEN,
                        cv::WINDOW_FULLSCREEN);
  cv::imshow("white pixel bottom right", image4);
  std::cout << "Press <space> to capture the next image.\n";
  cv::waitKey(0);
  cap.grab();
  cap >> bottomright;
  cv::cvtColor(bottomright, bottomright_gray, cv::COLOR_BGR2GRAY);
  // Empty buffer forcefully
  cap.release();
  cap.open(cam_3);

  // project at last a completely black image in order to capture the background
  cv::namedWindow("background", cv::WINDOW_NORMAL);
  cv::setWindowProperty("background", cv::WND_PROP_FULLSCREEN,
                        cv::WINDOW_FULLSCREEN);
  cv::imshow("background", image5);
  std::cout << "Press <space> to capture the background image.\n";
  cv::waitKey(0);
  cap.grab();
  cap >> background;
  cv::cvtColor(background, background_gray, cv::COLOR_BGR2GRAY);
  cv::destroyAllWindows();

  // remove background image from the other 4 images
  cv::Mat topleft_diff = topleft_gray - background_gray;
  cv::Mat topright_diff = topright_gray - background_gray;
  cv::Mat bottomleft_diff = bottomleft_gray - background_gray;
  cv::Mat bottomright_diff = bottomright_gray - background_gray;
  cv::waitKey(0);

  // get the maximum value pixel (since the camera has a higher resolution than
  // the projector
  // the white pixel of the projector falls onto several camera
  // pixels resulting in some small errors)
  cv::Point max_loc_t1;
  cv::Point max_loc_t2;
  cv::Point max_loc_t3;
  cv::Point max_loc_t4;
  cv::minMaxLoc(topleft_diff, NULL, NULL, NULL, &max_loc_t1);
  cv::minMaxLoc(topright_diff, NULL, NULL, NULL, &max_loc_t2);
  cv::minMaxLoc(bottomleft_diff, NULL, NULL, NULL, &max_loc_t3);
  cv::minMaxLoc(bottomright_diff, NULL, NULL, NULL, &max_loc_t4);

  // write camera points into a 4x2 matrix
  std::vector<cv::Point2d> cam_points;
  cam_points.push_back(max_loc_t1);
  cam_points.push_back(max_loc_t2);
  cam_points.push_back(max_loc_t3);
  cam_points.push_back(max_loc_t4);
  cv::Mat_<cv::Point2d> cam_points_mat(cam_points);

  // write projector points into a 4x2 matrix
  std::vector<cv::Point2d> proj_points = {cv::Point(0, 0), cv::Point(1280, 0),
                                          cv::Point(0, 800),
                                          cv::Point(1280, 800)};
  cv::Mat_<cv::Point2d> proj_points_mat(proj_points);

  // TRIANGULATE POINTS OF WEBCAM AND PROJECTOR
  // Read from .yml projector camera calibration file
  cv::FileStorage cal_mat(fn_calibration_data, cv::FileStorage::READ);
  if (!cal_mat.FileStorage::isOpened()) {
    std::cerr << "Not opened" << std::endl;
  }
  cv::Mat cam_K;
  cv::Mat proj_K;
  cv::Mat proj_R;
  cv::Mat proj_T;
  cal_mat["cam_K"] >> cam_K;
  cal_mat["proj_K"] >> proj_K;
  cal_mat["R"] >> proj_R;
  cal_mat["T"] >> proj_T;

  // multiply first row of projector intrinsics by two, since the calibration
  // resolved for lower resolution and scaled the higher by 0.5
  cv::Mat temp = 2 * proj_K.row(0);
  temp.copyTo(proj_K.row(0));
  // projection matrices for triangulation
  cv::Mat projconcat;
  cv::Mat camconcat;
  cv::hconcat(proj_R, proj_T, projconcat);
  cv::Mat projmat = proj_K * projconcat;
  // since  the world coordinate system has been fixed to the center of
  // projection of the camera in the calibraton software, R_cam = I and T_cam =
  // 0
  cv::Mat id = cv::Mat::eye(3, 3, cv::DataType<double>::type);
  cv::Mat zr = cv::Mat::zeros(3, 1, cv::DataType<double>::type);
  cv::hconcat(id, zr, camconcat);
  cv::Mat cammat = cam_K * camconcat;
  int num_imgpoints = 4;
  // output Matrix of triangulated 3D points in homogenous coordinates
  cv::Mat points3D_hom(4, num_imgpoints, cv::DataType<double>::type);

  cv::triangulatePoints(cammat, projmat, cam_points_mat, proj_points_mat,
                        points3D_hom);

  // get the x,y,z coordinates of the 4 points in the camera coordinate frame
  double x1 = points3D_hom.at<double>(0, 0) / points3D_hom.at<double>(3, 0);
  double y1 = points3D_hom.at<double>(1, 0) / points3D_hom.at<double>(3, 0);
  double z1 = points3D_hom.at<double>(2, 0) / points3D_hom.at<double>(3, 0);
  double x2 = points3D_hom.at<double>(0, 1) / points3D_hom.at<double>(3, 1);
  double y2 = points3D_hom.at<double>(1, 1) / points3D_hom.at<double>(3, 1);
  double z2 = points3D_hom.at<double>(2, 1) / points3D_hom.at<double>(3, 1);
  double x3 = points3D_hom.at<double>(0, 2) / points3D_hom.at<double>(3, 2);
  double y3 = points3D_hom.at<double>(1, 2) / points3D_hom.at<double>(3, 2);
  double z3 = points3D_hom.at<double>(2, 2) / points3D_hom.at<double>(3, 2);
  double x4 = points3D_hom.at<double>(0, 3) / points3D_hom.at<double>(3, 3);
  double y4 = points3D_hom.at<double>(1, 3) / points3D_hom.at<double>(3, 3);
  double z4 = points3D_hom.at<double>(2, 3) / points3D_hom.at<double>(3, 3);

  // Transform into beamer coordinates R*X+T
  cv::Mat cam_3Dpoints = (cv::Mat_<double>(3, 4) << x1, x2, x3, x4, y1, y2, y3,
                          y4, z1, z2, z3, z4);
  cv::hconcat(proj_T, proj_T, proj_T);
  cv::hconcat(proj_T, proj_T, proj_T);
  cv::Mat proj_3Dpoints = proj_R * cam_3Dpoints + proj_T;
  //  take the mean and divide by 1000 to get the distance in meters
  double Z = (proj_3Dpoints.at<double>(2, 0) + proj_3Dpoints.at<double>(2, 1) +
              proj_3Dpoints.at<double>(2, 2) + proj_3Dpoints.at<double>(2, 3)) /
             (4 * 1000);
  double distance_from_sensor_to_beamer_fixed_frame = 0.1;
  Z = Z + distance_from_sensor_to_beamer_fixed_frame;
  // write it into a projcalibout.yml file
  cv::FileStorage fs(fn_wall_distance, cv::FileStorage::WRITE);
  fs << "distance" << Z;
  // Calculate scale factors for finding the actual frame size
  double xscale = proj_K.at<double>(0, 2) * 2 / proj_K.at<double>(0, 0);
  double yscale = proj_K.at<double>(1, 2) / proj_K.at<double>(1, 1);
  fs << "xscale" << xscale;
  fs << "yscale" << yscale;
  fs.release();
  return Z;
}

double get_distance_to_wall(int cam_3, bool fixed_config) {
  cv::FileStorage fs(fn_wall_distance, cv::FileStorage::READ);
  if (fixed_config && fs.FileStorage::isOpened()) {
    double dist;
    fs["distance"] >> dist;
    return dist;
  } else {
    double dist_to_wall = distance_to_wall(cam_3);
    return dist_to_wall;
  }
}

double get_frame_width() {
  cv::FileStorage fs(fn_wall_distance, cv::FileStorage::READ);
  if (fs.FileStorage::isOpened()) {
    double dist, xscale;
    fs["distance"] >> dist;
    fs["xscale"] >> xscale;
    // Multiply the distance with the slope of the field of view
    return dist * xscale;
  } else {
    std::cerr << "Distance to wall not found, run get_distance_to_wall "
                 "first\n";
    return -1;
  }
}

double get_frame_height() {
  cv::FileStorage fs(fn_wall_distance, cv::FileStorage::READ);
  if (fs.FileStorage::isOpened()) {
    double dist, yscale;
    fs["distance"] >> dist;
    fs["yscale"] >> yscale;
    // Multiply the distance with the slope of the field of view
    return dist * yscale;
  } else {
    std::cerr << "Distance to wall not found, run get_distance_to_wall "
                 "first\n";
    return -1;
  }
}
