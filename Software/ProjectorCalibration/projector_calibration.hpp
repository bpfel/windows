//
//  main.cpp
//  Accessing webcam
//
//  Created by Uschatz Cédric on 18.03.19.
//  Copyright © 2019 Uschatz Cédric. All rights reserved.
//

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>

/**
   * @brief Location of the calibration files
   */
static std::string folder = "../calibration_data/";
/**
 * @brief Location of the beamer and camera calibration files.
 */
static std::string fn_calibration_data = folder + "calib_proj.yml";
/**
 * @brief Location of the file saving the wall distance.
 */
static std::string fn_wall_distance = folder + "projcalibout.yml";

/**
 * @brief Calculates the distance to the wall using the precalculated beamer and
 * camera calibrations as well as a set of 4 images with white pixel corner
 * locations and a black image for compensating for background illumination. The
 * resulting distance is saved in a file.
 *
 * @param cam_3 Current index of the wall-facing camera.
 *
 * @return Distance to the wall.
 *
 */
double distance_to_wall(int cam_3);

/**
 * @brief Calls the distance to wall calculation if either the configuration is
 * not fixed yet or the configuration file containing the distance has not yet
 * been created.
 *
 * @param cam_3 Current index of the wall-facing camera.
 *
 * @param fixed_config True if the configuration is set up and calibrated
 * already.
 *
 * @return Distance to the wall.
 */
double get_distance_to_wall(int cam_3, bool fixed_config);

/**
 * @brief After the distance to the wall is measured the frame width can be
 * calculated, based on the aperture angle of the BenQ-GP10
 *
 * @return The width of the frame
 */
double get_frame_width();

/**
 * @brief After the distance to the wall is measured the frame height can be
 * calculated, based on the aperture angle of the BenQ-GP10
 *
 * @return The height of the frame
 */
double get_frame_height();
