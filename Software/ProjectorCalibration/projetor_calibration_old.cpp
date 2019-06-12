//
//  main.cpp
//  Accessing webcam
//
//  Created by Uschatz Cédric on 18.03.19.
//  Copyright © 2019 Uschatz Cédric. All rights reserved.
//

#include <iostream>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;
int main() {
    // Project four images with the resolution of the projector (1280x800px) where one corner pixel is white and the rest black.
    // and capture it with the camera and save it as a gray image.
    // 1) white pixel top left
    Mat image1 = imread("topleft.jpg",0);
    if(!image1.data)
    {
        cout << "Could not open or find the image" << endl;
        return -1;
    }
    namedWindow("white pixel top left", WINDOW_AUTOSIZE);
    imshow("white pixel top left",image1);
    waitKey(0);
    // capture the image with the camera
    VideoCapture cap1(0);
    
    //Get the frame
    Mat topleft;
    if (!cap1.isOpened()){
        return -1;
    }
    else {
        cap1 >> topleft;
    }
    Mat topleft_gray;
    cvtColor(topleft,topleft_gray, COLOR_BGR2GRAY);
    
    // 2) white pixel top right
    Mat image2 = imread("topright.jpg",0);
    if(!image2.data)
    {
        cout << "Could not open or find the image" << endl;
        return -1;
    }
    namedWindow("white pixel top right", WINDOW_AUTOSIZE);
    imshow("white pixel top right",image2);
    waitKey(0);
    // capture the image with the camera
    VideoCapture cap2(0);
    
    //Get the frame
    Mat topright;
    if (!cap2.isOpened()){
        return -1;
    }
    else {
        cap2 >> topright;
    }
    Mat topright_gray;
    cvtColor(topright,topright_gray, COLOR_BGR2GRAY);
    
    // 3) white pixel bottom left
    Mat image3 = imread("bottomleft.jpg",0);
    if(!image3.data)
    {
        cout << "Could not open or find the image" << endl;
        return -1;
    }
    namedWindow("white pixel bottom left", WINDOW_AUTOSIZE);
    imshow("white pixel bottom left",image3);
    waitKey(0);
    // capture the image with the camera
    VideoCapture cap3(0);
    
    //Get the frame
    Mat bottomleft;
    if (!cap3.isOpened()){
        return -1;
    }
    else {
        cap3 >> bottomleft;
    }
    Mat bottomleft_gray;
    cvtColor(bottomleft,bottomleft_gray, COLOR_BGR2GRAY);
    
    // 4) white pixel bottom right
    Mat image4 = imread("bottomright.jpg",0);
    if(!image4.data)
    {
        cout << "Could not open or find the image" << endl;
        return -1;
    }
    namedWindow("white pixel bottom right", WINDOW_AUTOSIZE);
    imshow("white pixel bottom right",image4);
    waitKey(0);
    // capture the image with the camera
    VideoCapture cap4(0);
    
    //Get the frame
    Mat bottomright;
    if (!cap4.isOpened()){
        return -1;
    }
    else {
        cap4 >> bottomright;
    }
    Mat bottomright_gray;
    cvtColor(bottomright,bottomright_gray, COLOR_BGR2GRAY);
    
    // project at last a completely black image in order to capture the background
    Mat image5 = imread("background.jpg",0);
    if(!image5.data)
    {
        cout << "Could not open or find the image" << endl;
        return -1;
    }
    namedWindow("background", WINDOW_AUTOSIZE);
    imshow("background",image5);
    waitKey(0);
    // capture the image with the camera
    VideoCapture cap5(0);
    
    //Get the frame
    Mat background;
    if (!cap5.isOpened()){
        return -1;
    }
    else {
        cap5 >> background;
    }
    
    Mat background_gray;
    cvtColor(background,background_gray, COLOR_BGR2GRAY);
    // remove background image from the other 4 images
    Mat topleft_diff = topleft_gray- background_gray;
    namedWindow( "topleft", WINDOW_AUTOSIZE );
    imshow( "topleft", topleft_diff );
    waitKey(0);
    Mat topright_diff = topright_gray - background_gray;
    namedWindow( "topright", WINDOW_AUTOSIZE );
    imshow( "topright", topright_diff );
    waitKey(0);
    Mat bottomleft_diff = bottomleft_gray - background_gray;
    namedWindow( "bottomleft", WINDOW_AUTOSIZE );
    imshow( "bottomleft", bottomleft_diff );
    waitKey(0);
    Mat bottomright_diff = bottomright_gray - background_gray;
    namedWindow( "bottomright", WINDOW_AUTOSIZE );
    imshow( "bottomright", bottomright_diff );
    waitKey(0);
    // get the maximum value pixel (since the camera has a higher resolution than the projector
    // the white pixel of the projector falls onto several camera
    // pixels resulting in some small errors)
    Point max_loc_t1; Point max_loc_t2; Point max_loc_t3; Point max_loc_t4;
    minMaxLoc(topleft_diff,NULL,NULL,NULL,&max_loc_t1);
    minMaxLoc(topright_diff,NULL,NULL,NULL,&max_loc_t2);
    minMaxLoc(bottomleft_diff,NULL,NULL,NULL,&max_loc_t3);
    minMaxLoc(bottomright_diff,NULL,NULL,NULL,&max_loc_t4);
    // write camera points into a 4x2 matrix
    vector<Point2d> cam_points;
    cam_points.push_back(max_loc_t1); cam_points.push_back(max_loc_t2);
    cam_points.push_back(max_loc_t3); cam_points.push_back(max_loc_t4);
    Mat_<cv::Point2d> cam_points_mat(cam_points);

    // write projector points into a 4x2 matrix
    vector<Point2d> proj_points = {Point(0,0), Point(1280,0), Point(0,800), Point(1280,800)};
    Mat_<cv::Point2d> proj_points_mat(proj_points);
    
    // TRIANGULATE POINTS OF WEBCAM AND PROJECTOR
    // Read from .yml projector camera calibration file
    FileStorage cal_mat("calib.yml",FileStorage::READ);
    Mat cam_K; Mat cam_kc; Mat proj_K; Mat proj_kc; Mat proj_R; Mat proj_T;
    cal_mat["cam_K"]>>cam_K; cal_mat["cam_kc"]>>cam_kc;cal_mat["proj_K"]>>proj_K;cal_mat["proj_kc"]>>proj_kc;
    cal_mat["R"]>>proj_R; cal_mat["T"]>>proj_T;
    
    // multiply first row of projector intrinsics by two, since the calibration resolved for lower resolution and scaled the higher by 0.5
    Mat temp = 2 * proj_K.row(0);
    temp.copyTo(proj_K.row(0));
    // projection matrices for triangulation
    Mat projconcat; Mat camconcat;
    hconcat(proj_R,proj_T,projconcat);
    Mat projmat = proj_K * projconcat;
    // since  the world coordinate system has been fixed to the center of projection of the camera in the calibraton software, R_cam = I and T_cam = 0
    Mat id = Mat::eye(3, 3, CV_64F); Mat zr = Mat::zeros(3, 1, CV_64F);
    hconcat(id,zr,camconcat);
    Mat cammat = cam_K * camconcat;
    int num_imgpoints = 4;
    // output Matrix of triangulated 3D points in homogenous coordinates
    cv::Mat points3D_hom(4,num_imgpoints,CV_64F);
    
    triangulatePoints(cammat,projmat,cam_points_mat, proj_points_mat, points3D_hom);
    
    // get the x,y,z coordinates of the 4 points in the camera coordinate frame
    double x1 = points3D_hom.at<double>(0,0)/points3D_hom.at<double>(3,0);
    double y1 = points3D_hom.at<double>(1,0)/points3D_hom.at<double>(3,0);
    double z1 = points3D_hom.at<double>(2,0)/points3D_hom.at<double>(3,0);
    double x2 = points3D_hom.at<double>(0,1)/points3D_hom.at<double>(3,1);
    double y2 = points3D_hom.at<double>(1,1)/points3D_hom.at<double>(3,1);
    double z2 = points3D_hom.at<double>(2,1)/points3D_hom.at<double>(3,1);
    double x3 = points3D_hom.at<double>(0,2)/points3D_hom.at<double>(3,2);
    double y3 = points3D_hom.at<double>(1,2)/points3D_hom.at<double>(3,2);
    double z3 = points3D_hom.at<double>(2,2)/points3D_hom.at<double>(3,2);
    double x4 = points3D_hom.at<double>(0,3)/points3D_hom.at<double>(3,3);
    double y4 = points3D_hom.at<double>(1,3)/points3D_hom.at<double>(3,3);
    double z4 = points3D_hom.at<double>(2,3)/points3D_hom.at<double>(3,3);

    // Transform into beamer coordinates R*X+T
    Mat cam_3Dpoints = (Mat_<double>(3,4)<<x1,x2,x3,x4,y1,y2,y3,y4,z1,z2,z3,z4);
    hconcat(proj_T,proj_T,proj_T); hconcat(proj_T,proj_T,proj_T);
    Mat proj_3Dpoints = proj_R*cam_3Dpoints+proj_T;
    //  take the mean and divide by 1000 to get the distance in meters
    double Z = (proj_3Dpoints.at<double>(2,0)+proj_3Dpoints.at<double>(2,1)+proj_3Dpoints.at<double>(2,2)+proj_3Dpoints.at<double>(2,3))/(4*1000);
    // write it into a projcalibout.yml file
    FileStorage fs("projcalibout.yml", FileStorage::WRITE);
    fs << "DISTANCE TO THE WALL " << Z;
    fs.release();
    return 0;
    
}

