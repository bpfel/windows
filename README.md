# Abstract

In the past few years video cameras, computer vision algorithms and the hardware needed to run them have been made readily available to the public. With this project we present the integration of three webcams, a beamer and the relevant software to produce the illusion of a virtual window for a single user. We endeavour to present the viewer with an immersive experience that catches the eye and invites to linger.

We have established a projector calibration for computing the depth to the virtual window and its dimension, a calibrated stereo camera system enabling close to real time face detection and triangulation, a virtual three dimensional world with fast view rendering, a rack for mounting the hardware components and a single executable incorporating the whole  online pipeline.



# Installation

## Platform

The platform of choice is Ubuntu 16.04 Xenial.

## Basic Software

```sh
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install build-essential cmake unzip pkg-config git curl -y
```

## OpenCV-Dependencies

```sh
sudo apt-get install libjpeg-dev libpng-dev libtiff-dev -y
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev -y
sudo apt-get install libxvidcore-dev libx264-dev -y
sudo apt-get install libgtk-3-dev -y
sudo apt-get install libatlas-base-dev gfortran -y
```

## OpenCV 4.0.0

Download and unpack OpenCV

```sh
cd ~
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.0.0.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.0.0.zip
unzip opencv.zip
unzip opencv_contrib.zip
mv opencv-4.0.0 opencv
mv opencv_contrib-4.0.0 opencv_contrib
```

Compile OpenCV

```sh
cd ~/opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
-D CMAKE_INSTALL_PREFIX=/usr/local \
-D INSTALL_C_EXAMPLES=OFF \
-D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules ..
make -j8
sudo make install
sudo ldconfig
```

## OpenGL Dependencies

Install homebrew and add it to the path to help with installing the OpenGL dependencies.

```sh
sudo apt-get install libsoil-dev libglm-dev libassimp-dev libglew-dev libglfw3-dev libxinerama-dev libxcursor-dev libxi-dev -y
```
## Install windows

```sh
cd
git clone --recursive <repo-url>
cd windows
vim Software/Scene/includes/learnopengl/filesystem.h
```

Now edit line 22 such that envRoot contains the absolute path to 'windows/Software'.

```sh
cd Software
mkdir build
cd build
cmake ..
make -j8
```

# How to set up

1. Connect the cameras in order. First camera 1 (which is on the right), second camera 2 (which is on the left), third camera 3 which looks at the  virtual window. USB addresses are given depending on the temporal order of connection.
2. Run the program. In the first execution all calibration processes take place. After that most of the calibration data is saved and read from calibration files.

## Calibration

1. Single camera calibration, followed by a stereo camera calibration. Use one of the [calibration patterns](Software/Calibration/CalibrationPatterns) and specify it in the [CMakeLists](Software/CMakeLists.txt). Record the requested amount of frames, confirming each frame by pressing `<space>`.
2. Connect and set up the beamer. It will be needed for step 4. 
3. Horizontal direction calibration. Stand in front of the stereo camera such that your head is on the same height as the cameras. Confirm with `<space>` to record the location of your face as a representation of the horizontal direction.
4. Beamer-Wall-Distance calculation. 5 Frames will be shown, the first 4 containing a white pixel, the last completely black. Pace through them with `<space>`, waiting for each of them to load properly.
5. Enjoy the window. 

# Dependencies

*  [OpenCV](https://sourceforge.net/projects/opencvlibrary/files/opencv-win/3.4.3/)
*  [Dlib](http://dlib.net)
*  [OpenGL](https://www.opengl.org)

