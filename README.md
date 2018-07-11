# ObjectDetector
Object Detection using OpenCV and ROS Interface

## Table of Contents

## Description
This package includes an object detection framework which can be used with ROS for any robot application you prefer.

## Dependencies
* ROS Kinetic 
* OpenCV 2.4.9 
* ROS [usb_cam driver](https://github.com/ros-drivers/usb_cam) (should also work with [libuvc](https://github.com/ktossell/libuvc))

## Status 
The package has been tested with Ubuntu 16.04, ROS Kinetic and OpenCV 2.4.9 using the usb_cam driver package.

## Install
* Install ROS: see instructions [here](http://wiki.ros.org/ROS/Installation)
* [Setup the ROS environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
* download and install [OpenCV](https://docs.opencv.org/3.3.1/d7/d9f/tutorial_linux_install.html) and cam driver packages. 
* run `catkin build`
* source `source devel/setup.bash`
* start the usb cam driver, e.g. `rosrun usb_cam usb_cam_node`
* start the object detector using `rosrun color_detector color_detector`
* to see the output via ROS you could use a GUI e.g. `rqt_image_view` or direct line command `rosrun image_view image_view image:=/object_detector/image_treshold` 

## Topics
Basically the object detector can either output the thresholded image or the contour image (or both if you want). Therefore, two topics are essential 
* **`/object_detector/image_treshold`**

outputs the tresholded image with parameters *`width=640`*, *`height=480`*, *`step=640`*, *`encoding="mono8"`*
* **`/object_detector/contour_treshold`**

outputs the contour image with parameters *`width=640`*, *`height=480`*, *`step=640`*, *`encoding="mono8"`*

## Troubleshooting
Always ensure that the usb_cam driver is running *before* you start the object detector node. 
