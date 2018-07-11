#ifndef OBJECT_DETECTOR_H
#define OBJECT_DETECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <stdlib.h>

using namespace cv; 

class ObjectDetector
{
  ros::NodeHandle nh; 
  image_transport::ImageTransport it; 
  image_transport::Subscriber sub; 
  image_transport::Publisher pub; 

public:
  ObjectDetector(); 
 // ~ObjectDetector();
  Mat color_segmentation(const Mat &, const unsigned int, const unsigned int);
  Mat edge_detection(const Mat &, unsigned int, unsigned int, unsigned int); 
  void detect_object(const sensor_msgs::ImageConstPtr &);
};

#endif