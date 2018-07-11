#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "object_detector.hpp"

#define MAX_SATURATION 255
#define MIN_SATURATION 100
#define MIN_VALUE 100
#define MAX_VALUE 255
#define WAIT_TIME 1
#define WIDTH 640
#define HEIGHT 480
#define LB_HUE_RED 170
#define UB_HUE_RED 179
#define LB_HUE_GREEN 47 
#define UB_HUE_GREEN 84
#define LB_HUE_YELLOW 20
#define UB_HUE_YELLOW 40

using namespace cv;
using namespace std; 

static const string OPENCV_WINDOW = "ImageWindow";

ObjectDetector::ObjectDetector() : it(nh)
{
  sub = it.subscribe("/usb_cam/image_raw",1,&ObjectDetector::detect_object, this);
  pub = it.advertise("/object_detector/image_treshold", 1);
}

//ObjectDetector::~ObjectDetector() { }

Mat ObjectDetector::color_segmentation(const Mat &input_img, const unsigned int LowH, const unsigned int HighH)
{ 
  Mat hsv_img, treshold_img; 
  int LowV = MIN_VALUE; 
  int HighV = MAX_VALUE; 
  int LowS = MIN_SATURATION; 
  int HighS = MAX_SATURATION; 

  cvtColor(input_img, hsv_img, COLOR_BGR2HSV);
  inRange(hsv_img, Scalar(LowH, LowS, LowV), Scalar(HighH, HighS, HighV), treshold_img); // Thresholded image
      
  // morphological opening (remove small objects from the foreground)
  erode(treshold_img, treshold_img, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  dilate(treshold_img, treshold_img, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

  // morphological closing (fill small holes in the foreground)
  dilate(treshold_img, treshold_img, getStructuringElement(MORPH_ELLIPSE, Size(5, 5))); 
  erode(treshold_img, treshold_img, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

  return treshold_img; 
}

Mat ObjectDetector::edge_detection(const Mat &treshold_img, const unsigned int r, const unsigned int g, const unsigned int b)
{
  Mat contours_img;
  Mat drawing;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  // Edge detection and contour extraction using Canny Edge Detector
  Canny(treshold_img, contours_img, 200, 255*2, 3);
  findContours(contours_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE,Point(0,0));
      
  drawing = Mat::zeros(contours_img.size(), CV_8UC3);
 
  for(int i = 0; i < contours.size(); i++)
  {
    Scalar color = Scalar(b,g,r);
    drawContours(drawing,contours,i,color,3);
  }

  return drawing;
}

void ObjectDetector::detect_object(const sensor_msgs::ImageConstPtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv_bridge::CvImage cv_contour_img;  
  Mat treshold_img_red, treshold_img_green, treshold_img_yellow, contour_yellow, contour_red, contour_green;
  Mat treshold_img, contour_img; 
  sensor_msgs::ImagePtr ros_contour_img;

  try 
  {   
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch(cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s",e.what());  
  }

  treshold_img_red = ObjectDetector::color_segmentation(cv_ptr->image, LB_HUE_RED, UB_HUE_RED);
  treshold_img_green = ObjectDetector::color_segmentation(cv_ptr->image, LB_HUE_GREEN, UB_HUE_GREEN);
  treshold_img_yellow = ObjectDetector::color_segmentation(cv_ptr->image, LB_HUE_YELLOW, UB_HUE_YELLOW);
  contour_red = ObjectDetector::edge_detection(treshold_img_red,255,0,0);
  contour_green = ObjectDetector::edge_detection(treshold_img_green,0,255,0);
  contour_yellow = ObjectDetector::edge_detection(treshold_img_yellow,255,255,0);
  
  treshold_img = treshold_img_red + treshold_img_green + treshold_img_yellow; 
  contour_img = contour_red + contour_green + contour_yellow;

  #ifdef OPEN_IMAGE_STREAM
    namedWindow(OPENCV_WINDOW,CV_WINDOW_AUTOSIZE);
    imshow("Treshold Images",treshold_img);
    imshow("Contour Images",contour_img);
    waitKey(WAIT_TIME);
  #endif

  cv_contour_img.encoding = "mono8";
  cv_contour_img.image = treshold_img;

  ros_contour_img = cv_contour_img.toImageMsg();
  ros_contour_img->width=WIDTH;
  ros_contour_img->height=HEIGHT;
  ros_contour_img->step=WIDTH;

  pub.publish(ros_contour_img);
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"detect_color");
  ObjectDetector detector;
  ros::spin(); 
  return 0;
}
