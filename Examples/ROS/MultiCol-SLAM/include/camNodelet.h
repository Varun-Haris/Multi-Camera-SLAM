#ifndef CAMNODELET_H
#define CAMNODELET_H

// Standard c++ headers
#include <vector>
#include <algorithm>
#include <boost/make_shared.hpp>

// ROS related headers
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/transport_hints.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>

// OpenCV headers
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;

class camSubs{
public:
	ros::NodeHandle nh;
	image_transport::Subscriber sub, sub1, sub2;
	image_transport::ImageTransport it;
	cv::Mat frame1, frame2, frame3;
	std_msgs::Header header1, header2, header3;

	camSubs();
	void imageCbWeb(const sensor_msgs::ImageConstPtr& msg);
	void imageCbUsb1(const sensor_msgs::ImageConstPtr& msg);
	void imageCbUsb2(const sensor_msgs::ImageConstPtr& msg);
}; //End of class

#endif
