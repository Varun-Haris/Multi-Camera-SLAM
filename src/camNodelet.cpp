#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/transport_hints.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/bind.hpp>
#include <opencv2/highgui/highgui.hpp>

class camSubs{
public:
	ros::NodeHandle nh;
	image_transport::Subscriber sub, sub1, sub2;
	image_transport::ImageTransport it;

	camSubs():it(nh){
		this->sub = it.subscribe("camera/image/webcam", 1, &camSubs::imageCbWeb, this);
		this->sub1 = it.subscribe("camera/image/usb1", 1, &camSubs::imageCbUsb1, this);
		this->sub2 = it.subscribe("camera/image/usb2", 1, &camSubs::imageCbUsb2, this);
		ROS_INFO("Subscriber setup!!");
	} 

  	void imageCbWeb(const sensor_msgs::ImageConstPtr& msg){
  		std::cout << "webcam header : \n" << msg->header << std::endl;
  		try{
  			cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    		cv::waitKey(1);
  		}
  		catch(cv_bridge::Exception& e){
  			ROS_ERROR("Couldn't get image due to %s",e.what());
  		}
  	}

  	void imageCbUsb1(const sensor_msgs::ImageConstPtr& msg){
  		std::cout << "usb1 header : \n" << msg->header << std::endl;
  		try{
  			cv::imshow("view1", cv_bridge::toCvShare(msg, "bgr8")->image);
    		cv::waitKey(1);
  		}
  		catch(cv_bridge::Exception& e){
  			ROS_ERROR("Couldn't get image due to %s",e.what());
  		}
  	}

  	void imageCbUsb2(const sensor_msgs::ImageConstPtr& msg){
  		std::cout << "usb2 header : \n" << msg->header << std::endl;
  		try{
  			cv::imshow("view2", cv_bridge::toCvShare(msg, "bgr8")->image);
    		cv::waitKey(1);
  		}
  		catch(cv_bridge::Exception& e){
  			ROS_ERROR("Couldn't get image due to %s",e.what());
  		}
  	}
};

int main(int argc, char** argv){
	ros::init(argc, argv, "Image_Listener");
	ros::NodeHandle n;

	cv::namedWindow("view");
  	cv::startWindowThread();
  	
  	cv::namedWindow("view1");
  	cv::startWindowThread();
  
  	cv::namedWindow("view2");
  	cv::startWindowThread();

	camSubs *Subs = new camSubs;
	while(ros::ok()){
		ros::spinOnce();
	}
	
	cv::destroyWindow("view");
	cv::destroyWindow("view1");
	cv::destroyWindow("view2");
}
