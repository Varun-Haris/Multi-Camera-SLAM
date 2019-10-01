#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;

class driver{
public:
	ros::NodeHandle nh;
  	image_transport::Publisher pub, pub1, pub2;

  	driver(){
  		image_transport::ImageTransport it(nh);
		this->pub = it.advertise("camera/image/webcam", 1); 
		this->pub1 = it.advertise("camera/image/usb1", 1); 
		this->pub2 = it.advertise("camera/image/usb2", 1); 
  	}

	void publishMsg(sensor_msgs::ImagePtr msg, sensor_msgs::ImagePtr msg1, sensor_msgs::ImagePtr msg2){
		this->pub.publish(msg);
		this->pub1.publish(msg1);
		this->pub2.publish(msg2);
	}
};

int main(int argc, char** argv){
	ros::init(argc, argv, "Driver"); //Driver node
	ros::NodeHandle n;
	uint32_t i=0;
	VideoCapture cap, cam1, cam2;
	
	cap.open("/dev/video0");
	cam1.open("/dev/video1");
	cam2.open("/dev/video2");
	if(!cap.isOpened() || !cam1.isOpened() || !cam2.isOpened()){
		return -1;	
	}

	driver *camDrive = new driver;
	Mat frame, frame1, frame2;
	while(ros::ok()){
		cap >> frame;
		cam1 >> frame1;
		cam2 >> frame2;

		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
		msg->header.stamp = ros::Time::now(); //Adding timestamp
		msg->header.seq = i; //Adding sequence number
		msg->header.frame_id = "Webcam integrated";

		sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame1).toImageMsg();
		msg1->header.stamp = ros::Time::now(); //Adding timestamp
		msg1->header.seq = i; //Adding sequence number
		msg1->header.frame_id = "USB cam 1";

		sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame2).toImageMsg();
		msg2->header.stamp = ros::Time::now(); //Adding timestamp
		msg2->header.seq = i; //Adding sequence number
		msg2->header.frame_id = "USB cam 2";

		camDrive->publishMsg(msg, msg1, msg2);
		i++;
	}
	return 0;
}
