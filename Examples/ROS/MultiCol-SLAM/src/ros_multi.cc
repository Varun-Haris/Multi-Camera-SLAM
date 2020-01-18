// The original version was released under the following license
/**
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

// All modifications are released under the following license
/**
* This file is part of MultiCol-SLAM
*
* Copyright (C) 2015-2016 Steffen Urban <rurbste at googlemail.com>
* For more information see <https://github.com/urbste/MultiCol-SLAM>
*
* MultiCol-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* MultiCol-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with MultiCol-SLAM . If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <fstream> 
#include <iomanip>
#include <thread>
#include <mutex>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "cTracking.h"
#include "cConverter.h"
#include "cam_model_omni.h"
#include "cSystem.h"
#include "camNodelet.h"

using namespace cv;
using namespace std;

camSubs::camSubs():it(nh){
	sub = it.subscribe("/camera/image/webcam", 1, &camSubs::imageCbWeb, this);
	sub1 = it.subscribe("/camera/image/usb1", 1, &camSubs::imageCbUsb1, this);
	sub2 = it.subscribe("/camera/image/usb2", 1, &camSubs::imageCbUsb2, this);
	ROS_INFO("Subscriber setup!!");
} 

void camSubs::imageCbWeb(const sensor_msgs::ImageConstPtr& msg){
	try{
      	header1 = msg->header;
    	cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  		frame1 = img->image;
  	}
  	catch(cv_bridge::Exception& e){
  		ROS_ERROR("Couldn't get image due to %s",e.what());
  	}
}

void camSubs::imageCbUsb1(const sensor_msgs::ImageConstPtr& msg){
	try{
   		header2 = msg->header;
		cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
 		frame2 = img->image;
	}
 	catch(cv_bridge::Exception& e){
 		ROS_ERROR("Couldn't get image due to %s",e.what());
 	}
}

void camSubs::imageCbUsb2(const sensor_msgs::ImageConstPtr& msg){
	try{
    	header3 = msg->header;
		cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		frame3 = img->image;
	}
	catch(cv_bridge::Exception& e){
		ROS_ERROR("Couldn't get image due to %s",e.what());
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "slam");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	const string path2voc = "/home/varunharis/MultiCol-SLAM/Examples/small_orb_omni_voc_9_6.yml";
	const string path2settings = "/home/varunharis/MultiCol-SLAM/Examples/Lafida/Slam_Settings_indoor1.yaml"; 
	const string path2calibrations = "/home/varunharis/MultiCol-SLAM/Examples/Lafida/MultiCamSys_Calibration.yaml";

	camSubs *cam = new camSubs;
	
	cout << endl << "MultiCol-SLAM Copyright (C) 2016 Steffen Urban" << endl << endl;

	cv::FileStorage frameSettings(path2settings, cv::FileStorage::READ);

	cv::Mat frame1, frame2, frame3;
	std::vector<cv::Mat> images(3);
	MultiColSLAM::cSystem MultiSLAM(path2voc, path2settings, path2calibrations, true);

	// Main loop
	while(ros::ok()){
		ros::spinOnce();
		try{
			frame1 = cam->frame1;
			frame2 = cam->frame2;
			frame3 = cam->frame3;

			cvtColor(frame1, frame1, CV_BGR2GRAY);
			cvtColor(frame2, frame2, CV_BGR2GRAY);
			cvtColor(frame3, frame3, CV_BGR2GRAY);

			images = {frame1, frame2, frame3};
			// Pass through the MultiCol-SLAM system
			MultiSLAM.TrackMultiColSLAM(images, double(cam->header1.stamp.sec));
		}
		catch(cv::Exception& e){
			cout << "hold on" << endl;
		}
	}

	// Stop all threads
	MultiSLAM.Shutdown();
	ros::shutdown();

	// Save camera trajectory
	MultiSLAM.SaveMKFTrajectoryLAFIDA("MKFTrajectory.txt");
	cv::destroyAllWindows();
	return 0;
}
