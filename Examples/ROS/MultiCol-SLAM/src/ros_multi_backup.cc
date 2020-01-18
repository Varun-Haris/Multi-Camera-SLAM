/**
* This file is the ROS implementation of MultiCol SLAM2
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>

#include "../../../include/cSystem.h"

#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(MultiColSLAM::cSystem* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    MultiColSLAM::cSystem* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Multi");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun MultiCol Multi path_to_vocabulary path_to_settings path_to_calibration [1|0](save map?)" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    MultiColSLAM::cSystem SLAM(argv[1],argv[2],argv[3],true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth_registered/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveMKFTrajectoryLAFIDA("KeyFrameTrajectory.txt");
    //SLAM.SaveMap("map.bin");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    cv::Mat pose = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

   if (pose.empty())
       return;

       /* global left handed coordinate system */
    	static cv::Mat pose_prev = cv::Mat::eye(4,4, CV_32F);
    	static cv::Mat world_lh = cv::Mat::eye(4,4, CV_32F);
    	// matrix to flip signs of sinus in rotation matrix, not sure why we need to do that
    	static const cv::Mat flipSign = (cv::Mat_<float>(4,4) <<   1,-1,-1, 1,
    			-1, 1,-1, 1,
    			-1,-1, 1, 1,
    			1, 1, 1, 1);

    	//prev_pose * T = pose
    	cv::Mat translation =  (pose * pose_prev.inv()).mul(flipSign);
    	world_lh = world_lh * translation;
    	pose_prev = pose.clone();

    	tf::Matrix3x3 tf3d;
    	tf3d.setValue(pose.at<float>(0,0), pose.at<float>(0,1), pose.at<float>(0,2),
    			pose.at<float>(1,0), pose.at<float>(1,1), pose.at<float>(1,2),
    			pose.at<float>(2,0), pose.at<float>(2,1), pose.at<float>(2,2));

    	tf::Vector3 cameraTranslation_rh( world_lh.at<float>(0,3),world_lh.at<float>(1,3), - world_lh.at<float>(2,3) );

    	//rotate 270deg about x and 270deg about x to get ENU: x forward, y left, z up
    	const tf::Matrix3x3 rotation270degXZ(   0, 1, 0,
    			0, 0, 1,
    			1, 0, 0);

    	static tf::TransformBroadcaster br;

    	tf::Matrix3x3 globalRotation_rh = tf3d;
    	tf::Vector3 globalTranslation_rh = cameraTranslation_rh * rotation270degXZ;

    	tf::Quaternion tfqt;
    	globalRotation_rh.getRotation(tfqt);

    	double aux = tfqt[0];
    	tfqt[0]=-tfqt[2];
    	tfqt[2]=tfqt[1];
    	tfqt[1]=aux;

    	tf::Transform transform;
    	transform.setOrigin(globalTranslation_rh);
    	transform.setRotation(tfqt);

    	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "camera_pose"));
}
