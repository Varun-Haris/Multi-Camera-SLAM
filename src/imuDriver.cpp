#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <custom_msgs/IMUInfo.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

#include <math.h>
#include <vector>
#include <stdint.h>

class imuDriver{
public:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher pub;

  geometry_msgs::Vector3 gyro, acc, gyroInit;
  geometry_msgs::Quaternion quat;
  sensor_msgs::Imu msg;
  float  roll, pitch, yaw, tSample;

  imuDriver(){
    sub = nh.subscribe("/ecoprt/imu",10,&imuDriver::getIMUData, this);
    pub = nh.advertise<sensor_msgs::Imu>("/slam/imu", 10);
    gyroInit.x = 0;
    gyroInit.y = 0;
    gyroInit.z = 0;
  }

  void getIMUData(const custom_msgs::IMUInfoPtr& msg){
    gyro.x = msg->DataGyro[0];
    gyro.y = msg->DataGyro[1];
    gyro.z = msg->DataGyro[2];

    acc.x = msg->DataAcc[0];
    acc.y = msg->DataAcc[1];
    acc.z = msg->DataAcc[2];

    tSample = msg->IMUSampFreq;
    //std::cout << gyro << "\n" << acc << std::endl;
  }

  void getOrientation(){
    geometry_msgs::Vector3 unitAcc;
    // Unit vector for linear acceleration
    unitAcc.x = acc.x/sqrt(pow(acc.x,2) + pow(acc.y,2) + pow(acc.z,2));
    unitAcc.y = acc.y/sqrt(pow(acc.x,2) + pow(acc.y,2) + pow(acc.z,2));
    unitAcc.z = acc.z/sqrt(pow(acc.x,2) + pow(acc.y,2) + pow(acc.z,2));

    geometry_msgs::Vector3 angAcc;
    // Angular acceleration = derivative of angular velocity wrt time
    angAcc.x = (gyro.x - gyroInit.x)/tSample;
    angAcc.y = (gyro.y - gyroInit.y)/tSample;
    angAcc.z = (gyro.z - gyroInit.z)/tSample;

    roll = -atan2(-unitAcc.x,unitAcc.y);
    pitch = -atan2(unitAcc.z,sgn(unitAcc.y)*sqrt(pow(unitAcc.x,2)+pow(unitAcc.y,2)));
    yaw = angAcc.z*(gyroInit.z + gyro.z*tSample) + (1-angAcc.z)*atan2(acc.x,acc.y);

    gyroInit.x = gyro.x;
    gyroInit.y = gyro.y;
    gyroInit.z = gyro.z;
    //std::cout << "Roll " << roll*180/M_PI << "\n" << "Pitch " << pitch*180/M_PI << "\n" << "Yaw " << yaw*180/M_PI << std::endl;

    euler_to_quaternion(roll, pitch, yaw);
  }

  void euler_to_quaternion(float roll, float pitch, float yaw){
    float cy = cos(yaw*0.5);
    float sy = sin(yaw*0.5);
    float cp = cos(pitch*0.5);
    float sp = sin(pitch*0.5);
    float cr = cos(roll*0.5);
    float sr = sin(roll*0.5);

    quat.w = cy * cp * cr + sy * sp * sr;
    quat.x = cy * cp * sr - sy * sp * cr;
    quat.y = sy * cp * sr + cy * sp * cr;
    quat.z = sy * cp * cr - cy * sp * sr;

    publishImuData();
  }

  void publishImuData(){
    uint32_t seq_id = 0;

    if(seq_id == UINT32_MAX) seq_id = 0;
    // We set all zeros for covariance matrices since they're unknown
    msg.header.seq = seq_id;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "IMU";

    msg.orientation.x = quat.x;
    msg.orientation.y = quat.y;
    msg.orientation.z = quat.z;
    msg.orientation.w = quat.w;
    msg.orientation_covariance = {0,0,0,0,0,0,0,0,0};

    msg.angular_velocity.x = gyro.x;
    msg.angular_velocity.y = gyro.y;
    msg.angular_velocity.z = gyro.z;
    msg.angular_velocity_covariance = {0,0,0,0,0,0,0,0,0};

    msg.linear_acceleration.x = acc.x;
    msg.linear_acceleration.y = acc.x;
    msg.linear_acceleration.z = acc.x;
    msg.linear_acceleration_covariance = {0,0,0,0,0,0,0,0,0};

    pub.publish(msg);
  }

  int sgn(float v) {
    if (v < 0) return -1;
    if (v > 0) return 1;
    return 0;
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "IMUDriver");
  ros::NodeHandle n;

  imuDriver *imu = new imuDriver;
  while(ros::ok()){
    imu->getOrientation();
    ros::spinOnce();
  }
}
