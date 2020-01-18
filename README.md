# Multi-Camera-SLAM
ROS(Kinetic) implementation of Urbstate MultiCol-SLAM

This is a stable ROS implementation of the MultiCol-SLAM algorithm (original author: Stephan Urban). Please refer to the 
"https://github.com/urbste/MultiCol-SLAM" for initial implementation

To build ROS nodes, run the build_ros.sh script after running the build.sh script. Once you run the build.sh, please check the 
compilation of the MultiCol-SLAM library in the /lib folder.

Run the SLAM algorithm using the ROS toolchain:-
1) Initiate the ros master (roscore or an existing roslaunch file)
2) Run the camDriver node (rosrun multi_camera_slam camDriver)
3) Run the SLAM node (rosrun MultiCol-SLAM Multi)

To change the calibration or setting files, go to Examples/ROS/MultiCol-SLAM/src/ros_multi.cpp, change the file paths appropriately
