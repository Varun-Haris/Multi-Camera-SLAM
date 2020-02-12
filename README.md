# Multi-Camera-SLAM

ROS-Kinetic implementation of Urbstate MultiCol-SLAM

This is a stable ROS implementation of the MultiCol-SLAM algorithm (original author: Stephan Urban). Please refer to the 
"https://github.com/urbste/MultiCol-SLAM" for initial implementation. This enables the user to run MultiCol-SLAM algorithm using live camera feed and perform real-time SLAM. 

Please note that the calibration files are very crucial for efficient tracking. Please refer to https://github.com/urbste/MultiCol-SLAM/issues/9, for concerns on multiple camera calibration setup.

# Third Party dependencies needed

1) Pangolin
For visualization. Get the instructions here: https://github.com/stevenlovegrove/Pangolin. This isn't present in the ./ThirdParty folder, needs to be installed off-the-shelf.
   
2) Eigen3
Required by g2o Version 3.2.9 is included in the ./ThirdParty folder. Other version can be found at http://eigen.tuxfamily.org/index.php?title=Main_Page.
   
3) OpenGV
OpenGV can be found at: https://github.com/laurentkneip/opengv. It is also included in the ./ThirdParty folder. OpenGV is used for re-localization (GP3P) and relative orientation estimation during initialization (Stewenius).

4) DBoW2 and g2o
As ORB-SLAM2 modified versions of DBoW2 and g2o are used for place recognition and optimization respectively. Both are included in the ./ThirdParty folder. The original versions can be found here: https://github.com/dorian3d/DBoW2, https://github.com/RainerKuemmerle/g2o.

5) OpenCV 3
ROS-Kinetic has its own version of OpenCV-3.3.1-Dev. This version works fine with the MultiCol-SLAM, and I would recommend not to mend with the OpenCV version. In any case OpenCV can be found and installed from: https://github.com/opencv/opencv.

# Building and running ROS nodes

Build the package:-
1) Add the MultiCol-SLAM package path to the ROS_PACKAGE_PATH envirnoment variable as shown below 

   export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/*path_to_directory*/Examples/ROS/MultiCol-SLAM
   
2) Compile the MultiCol-SLAM source codes into a condensed library with *bash build.sh* command.
3) Compile the ROS executable with the condensed libraries with *bash build_ros.sh* command.

Run the SLAM algorithm using the ROS toolchain:-
1) Initiate the ros master (roscore or an existing roslaunch file)
2) Run the camDriver node (rosrun *your_package_name* camDriver)
3) Run the SLAM node (rosrun *your_package_name* Multi)

To change the calibration or setting files, go to Examples/ROS/MultiCol-SLAM/src/ros_multi.cpp, change the file paths appropriately
