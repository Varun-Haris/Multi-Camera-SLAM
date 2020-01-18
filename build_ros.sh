echo "Building ROS nodes"

cd Examples/ROS/MultiCol-SLAM
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j
