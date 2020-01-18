cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#project(MultiCol-SLAM-ROS)

rosbuild_init()

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

IF(NOT DEFINED EIGEN_INCLUDE_DIR_HINTS)
  SET(EIGEN_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/../../../ThirdParty/Eigen)
  MESSAGE("Eigen include DIR ${EIGEN_INCLUDE_DIR}")
  #SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/ThirdParty/Eigen/cmake)
  #FIND_PACKAGE(Eigen3 REQUIRED)
  #MESSAGE("-- Eigen version ${EIGEN3_VERSION}: ${EIGEN3_INCLUDE_DIR}")
  #INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
  SET(MultiCol-SLAM_USE_INTERNAL_EIGEN ON)
  INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})
ENDIF()

find_package(Pangolin REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/../../../
${PROJECT_SOURCE_DIR}/../../../include
${PROJECT_SOURCE_DIR}/../../../ThirdParty
${PROJECT_SOURCE_DIR}/../../../ThirdParty/g2o
${PROJECT_SOURCE_DIR}/../../../ThirdParty/OpenGV/include
${PROJECT_SOURCE_DIR}/../../../ThirdParty/DBoW2
${Pangolin_INCLUDE_DIRS}
)

SET (G2OLIBS ${PROJECT_SOURCE_DIR}/../../../ThirdParty/g2o/lib/libg2o.so)
SET (DBOW2LIBS ${PROJECT_SOURCE_DIR}/../../../ThirdParty/DBoW2/lib/libDBoW2.so)
SET (OPENGVLIBS ${PROJECT_SOURCE_DIR}/../../../ThirdParty/OpenGV/build/lib/librandom_generators.a
                ${PROJECT_SOURCE_DIR}/../../../ThirdParty/OpenGV/build/lib/libopengv.a)

set(LIBS 
${OpenCV_LIBS}
${Pangolin_LIBRARIES}
${G2OLIBS}
${DBOW2LIBS}
${OPENGVLIBS}
${RANDOMGENLIBS}
#-lboost_system
)

# Node for multi camera
rosbuild_add_executable(Multi
src/ros_multi.cc
) 

#add_dependencies(Multi g2o DBoW2)

target_link_libraries(Multi
${LIBS}
/home/varunharis/MultiCol-SLAM/lib/libMultiCol-SLAM.so
)
