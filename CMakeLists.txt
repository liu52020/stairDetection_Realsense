cmake_minimum_required(VERSION 3.19)
project(stairDetection_realsense)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

find_package(OpenCV REQUIRED)
FIND_PACKAGE(realsense2 REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} /usr/include/eigen3)
include_directories( ${realsense2_INCLUDE_DIRS})
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITION})

include_directories(${OpenCV_INCLUDE_DIRS})  # 添加opencv头文件

#显示OpenCV_INCLUDE_DIRS的值
message(${OpenCV_INCLUDE_DIRS})


add_executable(stairs main.cpp Scene.cpp Scene.h Plane.cpp Plane.h)
target_link_libraries(stairs ${PCL_LIBRARIES})
target_link_libraries(stairs ${OpenCV_LIBS})
target_link_libraries(stairs ${realsense2_LIBRARY})