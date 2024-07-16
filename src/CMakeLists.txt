cmake_minimum_required(VERSION 2.8.3)
project(system)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")
find_package(Boost REQUIRED COMPONENTS timer)

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        tf
        roscpp
        rospy
        cv_bridge
        pcl_conversions   
        message_generation
        visualization_msgs
        rosbag
)
# include directories
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
        DEPENDS PCL 

        CATKIN_DEPENDS
        message_runtime
        message_generation
        visualization_msgs
)

# link directories
link_directories(
        include
        ${PCL_LIBRARY_DIRS}
        ${OpenCV_LIBRARY_DIRS}
)


add_executable(${PROJECT_NAME}_task task.cpp)
target_link_libraries(${PROJECT_NAME}_task ${catkin_LIBRARIES})