cmake_minimum_required(VERSION 3.0.2)
project(map_create)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
        sensor_msgs
        cv_bridge
)

find_package(OpenCV REQUIRED)
include_directories(OpenCV_INCLUDE_DIRS)

find_package(PCL REQUIRED)
include_directories(PCL_INCLUDE_DIRS)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(create_map src/create_map_node.cpp)
add_executable(Rgbd2PointCloud src/Rgbd2PointCloud_node.cpp)

target_link_libraries(create_map ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(Rgbd2PointCloud ${catkin_LIBRARIES} ${PCL_LIBRARIES}  ${OpenCV_LIBS})