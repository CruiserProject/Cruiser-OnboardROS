cmake_minimum_required(VERSION 2.8.3)
project(dji_sdk_read_cam)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## is used, also find other catkin packages

catkin_package()
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  #opencv2
  roscpp
  sensor_msgs
  std_msgs

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(dji_sdk_read_cam src/nv_cam.cpp)
link_directories(/usr/local/lib /usr/lib/arm-linux-gnueabihf /usr/bin)
target_link_libraries(
  dji_sdk_read_cam
  dcam
  ${catkin_LIBRARIES})

