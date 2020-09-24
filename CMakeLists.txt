cmake_minimum_required(VERSION 2.8.3)
project(homography_vision)


set(OpenCV_DIR ~/opencv/opencv-3.4.0/build)
set (CMAKE_CXX_STANDARD 11)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  message_generation
  OpenCV REQUIRED
  cv_bridge
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES homography_vision
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}  
)

link_directories()

add_executable(homography_vision src/main.cpp)
target_link_libraries(homography_vision ${catkin_LIBRARIES})



