cmake_minimum_required(VERSION 2.8.3)
project(stereo_vision_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  camera_info_manager_py
  sensor_msgs
  message_generation
  cv_bridge
  image_proc
  std_msgs
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   ChangeModality.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
