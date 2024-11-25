cmake_minimum_required(VERSION 3.5)
project(video_io_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include/video_io)


# custom burst message 
set(msg_files
  "msg/BurstRecordCommand.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES 
  std_msgs
)
ament_export_dependencies(rosidl_default_runtime)


#add_library(color_encoding SHARED
#  src/color_encoding.cpp )
#target_compile_definitions(color_encoding
#  PUBLIC "VIDEO_PLAYER_BUILDING_DLL")
#
#
## video player
#add_executable(video_player_cpp src/video_player.cpp)
#target_include_directories(video_player_cpp PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)
#ament_target_dependencies(
#  video_player_cpp
#  rclcpp
#  OpenCV
#  sensor_msgs
#)
#target_link_libraries(video_player_cpp
#  color_encoding
#)
#
## video saver
#add_executable(video_saver_cpp src/video_saver.cpp)
#target_include_directories(video_saver_cpp PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)
#target_link_libraries(video_saver_cpp
#  color_encoding
#)
#ament_target_dependencies(
#  video_saver_cpp
#  rclcpp
#  OpenCV
#  sensor_msgs
#)
#
## burst video saver
#add_executable(burst_video_saver_cpp src/burst_video_saver.cpp)
#target_include_directories(burst_video_saver_cpp PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)
#rosidl_target_interfaces(burst_video_saver_cpp ${PROJECT_NAME} "rosidl_typesupport_cpp")
#target_link_libraries(burst_video_saver_cpp
#  color_encoding
#)
#ament_target_dependencies(
#  burst_video_saver_cpp
#  rclcpp
#  OpenCV
#  sensor_msgs
#)
#
## burst publisher, for testing
#add_executable(simple_burst_publisher_cpp src/simple_burst_publisher.cpp)
#target_include_directories(simple_burst_publisher_cpp PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)
#rosidl_target_interfaces(simple_burst_publisher_cpp ${PROJECT_NAME} "rosidl_typesupport_cpp")
#ament_target_dependencies(
#  simple_burst_publisher_cpp
#  rclcpp
#)
#
#install(TARGETS
#  color_encoding
#  ARCHIVE DESTINATION lib
#)
#
#install(TARGETS
#  video_player_cpp
#  video_saver_cpp
#  burst_video_saver_cpp
#  simple_burst_publisher_cpp
#  DESTINATION lib/${PROJECT_NAME})
#
#install(DIRECTORY
#  launch
#  config
#  DESTINATION share/${PROJECT_NAME}/
#)

ament_package()
