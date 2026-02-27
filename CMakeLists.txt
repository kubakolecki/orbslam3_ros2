cmake_minimum_required(VERSION 3.20)
project(orbslam3)

# You should set the PYTHONPATH to your own python site-packages path
# set(ENV{PYTHONPATH} "/opt/ros/jazzy/lib/python3.8/site-packages/")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(ORB_SLAM3 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GeoreferencedStereoImage.msg"
  DEPENDENCIES geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

message("Packages found")

include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  
)

link_directories(
  include
)

add_executable(mono
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
)
ament_target_dependencies(mono rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin)

message("added executable mono")

#target_include_directories(mono PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>
#)

get_target_property(dirs mono INCLUDE_DIRECTORIES)
message("mono include dirs: ${dirs}")

add_executable(rgbd
  src/rgbd/rgbd.cpp
  src/rgbd/rgbd-slam-node.cpp
)
ament_target_dependencies(rgbd rclcpp rosidl_default_generators sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin)

message("added executable rgbd")

get_target_property(dirs rgbd INCLUDE_DIRECTORIES)
message("rgbd include dirs: ${dirs}")

add_executable(stereo
  src/stereo/stereo.cpp
  src/stereo/stereo-slam-node.cpp
)
ament_target_dependencies(stereo rclcpp rosidl_default_generators sensor_msgs geometry_msgs nav_msgs cv_bridge tf2 tf2_geometry_msgs tf2_eigen message_filters OpenCV
ORB_SLAM3 Pangolin)

set_property(TARGET stereo PROPERTY CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

message("added executable stereo")

get_target_property(dirs stereo INCLUDE_DIRECTORIES)
message("stereo include dirs: ${dirs}")


add_executable(stereo-inertial
  src/stereo-inertial/stereo-inertial.cpp
  src/stereo-inertial/stereo-inertial-node.cpp
)
ament_target_dependencies(stereo-inertial rclcpp sensor_msgs cv_bridge message_filters OpenCV ORB_SLAM3 Pangolin)

message("added executable stereo-inertial")

get_target_property(dirs stereo-inertial INCLUDE_DIRECTORIES)
message("stereo-inertial include dirs: ${dirs}")

install(TARGETS mono rgbd stereo stereo-inertial
  DESTINATION lib/${PROJECT_NAME})

rosidl_target_interfaces(stereo ${PROJECT_NAME} "rosidl_typesupport_cpp")  #export the interface for the stereo node, THIS IS VERY IMPORTANT!!!!!

# Install launch files.
#install(DIRECTORY launch config vocabulary
#  DESTINATION share/${PROJECT_NAME}/)

ament_package()

