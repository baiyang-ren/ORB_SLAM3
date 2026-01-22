cmake_minimum_required(VERSION 3.5)
project(orb_slam3_ros2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -march=native)
endif()

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# Find other dependencies
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

# ORB_SLAM3 paths
set(ORB_SLAM3_DIR ${PROJECT_SOURCE_DIR}/../../..)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${ORB_SLAM3_DIR}
  ${ORB_SLAM3_DIR}/include
  ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# ORB_SLAM3 libraries
set(ORB_SLAM3_LIBS
  ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
  ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORB_SLAM3_DIR}/Thirdparty/g2o/lib/libg2o.so
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  -lboost_system
  -lboost_serialization
  -lcrypto
)

# Stereo Inertial node
add_executable(stereo_inertial
  src/ros2_stereo_inertial.cc
)

ament_target_dependencies(stereo_inertial
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  message_filters
  tf2
  tf2_ros
  geometry_msgs
)

target_link_libraries(stereo_inertial
  ${ORB_SLAM3_LIBS}
)

# Install targets
install(TARGETS
  stereo_inertial
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

ament_package()
