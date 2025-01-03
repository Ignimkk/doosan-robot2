cmake_minimum_required(VERSION 3.5)
project(dsr_sensor)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(tf2_ros REQUIRED)
find_package(dsr_msgs2 REQUIRED)

# Include directories
include_directories(include ${OpenCV_INCLUDE_DIRS})

# Install Python scripts
install(
  PROGRAMS scripts/arucomarker_detect.py scripts/trans_link6_to_cam.py scripts/trans_cam_to_base.py scripts/camera_to_base.py scripts/test_marder.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install other resources
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Export package dependencies
ament_export_dependencies(
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  tf2_ros
)

ament_package()
