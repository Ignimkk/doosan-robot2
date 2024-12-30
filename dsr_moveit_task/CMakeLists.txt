cmake_minimum_required(VERSION 3.8)
project(dsr_moveit_task)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)


add_executable(dsr_moveit_task src/moveit_test.cpp)
ament_target_dependencies(dsr_moveit_task
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)




install(TARGETS dsr_moveit_task
  DESTINATION lib/${PROJECT_NAME})
  
ament_package()