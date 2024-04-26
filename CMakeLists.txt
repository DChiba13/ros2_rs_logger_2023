cmake_minimum_required(VERSION 3.5)
project(ros2_rs_logger_2023)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

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
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(tf2 REQUIRED)
find_package(ros_robocore_interfaces REQUIRED)

include_directories(
  include
  lib
  src
)
add_library(${PROJECT_NAME} SHARED
  src/logger_lib.cpp
  src/logger.cpp
  src/defs.cpp
  src/sample_logger_save.cpp
)

ament_target_dependencies(${PROJECT_NAME} 
  rclcpp
  rclcpp_components
  OpenCV
  Boost
  tf2
  cv_bridge
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  ros_robocore_interfaces
)
rclcpp_components_register_nodes(${PROJECT_NAME} "project_ryusei::ROS2Logger")
rclcpp_components_register_nodes(${PROJECT_NAME} "project_ryusei::ROS2LoggerSaveSignal")

install(TARGETS ${PROJECT_NAME} DESTINATION lib)

# yaml
install(DIRECTORY cfg DESTINATION share/${PROJECT_NAME}/)

# launch
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
