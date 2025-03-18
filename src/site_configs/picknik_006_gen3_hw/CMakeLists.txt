cmake_minimum_required(VERSION 3.16.3)
project(picknik_006_gen3_hw)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
  config
  description
  launch
  objectives
  rviz
  waypoints
  DESTINATION
  share/${PROJECT_NAME}
)

find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(calibration_node src/calibration_node.cpp)
ament_target_dependencies(calibration_node
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  tf2_geometry_msgs
)

install(TARGETS calibration_node RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
