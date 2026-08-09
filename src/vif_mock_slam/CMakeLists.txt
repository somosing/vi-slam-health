cmake_minimum_required(VERSION 3.8)
project(vif_mock_slam)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vif_interfaces REQUIRED)

add_executable(mock_slam_node src/mock_slam_node.cpp)
ament_target_dependencies(mock_slam_node
  geometry_msgs nav_msgs rclcpp tf2_ros vif_interfaces)
install(TARGETS mock_slam_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
