cmake_minimum_required(VERSION 3.8)
project(vif_sensor_frontend)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vif_interfaces REQUIRED)

add_executable(sensor_guard_node src/sensor_guard_node.cpp)
target_include_directories(sensor_guard_node PRIVATE include)
ament_target_dependencies(sensor_guard_node
  message_filters rclcpp sensor_msgs vif_interfaces
)

install(TARGETS sensor_guard_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
