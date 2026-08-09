cmake_minimum_required(VERSION 3.8)
project(vif_relocalization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vif_interfaces REQUIRED)

add_executable(relocalization_manager_node src/relocalization_manager_node.cpp)
ament_target_dependencies(relocalization_manager_node rclcpp vif_interfaces)
install(TARGETS relocalization_manager_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
