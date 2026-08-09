cmake_minimum_required(VERSION 3.8)
project(vif_health_monitor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vif_interfaces REQUIRED)

add_library(health_core src/health_core.cpp)
target_include_directories(health_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(health_monitor_node src/health_monitor_node.cpp)
target_link_libraries(health_monitor_node health_core)
ament_target_dependencies(health_monitor_node rclcpp vif_interfaces)

install(TARGETS health_core health_monitor_node
  EXPORT export_health_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_health_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_health_core test/test_health_core.cpp)
  target_link_libraries(test_health_core health_core)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
