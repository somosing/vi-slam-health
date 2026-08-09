cmake_minimum_required(VERSION 3.8)
project(vif_orbslam3)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(vif_interfaces REQUIRED)

add_library(vif_orbslam3_core src/adapter_core.cpp)
target_include_directories(vif_orbslam3_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(vif_orbslam3_core vif_interfaces)

set(ORB_SLAM3_ROOT "$ENV{ORB_SLAM3_ROOT}" CACHE PATH
  "Path to the pinned ORB-SLAM3 source/build tree")
option(VIF_BUILD_ORB_SLAM3_ADAPTER
  "Build the executable that links against the external ORB-SLAM3 tree" OFF)
option(VIF_ORB_SLAM3_HAS_DIAGNOSTICS
  "Compile against the included ORB-SLAM3 diagnostics patch" ON)
option(VIF_ALLOW_UNVERIFIED_ORB_SLAM3_REVISION
  "Allow an ORB-SLAM3 Git revision other than the pinned commit" OFF)

if(VIF_BUILD_ORB_SLAM3_ADAPTER)
  if(NOT ORB_SLAM3_ROOT)
    message(FATAL_ERROR
      "Set ORB_SLAM3_ROOT or -DORB_SLAM3_ROOT=/absolute/path/to/ORB_SLAM3")
  endif()

  set(VIF_ORB_EXPECTED_COMMIT "4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4")
  foreach(required_path
      include/System.h
      include/ImuTypes.h
      lib/libORB_SLAM3.so
      Thirdparty/DBoW2/lib/libDBoW2.so
      Thirdparty/g2o/lib/libg2o.so)
    if(NOT EXISTS "${ORB_SLAM3_ROOT}/${required_path}")
      message(FATAL_ERROR "Missing ORB-SLAM3 artifact: ${ORB_SLAM3_ROOT}/${required_path}")
    endif()
  endforeach()

  find_package(Git QUIET)
  if(Git_FOUND AND EXISTS "${ORB_SLAM3_ROOT}/.git")
    execute_process(
      COMMAND "${GIT_EXECUTABLE}" -C "${ORB_SLAM3_ROOT}" rev-parse HEAD
      OUTPUT_VARIABLE VIF_ORB_ACTUAL_COMMIT
      OUTPUT_STRIP_TRAILING_WHITESPACE
      RESULT_VARIABLE VIF_ORB_GIT_RESULT)
    if(NOT VIF_ORB_GIT_RESULT EQUAL 0)
      message(FATAL_ERROR "Could not determine the ORB-SLAM3 revision")
    endif()
    if(NOT VIF_ORB_ACTUAL_COMMIT STREQUAL VIF_ORB_EXPECTED_COMMIT AND
      NOT VIF_ALLOW_UNVERIFIED_ORB_SLAM3_REVISION)
      message(FATAL_ERROR
        "ORB-SLAM3 revision ${VIF_ORB_ACTUAL_COMMIT} is not pinned revision "
        "${VIF_ORB_EXPECTED_COMMIT}. Use VIF_ALLOW_UNVERIFIED_ORB_SLAM3_REVISION=ON "
        "only after reviewing the adapter and patch.")
    endif()
  else()
    message(WARNING "ORB-SLAM3 revision cannot be verified because .git is unavailable")
  endif()

  find_package(Boost REQUIRED COMPONENTS serialization)
  find_package(cv_bridge REQUIRED)
  find_package(Eigen3 REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(message_filters REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(OpenCV REQUIRED)
  find_package(OpenSSL REQUIRED)
  find_package(Pangolin REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)

  add_executable(orbslam3_node src/orbslam3_node.cpp)
  target_include_directories(orbslam3_node PRIVATE
    "${ORB_SLAM3_ROOT}"
    "${ORB_SLAM3_ROOT}/include"
    "${ORB_SLAM3_ROOT}/include/CameraModels"
    "${ORB_SLAM3_ROOT}/Thirdparty/Sophus"
    ${OpenCV_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS})
  ament_target_dependencies(orbslam3_node
    cv_bridge geometry_msgs message_filters nav_msgs rclcpp sensor_msgs tf2_ros
    vif_interfaces)
  target_link_libraries(orbslam3_node
    vif_orbslam3_core
    Eigen3::Eigen
    "${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so"
    "${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib/libDBoW2.so"
    "${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib/libg2o.so"
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    Boost::serialization
    OpenSSL::Crypto)
  if(VIF_ORB_SLAM3_HAS_DIAGNOSTICS)
    target_compile_definitions(orbslam3_node PRIVATE VIF_ORB_SLAM3_HAS_DIAGNOSTICS=1)
  endif()
  set_target_properties(orbslam3_node PROPERTIES
    BUILD_RPATH
      "${ORB_SLAM3_ROOT}/lib;${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib;${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib"
    INSTALL_RPATH_USE_LINK_PATH TRUE)
  install(TARGETS orbslam3_node DESTINATION lib/${PROJECT_NAME})
endif()

install(TARGETS vif_orbslam3_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_adapter_core test/test_adapter_core.cpp)
  target_link_libraries(test_adapter_core vif_orbslam3_core)
  ament_target_dependencies(test_adapter_core vif_interfaces)
endif()

ament_export_include_directories(include)
ament_export_libraries(vif_orbslam3_core)
ament_export_dependencies(vif_interfaces)
ament_package()
