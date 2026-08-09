cmake_minimum_required(VERSION 3.8)
project(vif_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FaultEvent.msg"
  "msg/LocalizationHealth.msg"
  "msg/RelocalizationEvent.msg"
  "msg/SensorHealth.msg"
  "msg/TrackingStatus.msg"
  "srv/ManageSlam.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
