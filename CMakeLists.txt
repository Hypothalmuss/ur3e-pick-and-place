cmake_minimum_required(VERSION 3.8)
project(ur3e_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  msg/ArucoDetection.msg
  msg/ArucoDetectionArray.msg
  msg/YoloDetection.msg
  msg/YoloDetectionArray.msg
  msg/DetectedObject.msg
  msg/DetectedObjectArray.msg
  msg/BTStatus.msg
  msg/SafetyCommand.msg
)

set(srv_files
  srv/AssignPickGoal.srv
  srv/GetBTStatus.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
