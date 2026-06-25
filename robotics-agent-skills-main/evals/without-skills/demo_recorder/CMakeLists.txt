cmake_minimum_required(VERSION 3.8)
project(demo_recorder)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RecordingStatus.msg"
  "srv/StartRecording.srv"
  "srv/StopRecording.srv"
  DEPENDENCIES builtin_interfaces
)

ament_package()
