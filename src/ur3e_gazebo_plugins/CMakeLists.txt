cmake_minimum_required(VERSION 3.8)
project(ur3e_gazebo_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(Protobuf REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})

# --- protobuf message used by GazeboGraspFix (published on ~/grasp_events) ---
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS msgs/grasp_event.proto)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}   # for generated grasp_event.pb.h
)

# --- contact-based grasp fix plugin (JenniferBuehler) ---
add_library(gazebo_grasp_fix SHARED
  src/GazeboGraspFix.cpp
  src/GazeboGraspGripper.cpp
  src/GazeboVersionHelpers.cpp
  ${PROTO_SRCS}
)
target_link_libraries(gazebo_grasp_fix
  ${GAZEBO_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
)

# --- PID mimic-joint plugin ---
add_library(ur3e_mimic_joint_plugin SHARED src/mimic_joint_plugin.cpp)
target_link_libraries(ur3e_mimic_joint_plugin ${GAZEBO_LIBRARIES})

# --- attach-on-close grasp plugin (stable alternative to GazeboGraspFix) ---
add_library(ur3e_gripper_grasp_plugin SHARED src/gripper_grasp_plugin.cpp)
target_link_libraries(ur3e_gripper_grasp_plugin ${GAZEBO_LIBRARIES})

install(TARGETS gazebo_grasp_fix ur3e_mimic_joint_plugin ur3e_gripper_grasp_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
