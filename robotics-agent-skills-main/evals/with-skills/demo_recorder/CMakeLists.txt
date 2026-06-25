cmake_minimum_required(VERSION 3.8)
project(demo_recorder)

# Default to C++17 for any compiled components
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- dependencies --------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# ---- interface generation (msg / srv) ------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RecordingStatus.msg"
  "srv/StartRecording.srv"
  "srv/StopRecording.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

# ---- install Python modules ----------------------------------------------
ament_python_install_package(${PROJECT_NAME})

# ---- install Python executables ------------------------------------------
install(PROGRAMS
  demo_recorder/demo_recorder_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---- install launch, config, resource files ------------------------------
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(FILES
  resource/demo_recorder
  DESTINATION share/ament_index/resource_index/packages
)

# ---- testing -------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_demo_recorder test/test_demo_recorder.py
    TIMEOUT 60
  )
endif()

ament_package()
