cmake_minimum_required(VERSION 3.10)
project(main)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclpy REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(estimation REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/StartRecroding.srv"
  "srv/EndRecording.srv"
  # "srv/StartPlayback.srv"
  DEPENDENCIES std_msgs
)

install(PROGRAMS
  "scripts/recording_service_node.py"
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
