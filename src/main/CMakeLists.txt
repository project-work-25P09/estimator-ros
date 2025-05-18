cmake_minimum_required(VERSION 3.10)
project(optical_sensor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclpy REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake REQUIRED)

install(PROGRAMS
  "scripts/start_optical.py"
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
