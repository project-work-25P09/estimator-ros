cmake_minimum_required(VERSION 3.10)
project(server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclpy REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(estimation REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rpi_hw_monitor REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install directories
install(DIRECTORY web DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME})

# Install executable scripts
install(PROGRAMS
  "scripts/start_server.py"
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME}_pkg)
ament_package()
