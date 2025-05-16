cmake_minimum_required(VERSION 3.8)
project(arcsai_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)


# Install launch files
install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}/
)

# Testing dependencies (must be before ament_package)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)  # Ensure this is before ament_package
  ament_lint_auto_find_test_dependencies()
endif()

# Finalize the package
ament_package()
