cmake_minimum_required(VERSION 3.8)
project(visualization_marker_tutorials)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

ament_export_dependencies(
  rclcpp
  geometry_msgs
  visualization_msgs)

add_executable(basic_shapes src/basic_shapes.cpp)
ament_target_dependencies(basic_shapes
  "rclcpp"
  "visualization_msgs")

add_executable(points_and_lines src/points_and_lines.cpp)
ament_target_dependencies(points_and_lines
  "rclcpp"
  "geometry_msgs"
  "visualization_msgs")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  basic_shapes
  points_and_lines
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
