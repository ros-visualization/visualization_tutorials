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
target_link_libraries(basic_shapes PUBLIC
  ${visualization_msgs_TARGETS}
  rclcpp::rclcpp
)

add_executable(points_and_lines src/points_and_lines.cpp)
target_link_libraries(points_and_lines PUBLIC
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  rclcpp::rclcpp
)


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
