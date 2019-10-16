cmake_minimum_required(VERSION 3.5)
project(interactive_marker_tutorials)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

ament_export_dependencies(
  "interactive_markers"
  "rclcpp"
  "rclcpp_components"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
  "visualization_msgs")

add_executable(basic_controls src/basic_controls.cpp)
ament_target_dependencies(basic_controls
  "interactive_markers"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
  "visualization_msgs")

add_executable(cube src/cube.cpp)
ament_target_dependencies(cube
  "interactive_markers"
  "rclcpp"
  "tf2"
  "visualization_msgs")

add_executable(selection src/selection.cpp)
ament_target_dependencies(selection
  "geometry_msgs"
  "interactive_markers"
  "rclcpp"
  "std_msgs"
  "tf2"
  "tf2_geometry_msgs"
  "visualization_msgs")

add_executable(simple_marker src/simple_marker.cpp)
ament_target_dependencies(simple_marker
  "interactive_markers"
  "rclcpp"
  "visualization_msgs")

add_executable(menu src/menu.cpp)
ament_target_dependencies(menu
  "interactive_markers"
  "rclcpp"
  "visualization_msgs")

add_library(pong_library SHARED src/pong.cpp)
ament_target_dependencies(pong_library
  "interactive_markers"
  "rclcpp"
  "rclcpp_components"
  "tf2"
  "tf2_geometry_msgs"
  "visualization_msgs")
# Register pong plugin and build an executable
rclcpp_components_register_node(pong_library
  PLUGIN "interactive_marker_tutorials::PongGameNode"
  EXECUTABLE pong)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(PROGRAMS
  scripts/basic_controls.py
  scripts/cube.py
  scripts/menu.py
  scripts/simple_marker.py
  DESTINATION bin
)

install(TARGETS
  pong_library
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  basic_controls
  cube
  selection
  simple_marker
  menu
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
