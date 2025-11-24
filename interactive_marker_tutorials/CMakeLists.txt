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
target_link_libraries(basic_controls PUBLIC
  ${visualization_msgs_TARGETS}
  interactive_markers::interactive_markers
  rclcpp::rclcpp
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::static_transform_broadcaster_node
  tf2_ros::tf2_ros
)

add_executable(cube src/cube.cpp)
target_link_libraries(cube PUBLIC
  ${visualization_msgs_TARGETS}
  interactive_markers::interactive_markers
  rclcpp::rclcpp
  tf2::tf2
)

add_executable(selection src/selection.cpp)
target_link_libraries(selection PUBLIC
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  interactive_markers::interactive_markers
  rclcpp::rclcpp
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
)

add_executable(simple_marker src/simple_marker.cpp)
target_link_libraries(simple_marker PUBLIC
  ${visualization_msgs_TARGETS}
  interactive_markers::interactive_markers
  rclcpp::rclcpp
)

add_executable(menu src/menu.cpp)
target_link_libraries(menu PUBLIC
  ${visualization_msgs_TARGETS}
  interactive_markers::interactive_markers
  rclcpp::rclcpp
)


add_library(pong_library SHARED src/pong.cpp)
target_link_libraries(pong_library
  ${visualization_msgs_TARGETS}
  interactive_markers::interactive_markers
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
)
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
