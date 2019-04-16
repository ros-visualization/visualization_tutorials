cmake_minimum_required(VERSION 3.5)
project(visualization_marker_tutorials)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

function(custom_executable target)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target}
    "rclcpp"
    "visualization_msgs")
  install(TARGETS ${target}
  DESTINATION lib/${PROJECT_NAME})
endfunction()

custom_executable(basic_shapes)
custom_executable(points_and_lines)

ament_package()
