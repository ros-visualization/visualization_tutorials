## BEGIN_TUTORIAL
## This CMakeLists.txt file for rviz_plugin_tutorials builds both the
## TeleopPanel tutorial and the ImuDisplay tutorial.

cmake_minimum_required(VERSION 3.8)
project(rviz_plugin_tutorials)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(sensor_msgs REQUIRED)

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## Here we specify the list of source files.
## The generated MOC files are included automatically as headers.
set(SRC_FILES
  src/drive_widget.cpp
  src/imu_display.cpp
  src/imu_visual.cpp
  src/plant_flag_tool.cpp
  src/teleop_panel.cpp
)

## An rviz plugin is just a shared library, so here we declare the
## library to be called ``${PROJECT_NAME}`` (which is
## "rviz_plugin_tutorials", or whatever your version of this project
## is called) and specify the list of source files we collected above
## in ``${SRC_FILES}``.
add_library(${PROJECT_NAME} SHARED ${SRC_FILES})
ament_target_dependencies(${PROJECT_NAME}
  "geometry_msgs"
  "Qt5Core"
  "Qt5Widgets"
  "rclcpp"
  "rviz_common"
  "rviz_rendering"
  "sensor_msgs")

## END_TUTORIAL

ament_export_dependencies(
  geometry_msgs
  Qt5Core
  Qt5Widgets
  rclcpp
  rviz_common
  rviz_rendering
  sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

## Install rules

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  plugin_description.xml
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY media/
  DESTINATION lib/${PROJECT_NAME}/media)

install(DIRECTORY icons/
  DESTINATION lib/${PROJECT_NAME}/icons)

install(PROGRAMS scripts/send_test_msgs.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
