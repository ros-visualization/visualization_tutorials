## BEGIN_TUTORIAL
## This CMakeLists.txt file for librviz_tutorial builds the myviz executable.

cmake_minimum_required(VERSION 3.8)
project(librviz_tutorial)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)

## This setting causes Qt's "MOC" generation to happen automatically, which is required for
## Qt's signal/slot connections to work.
set(CMAKE_AUTOMOC ON)

## Here we specify the list of source files.
## The generated MOC files are included automatically as headers.
set(SRC_FILES
  src/myviz.cpp
  src/main.cpp
)

## Add the "myviz" executable and specify the list of source files we
## collected above in ``${SRC_FILES}``.
add_executable(myviz ${SRC_FILES})
ament_target_dependencies(myviz
  "Qt5Core"
  "Qt5Widgets"
  "rclcpp"
  "rviz_common"
  "rviz_default_plugins"
  "rviz_rendering")
target_include_directories(myviz PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS})

## END_TUTORIAL

ament_export_dependencies(
  rclcpp
  rviz_common
  rviz_default_plugins
  rviz_rendering)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

## Install
install(TARGETS myviz RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
