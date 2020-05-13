cmake_minimum_required(VERSION 3.0.2)
project(interactive_marker_tutorials)

find_package(catkin REQUIRED COMPONENTS interactive_markers roscpp visualization_msgs tf)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS interactive_markers roscpp visualization_msgs tf
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(simple_marker src/simple_marker.cpp)
target_link_libraries(simple_marker
   ${catkin_LIBRARIES}
)

add_executable(basic_controls src/basic_controls.cpp)
target_link_libraries(basic_controls
   ${catkin_LIBRARIES}
)

add_executable(selection src/selection.cpp)
target_link_libraries(selection
   ${catkin_LIBRARIES}
)

add_executable(pong src/pong.cpp)
target_link_libraries(pong
   ${catkin_LIBRARIES}
)

add_executable(cube src/cube.cpp)
target_link_libraries(cube
   ${catkin_LIBRARIES}
)

add_executable(menu src/menu.cpp)
target_link_libraries(menu
   ${catkin_LIBRARIES}
)

add_executable(point_cloud src/point_cloud.cpp)
target_link_libraries(point_cloud
   ${catkin_LIBRARIES}
)
#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/basic_controls.py
  scripts/cube.py
  scripts/menu.py
  scripts/simple_marker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS
  simple_marker
  basic_controls
  selection
  pong
  cube
  menu
  point_cloud
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
