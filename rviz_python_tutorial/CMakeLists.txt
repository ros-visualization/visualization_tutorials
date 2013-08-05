cmake_minimum_required(VERSION 2.8.3)
project(rviz_python_tutorial)
find_package(catkin REQUIRED COMPONENTS rviz)
catkin_package()

## Install rules

install(FILES
  config.myviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS myviz.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
