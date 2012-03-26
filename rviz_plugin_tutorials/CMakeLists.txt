cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

# This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

# I prefer the Qt signals and slots to avoid defining "emit", "slots",
# etc because they can conflict with boost signals.
add_definitions(-DQT_NO_KEYWORDS)

# Here we specify which header files need to be run through "moc",
# Qt's meta-object compiler.
qt4_wrap_cpp(MOC_FILES
  src/drive_widget.h
  src/teleop_panel.h
)

# Here we specify the list of source files, including the output of
# the previous command which is stored in ${MOC_FILES}.
set(SOURCE_FILES
  src/drive_widget.cpp
  src/teleop_panel.cpp 
  src/imu_display.cpp
  src/imu_visual.cpp
  ${MOC_FILES}
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# An rviz plugin is just a shared library, so here we declare the
# library and specify the list of source files we collected above.
rosbuild_add_library(${PROJECT_NAME} ${SOURCE_FILES})

# Link the library
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})
