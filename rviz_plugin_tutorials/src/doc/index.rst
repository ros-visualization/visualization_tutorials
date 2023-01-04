RViz Plugin Tutorials
=====================

The rviz_plugin_tutorials package builds a plugin library for RViz
containing three main classes: ImuDisplay, TeleopPanel, and PlantFlagTool.

- :doc:`display_plugin_tutorial` is an example of an rviz_common::Display
  subclass allowing RViz to show data from sensor_msgs::msg::Imu messages.

- :doc:`panel_plugin_tutorial` is an example of an rviz_common::Panel
  subclass which shows a simple control input for sending velocities
  to a mobile base.

- :doc:`tool_plugin_tutorial` is an example of an rviz_common::Tool
  subclass which lets you plant flag markers on the Z=0 plane.
  