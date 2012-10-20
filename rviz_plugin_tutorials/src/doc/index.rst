RViz Plugin Tutorials
=====================

The rviz_plugin_tutorials package builds a plugin library for rviz
containing two main classes: ImuDisplay and TeleopPanel.

- :doc:`display_plugin_tutorial` is an example of an rviz::Display
  subclass allowing rviz to show data from sensor_msgs::Imu messages.

- :doc:`panel_plugin_tutorial` is an example of an rviz::Panel
  subclass which shows a simple control input for sending velocities
  to a mobile base.

- :doc:`tool_plugin_tutorial` is an example of an rviz::Tool
  subclass which lets you plant flag markers on the Z=0 plane.
