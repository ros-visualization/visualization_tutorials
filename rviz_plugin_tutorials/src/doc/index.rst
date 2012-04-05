RViz Plugin Tutorials
=====================

The RViz plugin API and library API are preliminary in Fuerte. We
welcome feedback about how to make them more powerful and easier to
program with. We expect the APIs to change (possibly significantly)
between Fuerte and Groovy.

The rviz_plugin_tutorials package builds a plugin library for rviz
containing two main classes: ImuDisplay and TeleopPanel.

- :doc:`display_plugin_tutorial` is an example of an rviz::Display
  subclass allowing rviz to show data from sensor_msgs::Imu messages.

- :doc:`panel_plugin_tutorial` is an example of an rviz::Panel
  subclass which shows a simple control input for sending velocities
  to a mobile base.

