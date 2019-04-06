# Visualization Marker Tutorials

See Old ROS1 for User Interface instructions:

*   [Markers: Sending Basic Shapes (C++)](http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes)
*   [Markers: Points and Lines (C++)](http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Points%20and%20Lines)

To run this with your workspace built and sourced:

```sh
rviz2 -d ./config.rviz &
ros2 run visualization_marker_tutorials basic_shapes
# Look at pretty shapes! Then Ctrl+C it.
ros2 run visualization_marker_tutorials points_and_lines
# Look at the pretty colors!
```
