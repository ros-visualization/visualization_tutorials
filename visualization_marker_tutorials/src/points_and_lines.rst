
Markers: Points and Lines
=========================

Overview
--------

In the :doc:`basic_shapes` tutorial, you learned how to send simple shapes to RViz using visualization markers.
You can send more than just simple shapes though, and this tutorial will introduce you to the POINTS, LINE_STRIP and LINE_LIST marker types.
For a full list of types, see the `Marker Display page <http://wiki.ros.org/rviz/DisplayTypes/Marker#Object_Types>`_.

Using points, line strips, and line lists
-----------------------------------------

The POINTS, LINE_STRIP and LINE_LIST markers all use the points member of the `visualization_msgs/msg/Marker` message.
The POINTS type places a point at each point added.
The LINE_STRIP type uses each point as a vertex in a connected set of lines, where point 0 is connected to point 1, 1 to 2, 2 to 3, etc.
The LINE_LIST type creates unconnected lines out of each pair of points, i.e. point 0 to 1, 2 to 3, etc.

The code
--------

The code for this tutorial is located in `points_and_lines.cpp <https://github.com/ros-visualization/visualization_tutorials/blob/ros2/visualization_marker_tutorials/src/points_and_lines.cpp>`_.

Now let's break down the code, skipping things that were explained in the previous tutorial. 
The overall effect created is a rotating helix with lines sticking upwards from each vertex.

.. literalinclude:: points_and_lines.cpp
    :language: c++
    :lines: 48-52

Here we create three `visualization_msgs/msg/Marker` messages and initialize all of their shared data.
By default, the marker message contains a pose with a `geometry_msgs/msg/Quaternion` that is initialized to the identity quaternion.

.. literalinclude:: points_and_lines.cpp
    :language: c++
    :lines: 54-56

We assign three different IDs to the three markers.
The use of the points_and_lines namespace ensures they won't collide with other broadcasters.

.. literalinclude:: points_and_lines.cpp
    :language: c++
    :lines: 58-60

Here we set the marker types to POINTS, LINE_STRIP and LINE_LIST.

.. literalinclude:: points_and_lines.cpp
    :language: c++
    :lines: 62-68

The scale member means different things for these marker types.
The POINTS marker uses the x and y members for width and height respectively, while the LINE_STRIP and LINE_LIST markers only use the x component, which defines the line width.
Scale values are in meters.

.. literalinclude:: points_and_lines.cpp
    :language: c++
    :lines: 70-80

Here we set the points to green, the line strip to blue, and the line list to red.

.. literalinclude:: points_and_lines.cpp
    :language: c++
    :lines: 83-99

We use sine and cosine to generate a helix.
The POINTS and LINE_STRIP markers both require only a point for each vertex, while the LINE_LIST marker requires 2.

Viewing the markers
-------------------
To view the markers, build and run the code:

.. code-block:: bash

    colcon build
    ros2 run visualization_marker_tutorials points_and_lines

Then start RViz:

.. code-block:: bash

    rviz2

Like the last tutorial, you need to set "/my_frame" as the fixed frame and add a Marker Display.
You should now see a rotating helix shape.
