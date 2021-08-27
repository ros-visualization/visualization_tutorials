Markers: Basic Shapes
=====================

Overview
--------

In this tutorial, you will learn how to send the four basic shapes (boxes, spheres, cylinders, and arrows) to RViz.
We'll create a program that sends out a new marker every second, replacing the last one with a different shape.

The code
--------

The code for this tutorial is located in `basic_shapes.cpp <https://github.com/ros-visualization/visualization_tutorials/blob/ros2/visualization_marker_tutorials/src/basic_shapes.cpp>`_.

Let's break down the code piece by piece:

.. literalinclude:: basic_shapes.cpp
    :language: c++
    :lines: 32-34

We include headers for rclcpp and the `visualization_msgs/msg/Marker` message definition.

.. literalinclude:: basic_shapes.cpp
    :language: c++
    :lines: 38-42

This should look familiar.
We initialize rclcpp, then create a node and a publisher on the visualization_marker topic.

.. literalinclude:: basic_shapes.cpp
    :language: c++
    :lines: 45

Here we create an integer to keep track of what shape we're going to publish.
The four types we'll be using here all use the `visualization_msgs/msg/Marker` message in the same way, so we can simply switch out the shape type to demonstrate the four different shapes.

.. literalinclude:: basic_shapes.cpp
    :language: c++
    :lines: 47-51

This begins the meat of the program.
First we create a `visualization_msgs/msg/Marker`, and begin filling it out.
We set the frame_id member to `/my_frame` as an example. In a running system, this should be the frame relative to which you want the marker's pose to be interpreted.

.. literalinclude:: basic_shapes.cpp
    :language: c++
    :lines: 55-56

The namespace (ns) and ID are used to create a unique name for this marker.
If a marker message is received with the same namespace and ID, the new marker will replace the old one.

.. literalinclude:: basic_shapes.cpp
    :language: c++
    :lines: 60

This type field is what specifies the kind of marker we're sending.
The available types are enumerated in the `visualization_msgs/msg/Marker` message.
Here we set the type to our shape variable, which will change every time through the loop.

.. literalinclude:: basic_shapes.cpp
    :language: c++
    :lines: 64

The action field is what specifies what to do with the marker.
The options are visualization_msgs::msg::Marker::ADD, visualization_msgs::msg::Marker::DELETE, and visualization_msgs::Marker::DELETEALL.
ADD is something of a misnomer, it really means "create or modify".
DELETEALL deletes all markers in the particular RViz display, regardless of ID or namespace.

.. literalinclude:: basic_shapes.cpp
    :language: c++
    :lines: 68-74

Here we set the pose of the marker.
The `geometry_msgs/msg/Pose` message consists of a `geometry_msgs/msg/Vector3` to specify the position and a `geometry_msgs/msg/Quaternion` to specify the orientation.
Here we set the position to the origin, and the orientation to the identity orientation (note the 1.0 for w).

.. literalinclude:: basic_shapes.cpp
    :language: c++
    :lines: 77-79

Now we specify the scale of the marker.
For the basic shapes, a scale of 1 in all directions means 1 meter on a side.

.. literalinclude:: basic_shapes.cpp
    :language: c++
    :lines: 82-85

The color of the marker is specified as a `std_msgs/msg/ColorRGBA`.
Each member should be between 0 and 1.
An alpha (a) value of 0 means completely transparent (invisible), and 1 is completely opaque.

.. literalinclude:: basic_shapes.cpp
    :language: c++
    :lines: 88

The lifetime field specifies how long this marker should stick around before being automatically deleted. 
A value of 0 means never to auto-delete.
If a new marker message is received before the lifetime has been reached, the lifetime will be reset to the value in the new marker message.

.. literalinclude:: basic_shapes.cpp
    :language: c++
    :lines: 91

We publish the marker message.

.. literalinclude:: basic_shapes.cpp
    :language: c++
    :lines: 94-107

This code lets us show all four shapes while just publishing the one marker message.
Based on the current shape, we set what the next shape to publish will be.

.. literalinclude:: basic_shapes.cpp
    :language: c++
    :lines: 109

Sleep and loop back to the top.

Running the code
----------------

Navigate into your workspace, then build the code and run it:

.. code-block:: bash

    colcon build --packages-select visualization_marker_tutorials
    ros2 run visualization_marker_tutorials basic_shapes

Viewing the markers
-------------------

Now that you're publishing markers, you need to run RViz to view them:

.. code-block:: bash

    rviz2

If you've never used RViz before, please see the `ROS 1 RViz User Guide <http://wiki.ros.org/rviz/UserGuide>`_ to get you started.
A user guide for ROS 2 is currently in progress.

..
   TODO(rebecca-butler): link to ROS 2 user guide when available

The first thing to do, because we don't have any tf transforms set up, is to set the fixed frame.
Select the Fixed Frame field in RViz and enter "/my_frame".

Next, add a Marker Display.
Notice that the default topic specified, visualization_marker, is the same as the one being published.
You should now see a marker at the origin that changes shape every second.
