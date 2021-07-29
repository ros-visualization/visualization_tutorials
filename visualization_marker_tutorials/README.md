# visualization_marker_tutorials
Unlike other displays, the Marker Display lets you visualize data in RViz without RViz knowing anything about interpreting that data.
Instead, primitive objects are sent to the display through `visualization_msgs/msg/Marker` messages, which let you show things like shapes and lines.

This tutorial will show you how to send the four basic shapes (boxes, spheres, cylinders, and arrows).
We'll create a program that sends out a new marker every second, replacing the last one with a different shape.

## Basic Shapes

In this tutorial, you will learn how to send visualization markers and use the different shapes that are available.
Let's break down the code piece by piece:

```
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
```
We include headers for rclcpp and the `visualization_msgs/msg/Marker` message definition.

```
rclcpp::init(argc, argv);
auto node = rclcpp::Node::make_shared("basic_shapes");
auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>(
"visualization_marker", 1);
  rclcpp::Rate loop_rate(1);
```
This should look familiar.
We initialize rclcpp, then create a node and a publisher on the visualization_marker topic.

```
uint32_t shape = visualization_msgs::msg::Marker::CUBE;
```
Here we create an integer to keep track of what shape we're going to publish.
The four types we'll be using here all use the `visualization_msgs/msg/Marker` message in the same way, so we can simply switch out the shape type to demonstrate the four different shapes.

```
while (rclcpp::ok()) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "/my_frame";
  marker.header.stamp = rclcpp::Clock().now();
```
This begins the meat of the program.
First we create a `visualization_msgs/msg/Marker`, and begin filling it out.
We set the frame_id member to `/my_frame` as an example. In a running system, this should be the frame relative to which you want the marker's pose to be interpreted.

```
marker.ns = "basic_shapes";
marker.id = 0;
```
The namespace (ns) and ID are used to create a unique name for this marker.
If a marker message is received with the same namespace and ID, the new marker will replace the old one.

```
marker.type = shape;
```
This type field is what specifies the kind of marker we're sending.
The available types are enumerated in the `visualization_msgs/msg/Marker` message.
Here we set the type to our shape variable, which will change every time through the loop.

```
marker.action = visualization_msgs::msg::Marker::ADD;
```
The action field is what specifies what to do with the marker.
The options are visualization_msgs::msg::Marker::ADD, visualization_msgs::msg::Marker::DELETE, and visualization_msgs::Marker::DELETEALL.
ADD is something of a misnomer, it really means "create or modify".
DELETEALL deletes all markers in the particular RViz display, regardless of ID or namespace.

```
marker.pose.position.x = 0;
marker.pose.position.y = 0;
marker.pose.position.z = 0;
marker.pose.orientation.x = 0.0;
marker.pose.orientation.y = 0.0;
marker.pose.orientation.z = 0.0;
marker.pose.orientation.w = 1.0;
```
Here we set the pose of the marker.
The `geometry_msgs/msg/Pose` message consists of a `geometry_msgs/msg/Vector3` to specify the position and a `geometry_msgs/msg/Quaternion` to specify the orientation.
Here we set the position to the origin, and the orientation to the identity orientation (note the 1.0 for w).

```
marker.scale.x = 1.0;
marker.scale.y = 1.0;
marker.scale.z = 1.0;
```
Now we specify the scale of the marker.
For the basic shapes, a scale of 1 in all directions means 1 meter on a side.

```
marker.color.r = 0.0f;
marker.color.g = 1.0f;
marker.color.b = 0.0f;
marker.color.a = 1.0;
```
The color of the marker is specified as a `std_msgs/msg/ColorRGBA`.
Each member should be between 0 and 1.
An alpha (a) value of 0 means completely transparent (invisible), and 1 is completely opaque.

```
marker.lifetime = rclcpp::Duration::from_nanoseconds(0);
```
The lifetime field specifies how long this marker should stick around before being automatically deleted. 
A value of 0 means never to auto-delete.
If a new marker message is received before the lifetime has been reached, the lifetime will be reset to the value in the new marker message.

```
while (marker_pub->get_subscription_count() < 1) {
  if (!rclcpp::ok()) {
    return 0;
  }
  auto logger = rclcpp::get_logger("my_subscriber");
  RCLCPP_WARN(logger, "Please create a subscriber to the marker");
  loop_rate.sleep();
}
marker_pub->publish(marker);
```
We wait for the marker to have a subscriber and then we publish the marker.

```
switch (shape) {
  case visualization_msgs::msg::Marker::CUBE:
    shape = visualization_msgs::msg::Marker::SPHERE;
    break;
  case visualization_msgs::msg::Marker::SPHERE:
    shape = visualization_msgs::msg::Marker::ARROW;
    break;
  case visualization_msgs::msg::Marker::ARROW:
    shape = visualization_msgs::msg::Marker::CYLINDER;
    break;
  case visualization_msgs::msg::Marker::CYLINDER:
    shape = visualization_msgs::msg::Marker::CUBE;
    break;
}
```
This code lets us show all four shapes while just publishing the one marker message.
Based on the current shape, we set what the next shape to publish will be.

```
loop_rate.sleep();
```
Sleep and loop back to the top.

### Running the code
Naviagte into your workspace, then build the code and run it:

```
$ colcon build --packages-select visualization_marker_tutorials
$ ros2 run visualization_marker_tutorials basic_shapes
``` 
### Viewing the markers
Now that you're publishing markers, you need to run RViz to view them:

```
$ rviz2
```

If you've never used RViz before, please see the User's Guide to get you started.

The first thing to do, because we don't have any tf transforms set up, is to set the fixed frame.
Select the Fixed Frame field in RViz and enter "/my_frame".

Next, add a Marker Display.
Notice that the default topic specified, visualization_marker, is the same as the one being published.
You should now see a marker at the origin that changes shape every second.

## Points and Lines
In the Basic Shapes tutorial, you learned how to send simple shapes to RViz using visualization markers.
You can send more than just simple shapes though, and this tutorial will introduce you to the POINTS, LINE_STRIP and LINE_LIST marker types.
For a full list of types, see the Marker Display page.

### Using points, line strips, and line lists
The POINTS, LINE_STRIP and LINE_LIST markers all use the points member of the `visualization_msgs/msg/Marker` message.
The POINTS type places a point at each point added.
The LINE_STRIP type uses each point as a vertex in a connected set of lines, where point 0 is connected to point 1, 1 to 2, 2 to 3, etc.
The LINE_LIST type creates unconnected lines out of each pair of points, i.e. point 0 to 1, 2 to 3, etc.

Now let's break down the code, skipping things that were explained in the previous tutorial. 
The overall effect created is a rotating helix with lines sticking upwards from each vertex.

```
visualization_msgs::msg::Marker points, line_strip, line_list;
points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/my_frame";
points.header.stamp = line_strip.header.stamp = line_list.header.stamp = rclcpp::Clock().now();
points.ns = line_strip.ns = line_list.ns = "points_and_lines";
points.action = line_strip.action = line_list.action = visualization_msgs::msg::Marker::ADD;
points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;
```
Here we create three `visualization_msgs/msg/Marker` messages and initialize all of their shared data.
We take advantage of the fact that message members default to 0 and only set the w member of the pose.

```
points.id = 0;
line_strip.id = 1;
line_list.id = 2;
```
We assign three different IDs to the three markers.
The use of the points_and_lines namespace ensures they won't collide with other broadcasters.

```
points.type = visualization_msgs::msg::Marker::POINTS;
line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
```
Here we set the marker types to POINTS, LINE_STRIP and LINE_LIST.

```
points.scale.x = 0.2;
points.scale.y = 0
line_strip.scale.x = 0.1;
line_list.scale.x = 0.1;
```
The scale member means different things for these marker types.
The POINTS marker uses the x and y members for width and height respectively, while the LINE_STRIP and LINE_LIST markers only use the x component, which defines the line width.
Scale values are in meters.

```
// Points are green
points.color.g = 1.0f;
points.color.a = 1
// Line strip is blue
line_strip.color.b = 1.0;
line_strip.color.a = 1
// Line list is red
line_list.color.r = 1.0;
line_list.color.a = 1.0;
```
Here we set the points to green, the line strip to blue, and the line list to red.

```
for (uint32_t i = 0; i < 100; ++i) {
  float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
  float z = 5 * cos(f + i / 100.0f * 2 * M_PI);

  geometry_msgs::msg::Point p;
  p.x = (int32_t)i - 50;
  p.y = y;
  p.z = z;

  points.points.push_back(p);
  line_strip.points.push_back(p);

  // The line list needs two points for each line
  line_list.points.push_back(p);
  p.z += 1.0;
  line_list.points.push_back(p);
}
```
We use sine and cosine to generate a helix.
The POINTS and LINE_STRIP markers both require only a point for each vertex, while the LINE_LIST marker requires 2.

### Viewing the markers
To view the markers, build and run the code:

```
$ colcon build
$ ros2 run visualization_marker_tutorials points_and_lines
```

Then start RViz:
```
$ rviz2
```

Like the last tutorial, you need to set "/my_frame" as the fixed frame and add a Marker Display.
You should now see a rotating helix shape.
