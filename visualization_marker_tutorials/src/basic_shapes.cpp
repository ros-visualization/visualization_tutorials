// Copyright (c) 2010, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <memory>

#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("basic_shapes");
  auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>(
    "visualization_marker", 1);
  rclcpp::Rate loop_rate(1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::msg::Marker::CUBE;

  while (rclcpp::ok()) {
    visualization_msgs::msg::Marker marker;
    // Set the frame ID and timestamp. See the TF tutorials for information on these.
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = rclcpp::Clock().now();

    // Set the namespace and id for this marker. This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type
    // Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action
    // Options are ADD, DELETE, and DELETEALL
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set the pose of the marker
    // This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    // Set the lifetime of the marker -- 0 indicates forever
    marker.lifetime = rclcpp::Duration::from_nanoseconds(0);

    // Publish the marker
    marker_pub->publish(marker);

    // Cycle between different shapes
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

    loop_rate.sleep();
  }
}
