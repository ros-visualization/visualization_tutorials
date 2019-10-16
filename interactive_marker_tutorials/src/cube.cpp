// Copyright (c) 2011, Willow Garage, Inc.
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
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Vector3.h"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace interactive_marker_tutorials
{

class CubeNode : public rclcpp::Node
{
public:
  explicit CubeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~CubeNode() = default;

private:
  void processFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);

  visualization_msgs::msg::InteractiveMarkerControl & makeBoxControl(
    visualization_msgs::msg::InteractiveMarker & msg);

  void makeCube();

  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
  std::vector<tf2::Vector3> positions_;
};  // class CubeNode

CubeNode::CubeNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("cube", options)
{
  server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
    "cube",
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_topics_interface(),
    get_node_services_interface());
  makeCube();
  server_->applyChanges();
  RCLCPP_INFO(get_logger(), "Ready");
}

void CubeNode::processFeedback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
  switch (feedback->event_type) {
    case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
      {
        // compute difference vector for this cube
        tf2::Vector3 fb_pos(
          feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
        std::size_t index = static_cast<std::size_t>(std::stoi(feedback->marker_name));

        if (index > positions_.size()) {
          return;
        }

        tf2::Vector3 fb_delta = fb_pos - positions_[index];

        // move all markers in that direction
        for (std::size_t i = 0; i < positions_.size(); ++i) {
          double d = fb_pos.distance(positions_[i]);
          double t = 1 / ( d * 5.0 + 1.0) - 0.2;
          if (t < 0.0) {
            t = 0.0;
          }

          positions_[i] += t * fb_delta;

          if (i == index) {
            positions_[i] = fb_pos;
          }

          geometry_msgs::msg::Pose pose;
          pose.position.x = positions_[i].x();
          pose.position.y = positions_[i].y();
          pose.position.z = positions_[i].z();

          server_->setPose(std::to_string(i), pose);
        }
        break;
      }
  }
  server_->applyChanges();
}

visualization_msgs::msg::InteractiveMarkerControl & CubeNode::makeBoxControl(
  visualization_msgs::msg::InteractiveMarker & msg)
{
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.always_visible = true;
  control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::VIEW_FACING;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
  control.independent_marker_orientation = true;

  visualization_msgs::msg::Marker marker;

  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.scale.x = msg.scale;
  marker.scale.y = msg.scale;
  marker.scale.z = msg.scale;
  marker.color.r = static_cast<float>(0.65 + 0.7 * msg.pose.position.x);
  marker.color.g = static_cast<float>(0.65 + 0.7 * msg.pose.position.y);
  marker.color.b = static_cast<float>(0.65 + 0.7 * msg.pose.position.z);
  marker.color.a = 1.0f;

  control.markers.push_back(marker);
  msg.controls.push_back(control);

  return msg.controls.back();
}

void CubeNode::makeCube()
{
  const int side_length = 10;
  const double step = 1.0 / static_cast<double>(side_length);
  int count = 0;

  positions_.reserve(side_length * side_length * side_length);

  for (double x = -0.5; x < 0.5; x += step) {
    for (double y = -0.5; y < 0.5; y += step) {
      for (double z = 0.0; z < 1.0; z += step) {
        visualization_msgs::msg::InteractiveMarker int_marker;
        int_marker.header.frame_id = "base_link";
        int_marker.scale = static_cast<float>(step);

        int_marker.pose.position.x = x;
        int_marker.pose.position.y = y;
        int_marker.pose.position.z = z;

        positions_.push_back(tf2::Vector3(x, y, z));

        int_marker.name = std::to_string(count);

        makeBoxControl(int_marker);

        server_->insert(int_marker);
        server_->setCallback(
          int_marker.name,
          std::bind(&CubeNode::processFeedback, this, std::placeholders::_1));

        count++;
      }
    }
  }
}

}  // namespace interactive_marker_tutorials

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<interactive_marker_tutorials::CubeNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
