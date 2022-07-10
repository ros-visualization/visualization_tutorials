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
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "interactive_markers/tools.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"

#include "./utilities.hpp"

namespace interactive_marker_tutorials
{

bool testPointAgainstAabb2(
  const tf2::Vector3 & aabbMin1, const tf2::Vector3 & aabbMax1, const tf2::Vector3 & point)
{
  bool overlap = true;
  overlap = (aabbMin1.getX() > point.getX() || aabbMax1.getX() < point.getX()) ? false : overlap;
  overlap = (aabbMin1.getZ() > point.getZ() || aabbMax1.getZ() < point.getZ()) ? false : overlap;
  overlap = (aabbMin1.getY() > point.getY() || aabbMax1.getY() < point.getY()) ? false : overlap;
  return overlap;
}

class PointCloudSelectorNode : public rclcpp::Node
{
public:
  explicit PointCloudSelectorNode(std::vector<tf2::Vector3> & points)
  : rclcpp::Node("point_cloud_selector", rclcpp::NodeOptions()),
    min_sel_(-1, -1, -1),
    max_sel_(1, 1, 1),
    points_(points)
  {
    server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
      "point_cloud_selector",
      get_node_base_interface(),
      get_node_clock_interface(),
      get_node_logging_interface(),
      get_node_topics_interface(),
      get_node_services_interface());

    updateBox();
    updatePointClouds();
    makeSizeHandles();

    server_->applyChanges();

    RCLCPP_INFO(get_logger(), "Ready");
  }

  void processAxisFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
  {
    std::ostringstream oss;
    oss << feedback->marker_name << " is now at " <<
      feedback->pose.position.x <<
      ", " << feedback->pose.position.y <<
      ", " << feedback->pose.position.z;
    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());

    if (feedback->marker_name == "min_x") {
      min_sel_.setX(feedback->pose.position.x);
    }
    if (feedback->marker_name == "max_x") {
      max_sel_.setX(feedback->pose.position.x);
    }
    if (feedback->marker_name == "min_y") {
      min_sel_.setY(feedback->pose.position.y);
    }
    if (feedback->marker_name == "max_y") {
      max_sel_.setY(feedback->pose.position.y);
    }
    if (feedback->marker_name == "min_z") {
      min_sel_.setZ(feedback->pose.position.z);
    }
    if (feedback->marker_name == "max_z") {
      max_sel_.setZ(feedback->pose.position.z);
    }

    updateBox();
    updateSizeHandles();

    if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP) {
      updatePointClouds();
    }

    server_->applyChanges();
  }

  visualization_msgs::msg::Marker makeBox(
    const tf2::Vector3 & min_bound,
    const tf2::Vector3 & max_bound)
  {
    visualization_msgs::msg::Marker marker;

    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.scale.x = max_bound.x() - min_bound.x();
    marker.scale.y = max_bound.y() - min_bound.y();
    marker.scale.z = max_bound.z() - min_bound.z();
    marker.pose.position.x = 0.5 * (max_bound.x() + min_bound.x());
    marker.pose.position.y = 0.5 * (max_bound.y() + min_bound.y());
    marker.pose.position.z = 0.5 * (max_bound.z() + min_bound.z());
    marker.color.r = 0.5f;
    marker.color.g = 0.5f;
    marker.color.b = 0.5f;
    marker.color.a = 0.5f;

    return marker;
  }

  void updateBox()
  {
    visualization_msgs::msg::InteractiveMarker msg;
    msg.header.frame_id = "base_link";

    visualization_msgs::msg::InteractiveMarkerControl control;
    control.always_visible = false;
    control.markers.push_back(makeBox(min_sel_, max_sel_));
    msg.controls.push_back(control);

    server_->insert(msg);
  }

  void updatePointCloud(
    const std::string name,
    const std_msgs::msg::ColorRGBA & color,
    std::vector<tf2::Vector3> & points)
  {
    // create an interactive marker for our server
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    int_marker.name = name;

    // create a point cloud marker
    visualization_msgs::msg::Marker points_marker;
    points_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    points_marker.scale.x = 0.05;
    points_marker.scale.y = 0.05;
    points_marker.scale.z = 0.05;
    points_marker.color = color;

    for (std::size_t i = 0u; i < points.size(); ++i) {
      geometry_msgs::msg::Point point;
      point.x = points[i].x();
      point.y = points[i].y();
      point.z = points[i].z();
      points_marker.points.push_back(point);
    }

    // create container control
    visualization_msgs::msg::InteractiveMarkerControl points_control;
    points_control.always_visible = true;
    points_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::NONE;
    points_control.markers.push_back(points_marker);

    // add the control to the interactive marker
    int_marker.controls.push_back(points_control);

    server_->insert(int_marker);
  }

  void updatePointClouds()
  {
    std::vector<tf2::Vector3> points_in, points_out;
    points_in.reserve(points_.size());
    points_out.reserve(points_.size());

    // determine which points are selected (i.e. inside the selection box)
    for (std::size_t i = 0u; i < points_.size(); ++i) {
      if (testPointAgainstAabb2(min_sel_, max_sel_, points_[i])) {
        points_in.push_back(points_[i]);
      } else {
        points_out.push_back(points_[i]);
      }
    }

    std_msgs::msg::ColorRGBA in_color;
    in_color.r = 1.0f;
    in_color.g = 0.8f;
    in_color.b = 0.0f;
    in_color.a = 1.0f;

    std_msgs::msg::ColorRGBA out_color;
    out_color.r = 0.5f;
    out_color.g = 0.5f;
    out_color.b = 0.5f;
    out_color.a = 1.0f;

    updatePointCloud("selected_points", in_color, points_in);
    updatePointCloud("unselected_points", out_color, points_out);
  }

  void makeSizeHandles()
  {
    for (int axis = 0; axis < 3; ++axis) {
      for (int sign = -1; sign <= 1; sign += 2) {
        visualization_msgs::msg::InteractiveMarker int_marker;
        int_marker.header.frame_id = "base_link";
        int_marker.scale = 1.0;

        visualization_msgs::msg::InteractiveMarkerControl control;
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::INHERIT;
        control.always_visible = false;

        tf2::Quaternion orien;

        switch (axis) {
          case 0:
            int_marker.name = sign > 0 ? "max_x" : "min_x";
            int_marker.pose.position.x = sign > 0 ? max_sel_.x() : min_sel_.x();
            int_marker.pose.position.y = 0.5 * (max_sel_.y() + min_sel_.y());
            int_marker.pose.position.z = 0.5 * (max_sel_.z() + min_sel_.z());
            orien = tf2::Quaternion(1.0, 0.0, 0.0, 1.0);
            orien.normalize();
            control.orientation = tf2::toMsg(orien);
            break;
          case 1:
            int_marker.name = sign > 0 ? "max_y" : "min_y";
            int_marker.pose.position.x = 0.5 * (max_sel_.x() + min_sel_.x());
            int_marker.pose.position.y = sign > 0 ? max_sel_.y() : min_sel_.y();
            int_marker.pose.position.z = 0.5 * (max_sel_.z() + min_sel_.z());
            orien = tf2::Quaternion(0.0, 0.0, 1.0, 1.0);
            orien.normalize();
            control.orientation = tf2::toMsg(orien);
            break;
          default:
            int_marker.name = sign > 0 ? "max_z" : "min_z";
            int_marker.pose.position.x = 0.5 * (max_sel_.x() + min_sel_.x());
            int_marker.pose.position.y = 0.5 * (max_sel_.y() + min_sel_.y());
            int_marker.pose.position.z = sign > 0 ? max_sel_.z() : min_sel_.z();
            orien = tf2::Quaternion(0.0, -1.0, 0.0, 1.0);
            orien.normalize();
            control.orientation = tf2::toMsg(orien);
            break;
        }

        interactive_markers::makeArrow(int_marker, control, 0.5f * sign);

        int_marker.controls.push_back(control);
        server_->insert(
          int_marker,
          std::bind(&PointCloudSelectorNode::processAxisFeedback, this, std::placeholders::_1));
      }
    }
  }

  void updateSizeHandles()
  {
    for (int axis = 0; axis < 3; ++axis) {
      for (int sign = -1; sign <= 1; sign += 2) {
        std::string name;
        geometry_msgs::msg::Pose pose;

        switch (axis) {
          case 0:
            name = sign > 0 ? "max_x" : "min_x";
            pose.position.x = sign > 0 ? max_sel_.x() : min_sel_.x();
            pose.position.y = 0.5 * (max_sel_.y() + min_sel_.y());
            pose.position.z = 0.5 * (max_sel_.z() + min_sel_.z());
            break;
          case 1:
            name = sign > 0 ? "max_y" : "min_y";
            pose.position.x = 0.5 * (max_sel_.x() + min_sel_.x());
            pose.position.y = sign > 0 ? max_sel_.y() : min_sel_.y();
            pose.position.z = 0.5 * (max_sel_.z() + min_sel_.z());
            break;
          default:
            name = sign > 0 ? "max_z" : "min_z";
            pose.position.x = 0.5 * (max_sel_.x() + min_sel_.x());
            pose.position.y = 0.5 * (max_sel_.y() + min_sel_.y());
            pose.position.z = sign > 0 ? max_sel_.z() : min_sel_.z();
            break;
        }

        server_->setPose(name, pose);
      }
    }
  }

private:
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

  tf2::Vector3 min_sel_, max_sel_;
  std::vector<tf2::Vector3> points_;

  visualization_msgs::msg::InteractiveMarker sel_points_marker_;
  visualization_msgs::msg::InteractiveMarker unsel_points_marker_;
};  // class PointCloudSelectorNode

void makePoints(std::vector<tf2::Vector3> & points_out, int num_points)
{
  double radius = 3.0;
  double scale = 0.2;
  points_out.resize(num_points);
  for (int i = 0; i < num_points; ++i) {
    points_out[i].setX(scale * randFromRange(-radius, radius));
    points_out[i].setY(scale * randFromRange(-radius, radius));
    points_out[i].setZ(
      scale * radius * 0.2 *
      (sin(10.0 / radius * points_out[i].x()) + cos(10.0 / radius * points_out[i].y())));
  }
}

}  // namespace interactive_marker_tutorials

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::vector<tf2::Vector3> points;
  interactive_marker_tutorials::makePoints(points, 10000);
  std::shared_ptr<interactive_marker_tutorials::PointCloudSelectorNode> node(
    new interactive_marker_tutorials::PointCloudSelectorNode(points));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
