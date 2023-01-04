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

#include <chrono>
#include <memory>
#include <string>

#include "interactive_markers/interactive_marker_server.hpp"
#include "interactive_markers/menu_handler.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "./utilities.hpp"

using std::placeholders::_1;

namespace interactive_marker_tutorials
{

class BasicControlsNode : public rclcpp::Node
{
public:
  explicit BasicControlsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~BasicControlsNode() = default;

  inline void
  applyChanges()
  {
    server_->applyChanges();
  }

  visualization_msgs::msg::Marker
  makeBox(const visualization_msgs::msg::InteractiveMarker & msg);

  visualization_msgs::msg::InteractiveMarkerControl &
  makeBoxControl(visualization_msgs::msg::InteractiveMarker & msg);

  void
  make6DofMarker(
    bool fixed, unsigned int interaction_mode, const tf2::Vector3 & position, bool show_6dof);

  void
  makeRandomDofMarker(const tf2::Vector3 & position);

  void
  makeViewFacingMarker(const tf2::Vector3 & position);

  void
  makeQuadrocopterMarker(const tf2::Vector3 & position);

  void
  makeChessPieceMarker(const tf2::Vector3 & position);

  void
  makePanTiltMarker(const tf2::Vector3 & position);

  void
  makeMenuMarker(const tf2::Vector3 & position);

  void
  makeButtonMarker(const tf2::Vector3 & position);

  void
  makeMovingMarker(const tf2::Vector3 & position);

private:
  void
  frameCallback();

  void
  processFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);

  void
  alignMarker(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);

  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
  interactive_markers::MenuHandler menu_handler_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr frame_timer_;
};  // class BasicControlsNode

BasicControlsNode::BasicControlsNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("basic_controls", options),
  menu_handler_()
{
  server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
    "basic_controls",
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_topics_interface(),
    get_node_services_interface());

  menu_handler_.insert("First Entry", std::bind(&BasicControlsNode::processFeedback, this, _1));
  menu_handler_.insert("Second Entry", std::bind(&BasicControlsNode::processFeedback, this, _1));
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler_.insert("Submenu");
  menu_handler_.insert(
    sub_menu_handle, "First Entry", std::bind(&BasicControlsNode::processFeedback, this, _1));
  menu_handler_.insert(
    sub_menu_handle, "Second Entry", std::bind(&BasicControlsNode::processFeedback, this, _1));

  // create a timer to update the published transforms
  frame_timer_ = create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&BasicControlsNode::frameCallback, this));
}

visualization_msgs::msg::Marker
BasicControlsNode::makeBox(const visualization_msgs::msg::InteractiveMarker & msg)
{
  visualization_msgs::msg::Marker marker;

  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

visualization_msgs::msg::InteractiveMarkerControl &
BasicControlsNode::makeBoxControl(visualization_msgs::msg::InteractiveMarker & msg)
{
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(makeBox(msg));
  msg.controls.push_back(control);

  return msg.controls.back();
}

void
BasicControlsNode::frameCallback()
{
  static uint32_t counter = 0;

  if (!tf_broadcaster_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(shared_from_this());
  }

  tf2::TimePoint tf_time_point(std::chrono::nanoseconds(this->get_clock()->now().nanoseconds()));

  tf2::Stamped<tf2::Transform> transform;
  transform.stamp_ = tf_time_point;
  transform.frame_id_ = "base_link";
  transform.setOrigin(tf2::Vector3(0.0, 0.0, sin(static_cast<double>(counter) / 140.0) * 2.0));
  transform.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg = tf2::toMsg(transform);
  transform_msg.child_frame_id = "moving_frame";
  tf_broadcaster_->sendTransform(transform_msg);

  transform.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
  tf2::Quaternion quat;
  quat.setRPY(0.0, static_cast<double>(counter) / 140.0, 0.0);
  transform.setRotation(quat);
  transform_msg = tf2::toMsg(transform);
  transform_msg.child_frame_id = "rotating_frame";
  tf_broadcaster_->sendTransform(transform_msg);

  counter++;
}

void
BasicControlsNode::processFeedback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
  std::ostringstream oss;
  oss << "Feedback from marker '" << feedback->marker_name << "' " <<
    " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if (feedback->mouse_point_valid) {
    mouse_point_ss << " at " << feedback->mouse_point.x <<
      ", " << feedback->mouse_point.y <<
      ", " << feedback->mouse_point.z <<
      " in frame " << feedback->header.frame_id;
  }

  switch (feedback->event_type) {
    case visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK:
      oss << ": button click" << mouse_point_ss.str() << ".";
      RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
      break;

    case visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT:
      oss << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << ".";
      RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
      break;

    case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
      oss << ": pose changed" <<
        "\nposition = " <<
        feedback->pose.position.x <<
        ", " << feedback->pose.position.y <<
        ", " << feedback->pose.position.z <<
        "\norientation = " <<
        feedback->pose.orientation.w <<
        ", " << feedback->pose.orientation.x <<
        ", " << feedback->pose.orientation.y <<
        ", " << feedback->pose.orientation.z <<
        "\nframe: " << feedback->header.frame_id <<
        " time: " << feedback->header.stamp.sec << "sec, " <<
        feedback->header.stamp.nanosec << " nsec";
      RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
      break;

    case visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_DOWN:
      oss << ": mouse down" << mouse_point_ss.str() << ".";
      RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
      break;

    case visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP:
      oss << ": mouse up" << mouse_point_ss.str() << ".";
      RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
      break;
  }

  server_->applyChanges();
}

void
BasicControlsNode::alignMarker(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
  geometry_msgs::msg::Pose pose = feedback->pose;

  pose.position.x = round(pose.position.x - 0.5) + 0.5;
  pose.position.y = round(pose.position.y - 0.5) + 0.5;

  std::ostringstream oss;
  oss << feedback->marker_name << ":" <<
    " aligning position = " <<
    feedback->pose.position.x <<
    ", " << feedback->pose.position.y <<
    ", " << feedback->pose.position.z <<
    " to " <<
    pose.position.x <<
    ", " << pose.position.y <<
    ", " << pose.position.z;
  RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());

  server_->setPose(feedback->marker_name, pose);
  server_->applyChanges();
}

void
BasicControlsNode::make6DofMarker(
  bool fixed, unsigned int interaction_mode, const tf2::Vector3 & position, bool show_6dof)
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.pose.position.x = position.getX();
  int_marker.pose.position.y = position.getY();
  int_marker.pose.position.z = position.getZ();
  int_marker.scale = 1;

  int_marker.name = "simple_6dof";
  int_marker.description = "Simple 6-DOF Control";

  // insert a box
  makeBoxControl(int_marker);
  int_marker.controls[0].interaction_mode = interaction_mode;

  visualization_msgs::msg::InteractiveMarkerControl control;

  if (fixed) {
    int_marker.name += "_fixed";
    int_marker.description += "\n(fixed orientation)";
    control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  }

  if (interaction_mode != visualization_msgs::msg::InteractiveMarkerControl::NONE) {
    std::string mode_text;
    if (interaction_mode == visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D) {
      mode_text = "MOVE_3D";
    } else if (interaction_mode == visualization_msgs::msg::InteractiveMarkerControl::ROTATE_3D) {
      mode_text = "ROTATE_3D";
      // TODO(jacobperron): cpplint conflicts with uncrustify
      //                    https://github.com/ament/ament_lint/issues/158
    } else {
      if (interaction_mode == visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D) {
        mode_text = "MOVE_ROTATE_3D";
      }
    }
    int_marker.name += "_" + mode_text;
    int_marker.description = std::string("3D Control") +
      (show_6dof ? " + 6-DOF controls" : "") +
      "\n" +
      mode_text;
  }

  if (show_6dof) {
    tf2::Quaternion orien(1.0, 0.0, 0.0, 1.0);
    orien.normalize();
    control.orientation = tf2::toMsg(orien);
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    orien = tf2::Quaternion(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    control.orientation = tf2::toMsg(orien);
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    orien = tf2::Quaternion(0.0, 0.0, 1.0, 1.0);
    orien.normalize();
    control.orientation = tf2::toMsg(orien);
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  server_->insert(int_marker);
  server_->setCallback(int_marker.name, std::bind(&BasicControlsNode::processFeedback, this, _1));
  if (interaction_mode != visualization_msgs::msg::InteractiveMarkerControl::NONE) {
    menu_handler_.apply(*server_, int_marker.name);
  }
}

void
BasicControlsNode::makeRandomDofMarker(const tf2::Vector3 & position)
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.pose.position.x = position.getX();
  int_marker.pose.position.y = position.getY();
  int_marker.pose.position.z = position.getZ();
  int_marker.scale = 1;

  int_marker.name = "6dof_random_axes";
  int_marker.description = "6-DOF\n(Arbitrary Axes)";

  makeBoxControl(int_marker);

  visualization_msgs::msg::InteractiveMarkerControl control;

  for (int i = 0; i < 3; i++) {
    tf2::Quaternion orien(
      randFromRange(-1, 1), randFromRange(-1, 1), randFromRange(-1, 1), randFromRange(-1, 1));
    orien.normalize();
    control.orientation = tf2::toMsg(orien);
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  server_->insert(int_marker);
  server_->setCallback(int_marker.name, std::bind(&BasicControlsNode::processFeedback, this, _1));
}

void
BasicControlsNode::makeViewFacingMarker(const tf2::Vector3 & position)
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.pose.position.x = position.getX();
  int_marker.pose.position.y = position.getY();
  int_marker.pose.position.z = position.getZ();
  int_marker.scale = 1;

  int_marker.name = "view_facing";
  int_marker.description = "View Facing 6-DOF";

  visualization_msgs::msg::InteractiveMarkerControl control;

  // make a control that rotates around the view axis
  control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::VIEW_FACING;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation.w = 1;
  control.name = "rotate";

  int_marker.controls.push_back(control);

  // create a box in the center which should not be view facing,
  // but move in the camera plane.
  control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::VIEW_FACING;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
  control.independent_marker_orientation = true;
  control.name = "move";

  control.markers.push_back(makeBox(int_marker));
  control.always_visible = true;

  int_marker.controls.push_back(control);

  server_->insert(int_marker);
  server_->setCallback(int_marker.name, std::bind(&BasicControlsNode::processFeedback, this, _1));
}

void
BasicControlsNode::makeQuadrocopterMarker(const tf2::Vector3 & position)
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.pose.position.x = position.getX();
  int_marker.pose.position.y = position.getY();
  int_marker.pose.position.z = position.getZ();
  int_marker.scale = 1;

  int_marker.name = "quadrocopter";
  int_marker.description = "Quadrocopter";

  makeBoxControl(int_marker);

  visualization_msgs::msg::InteractiveMarkerControl control;

  tf2::Quaternion orien(0.0, 1.0, 0.0, 1.0);
  orien.normalize();
  control.orientation = tf2::toMsg(orien);
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE;
  int_marker.controls.push_back(control);
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  server_->insert(int_marker);
  server_->setCallback(int_marker.name, std::bind(&BasicControlsNode::processFeedback, this, _1));
}

void
BasicControlsNode::makeChessPieceMarker(const tf2::Vector3 & position)
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.pose.position.x = position.getX();
  int_marker.pose.position.y = position.getY();
  int_marker.pose.position.z = position.getZ();
  int_marker.scale = 1;

  int_marker.name = "chess_piece";
  int_marker.description = "Chess Piece\n(2D Move + Alignment)";

  visualization_msgs::msg::InteractiveMarkerControl control;

  tf2::Quaternion orien(0.0, 1.0, 0.0, 1.0);
  orien.normalize();
  control.orientation = tf2::toMsg(orien);
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
  int_marker.controls.push_back(control);

  // make a box which also moves in the plane
  control.markers.push_back(makeBox(int_marker));
  control.always_visible = true;
  int_marker.controls.push_back(control);

  // we want to use our special callback function
  server_->insert(int_marker);
  server_->setCallback(int_marker.name, std::bind(&BasicControlsNode::processFeedback, this, _1));

  // set different callback for POSE_UPDATE feedback
  server_->setCallback(
    int_marker.name,
    std::bind(&BasicControlsNode::alignMarker, this, _1),
    visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE);
}

void
BasicControlsNode::makePanTiltMarker(const tf2::Vector3 & position)
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.pose.position.x = position.getX();
  int_marker.pose.position.y = position.getY();
  int_marker.pose.position.z = position.getZ();
  int_marker.scale = 1;

  int_marker.name = "pan_tilt";
  int_marker.description = "Pan / Tilt";

  makeBoxControl(int_marker);

  visualization_msgs::msg::InteractiveMarkerControl control;

  tf2::Quaternion orien(0.0, 1.0, 0.0, 1.0);
  orien.normalize();
  control.orientation = tf2::toMsg(orien);
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  int_marker.controls.push_back(control);

  orien = tf2::Quaternion(0.0, 0.0, 1.0, 1.0);
  orien.normalize();
  control.orientation = tf2::toMsg(orien);
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::INHERIT;
  int_marker.controls.push_back(control);

  server_->insert(int_marker);
  server_->setCallback(int_marker.name, std::bind(&BasicControlsNode::processFeedback, this, _1));
}

void
BasicControlsNode::makeMenuMarker(const tf2::Vector3 & position)
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.pose.position.x = position.getX();
  int_marker.pose.position.y = position.getY();
  int_marker.pose.position.z = position.getZ();
  int_marker.scale = 1;

  int_marker.name = "context_menu";
  int_marker.description = "Context Menu\n(Right Click)";

  visualization_msgs::msg::InteractiveMarkerControl control;

  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MENU;
  control.name = "menu_only_control";

  visualization_msgs::msg::Marker marker = makeBox(int_marker);
  control.markers.push_back(marker);
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server_->insert(int_marker);
  server_->setCallback(int_marker.name, std::bind(&BasicControlsNode::processFeedback, this, _1));
  menu_handler_.apply(*server_, int_marker.name);
}

void
BasicControlsNode::makeButtonMarker(const tf2::Vector3 & position)
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.pose.position.x = position.getX();
  int_marker.pose.position.y = position.getY();
  int_marker.pose.position.z = position.getZ();
  int_marker.scale = 1;

  int_marker.name = "button";
  int_marker.description = "Button\n(Left Click)";

  visualization_msgs::msg::InteractiveMarkerControl control;

  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
  control.name = "button_control";

  visualization_msgs::msg::Marker marker = makeBox(int_marker);
  control.markers.push_back(marker);
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server_->insert(int_marker);
  server_->setCallback(int_marker.name, std::bind(&BasicControlsNode::processFeedback, this, _1));
}

void
BasicControlsNode::makeMovingMarker(const tf2::Vector3 & position)
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "moving_frame";
  int_marker.pose.position.x = position.getX();
  int_marker.pose.position.y = position.getY();
  int_marker.pose.position.z = position.getZ();
  int_marker.scale = 1;

  int_marker.name = "moving";
  int_marker.description = "Marker Attached to a\nMoving Frame";

  visualization_msgs::msg::InteractiveMarkerControl control;

  tf2::Quaternion orien(1.0, 0.0, 0.0, 1.0);
  orien.normalize();
  control.orientation = tf2::toMsg(orien);
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
  control.always_visible = true;
  control.markers.push_back(makeBox(int_marker));
  int_marker.controls.push_back(control);

  server_->insert(int_marker);
  server_->setCallback(int_marker.name, std::bind(&BasicControlsNode::processFeedback, this, _1));
}

}  // namespace interactive_marker_tutorials

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto basic_controls = std::make_shared<interactive_marker_tutorials::BasicControlsNode>();

  tf2::Vector3 position(-3, 3, 0);
  basic_controls->make6DofMarker(
    false, visualization_msgs::msg::InteractiveMarkerControl::NONE, position, true);
  position = tf2::Vector3(0, 3, 0);
  basic_controls->make6DofMarker(
    true, visualization_msgs::msg::InteractiveMarkerControl::NONE, position, true);
  position = tf2::Vector3(3, 3, 0);
  basic_controls->makeRandomDofMarker(position);
  position = tf2::Vector3(-3, 0, 0);
  basic_controls->make6DofMarker(
    false, visualization_msgs::msg::InteractiveMarkerControl::ROTATE_3D, position, false);
  position = tf2::Vector3(0, 0, 0);
  basic_controls->make6DofMarker(
    false, visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D, position, true);
  position = tf2::Vector3(3, 0, 0);
  basic_controls->make6DofMarker(
    false, visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D, position, false);
  position = tf2::Vector3(-3, -3, 0);
  basic_controls->makeViewFacingMarker(position);
  position = tf2::Vector3(0, -3, 0);
  basic_controls->makeQuadrocopterMarker(position);
  position = tf2::Vector3(3, -3, 0);
  basic_controls->makeChessPieceMarker(position);
  position = tf2::Vector3(-3, -6, 0);
  basic_controls->makePanTiltMarker(position);
  position = tf2::Vector3(0, -6, 0);
  basic_controls->makeMovingMarker(position);
  position = tf2::Vector3(3, -6, 0);
  basic_controls->makeMenuMarker(position);
  position = tf2::Vector3(0, -9, 0);
  basic_controls->makeButtonMarker(position);

  basic_controls->applyChanges();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(basic_controls);
  RCLCPP_INFO(basic_controls->get_logger(), "Ready");
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
