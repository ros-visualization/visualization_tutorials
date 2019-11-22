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

#include "interactive_markers/interactive_marker_server.hpp"
#include "interactive_markers/menu_handler.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"

namespace interactive_marker_tutorials
{

visualization_msgs::msg::Marker makeBox(visualization_msgs::msg::InteractiveMarker & msg)
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

visualization_msgs::msg::InteractiveMarkerControl & makeBoxControl(
  visualization_msgs::msg::InteractiveMarker & msg)
{
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(makeBox(msg));
  msg.controls.push_back(control);

  return msg.controls.back();
}

visualization_msgs::msg::InteractiveMarker makeMenuMarker(const std::string name)
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.scale = 1.0;
  int_marker.name = name;

  visualization_msgs::msg::InteractiveMarkerControl control;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
  control.always_visible = true;

  control.markers.push_back(makeBox(int_marker));
  int_marker.controls.push_back(control);

  return int_marker;
}


class MenuInteractiveServerNode : public rclcpp::Node
{
public:
  explicit MenuInteractiveServerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~MenuInteractiveServerNode() = default;

private:
  void initializeMenu();

  void enableCallback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);

  void modeCallback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);

  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
  interactive_markers::MenuHandler menu_handler_;
  interactive_markers::MenuHandler::EntryHandle menu_handler_first_entry_;
  interactive_markers::MenuHandler::EntryHandle menu_handler_mode_last_;
};  // class MenuInteractiveServerNode

MenuInteractiveServerNode::MenuInteractiveServerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("menu_node", options)
{
  server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
    "menu",
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_topics_interface(),
    get_node_services_interface());
  initializeMenu();
  server_->insert(makeMenuMarker("menu_marker"));
  menu_handler_.apply(*server_, "menu_marker");
  server_->applyChanges();
  RCLCPP_INFO(get_logger(), "Ready");
}

void MenuInteractiveServerNode::initializeMenu()
{
  using namespace std::placeholders;

  menu_handler_first_entry_ = menu_handler_.insert("First Entry");
  interactive_markers::MenuHandler::EntryHandle entry = menu_handler_.insert(
    menu_handler_first_entry_, "deep");
  entry = menu_handler_.insert(entry, "sub");
  entry = menu_handler_.insert(
    entry,
    "menu",
    [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr)
    {
      RCLCPP_INFO(get_logger(), "The deep sub-menu has been found.");
    });

  menu_handler_.setCheckState(
    menu_handler_.insert(
      "Show First Entry", std::bind(&MenuInteractiveServerNode::enableCallback, this, _1)),
    interactive_markers::MenuHandler::CHECKED);

  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler_.insert("Switch");

  for (int i = 0; i < 5; ++i) {
    std::ostringstream s;
    s << "Mode " << i;
    menu_handler_mode_last_ = menu_handler_.insert(
      sub_menu_handle, s.str(), std::bind(&MenuInteractiveServerNode::modeCallback, this, _1));
    menu_handler_.setCheckState(
      menu_handler_mode_last_, interactive_markers::MenuHandler::UNCHECKED);
  }

  // check the very last entry
  menu_handler_.setCheckState(menu_handler_mode_last_, interactive_markers::MenuHandler::CHECKED);
}

void MenuInteractiveServerNode::enableCallback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
  interactive_markers::MenuHandler::EntryHandle handle = feedback->menu_entry_id;
  interactive_markers::MenuHandler::CheckState state;
  menu_handler_.getCheckState(handle, state);

  if (state == interactive_markers::MenuHandler::CHECKED) {
    menu_handler_.setCheckState(handle, interactive_markers::MenuHandler::UNCHECKED);
    RCLCPP_INFO(get_logger(), "Hiding first menu entry");
    menu_handler_.setVisible(menu_handler_first_entry_, false);
  } else {
    menu_handler_.setCheckState(handle, interactive_markers::MenuHandler::CHECKED);
    RCLCPP_INFO(get_logger(), "Showing first menu entry");
    menu_handler_.setVisible(menu_handler_first_entry_, true);
  }
  menu_handler_.reApply(*server_);
  RCLCPP_INFO(get_logger(), "Update");
  server_->applyChanges();
}

void MenuInteractiveServerNode::modeCallback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
  menu_handler_.setCheckState(
    menu_handler_mode_last_, interactive_markers::MenuHandler::UNCHECKED);
  menu_handler_mode_last_ = feedback->menu_entry_id;
  menu_handler_.setCheckState(menu_handler_mode_last_, interactive_markers::MenuHandler::CHECKED);

  RCLCPP_INFO(get_logger(), "Switching to menu entry #%d", menu_handler_mode_last_);

  menu_handler_.reApply(*server_);
  server_->applyChanges();
}

}  // namespace interactive_marker_tutorials

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<interactive_marker_tutorials::MenuInteractiveServerNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
