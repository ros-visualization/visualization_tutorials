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

#include <cstdlib>
#include <memory>
#include <string>
#include <sstream>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"

namespace interactive_marker_tutorials
{

class PongGameNode : public rclcpp::Node
{
  constexpr static float FIELD_WIDTH = 12.0f;
  constexpr static float FIELD_HEIGHT = 8.0f;
  constexpr static float BORDER_SIZE = 0.5f;
  constexpr static float PADDLE_SIZE = 2.0f;
  constexpr static float UPDATE_RATE = 1.0f / 30.0f;
  constexpr static float PLAYER_X = FIELD_WIDTH * 0.5f + BORDER_SIZE;
  constexpr static float AI_SPEED_LIMIT = 0.25f;

public:
  explicit PongGameNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("pong_game", options),
    last_ball_pos_x_(0),
    last_ball_pos_y_(0)
  {
    server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
      "pong",
      get_node_base_interface(),
      get_node_clock_interface(),
      get_node_logging_interface(),
      get_node_topics_interface(),
      get_node_services_interface());

    player_contexts_.resize(2);

    makeFieldMarker();
    makePaddleMarkers();
    makeBallMarker();

    resetRound();
    updateScore();

    game_loop_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(UPDATE_RATE * 1000.0f)),
      std::bind(&PongGameNode::spinOnce, this));

    RCLCPP_INFO(get_logger(), "Ready");
  }

  ~PongGameNode() = default;

private:
  /// Main control loop
  void spinOnce()
  {
    if (player_contexts_[0].active || player_contexts_[1].active) {
      float ball_dx = speed_ * ball_dir_x_;
      float ball_dy = speed_ * ball_dir_y_;

      ball_pos_x_ += ball_dx;
      ball_pos_y_ += ball_dy;

      // bounce off top / bottom
      float t = 0.0f;
      if (reflect(ball_pos_y_, last_ball_pos_y_, FIELD_HEIGHT * 0.5, t)) {
        ball_pos_x_ -= t * ball_dx;
        ball_pos_y_ -= t * ball_dy;

        ball_dir_y_ *= -1.0f;

        ball_dx = speed_ * ball_dir_x_;
        ball_dy = speed_ * ball_dir_y_;
        ball_pos_x_ += t * ball_dx;
        ball_pos_y_ += t * ball_dy;
      }

      size_t player = ball_pos_x_ > 0 ? 1 : 0;

      // reflect on paddles
      if (fabs(last_ball_pos_x_) < FIELD_WIDTH * 0.5f &&
        fabs(ball_pos_x_) >= FIELD_WIDTH * 0.5f)
      {
        // check if the paddle is roughly at the right position
        if (ball_pos_y_ > player_contexts_[player].pos - PADDLE_SIZE * 0.5f - 0.5f * BORDER_SIZE &&
          ball_pos_y_ < player_contexts_[player].pos + PADDLE_SIZE * 0.5f + 0.5f * BORDER_SIZE)
        {
          reflect(ball_pos_x_, last_ball_pos_x_, FIELD_WIDTH * 0.5f, t);
          ball_pos_x_ -= t * ball_dx;
          ball_pos_y_ -= t * ball_dy;

          // change direction based on distance to paddle center
          float offset = (ball_pos_y_ - player_contexts_[player].pos) / PADDLE_SIZE;

          ball_dir_x_ *= -1.0f;
          ball_dir_y_ += offset * 2.0f;

          normalizeBallVelocity();

          // limit angle to 45 deg
          if (fabs(ball_dir_y_) > 0.707106781f) {
            ball_dir_x_ = ball_dir_x_ > 0.0f ? 1.0f : -1.0f;
            ball_dir_y_ = ball_dir_y_ > 0.0f ? 1.0f : -1.0f;
            normalizeBallVelocity();
          }

          ball_dx = speed_ * ball_dir_x_;
          ball_dy = speed_ * ball_dir_y_;
          ball_pos_x_ += t * ball_dx;
          ball_pos_y_ += t * ball_dy;
        }
      }

      // ball hits the left/right border of the playing field
      if (fabs(ball_pos_x_) >= FIELD_WIDTH * 0.5f + 1.5f * BORDER_SIZE) {
        reflect(ball_pos_x_, last_ball_pos_x_, FIELD_WIDTH * 0.5f + 1.5f * BORDER_SIZE, t);
        ball_pos_x_ -= t * ball_dx;
        ball_pos_y_ -= t * ball_dy;
        updateBall();

        player_contexts_[1 - player].score++;
        updateScore();

        server_->applyChanges();
        resetRound();
      } else {
        updateBall();
      }

      last_ball_pos_x_ = ball_pos_x_;
      last_ball_pos_y_ = ball_pos_y_;

      // control computer player
      if (!player_contexts_[0].active || !player_contexts_[1].active) {
        size_t player = player_contexts_[0].active ? 1 : 0;
        float delta = ball_pos_y_ - player_contexts_[player].pos;
        // limit movement speed
        if (delta > AI_SPEED_LIMIT) {
          delta = AI_SPEED_LIMIT;
        }
        if (delta < -AI_SPEED_LIMIT) {
          delta = -AI_SPEED_LIMIT;
        }
        setPaddlePos(player, player_contexts_[player].pos + delta);
      }

      speed_ += 0.0003f;
    }

    server_->applyChanges();
  }

  void setPaddlePos(size_t player, float pos)
  {
    if (player > 1) {
      return;
    }

    // clamp
    if (pos > (FIELD_HEIGHT - PADDLE_SIZE) * 0.5f) {
      pos = (FIELD_HEIGHT - PADDLE_SIZE) * 0.5f;
    }
    if (pos < (FIELD_HEIGHT - PADDLE_SIZE) * -0.5f) {
      pos = (FIELD_HEIGHT - PADDLE_SIZE) * -0.5f;
    }

    player_contexts_[player].pos = pos;

    geometry_msgs::msg::Pose pose;
    pose.position.x = (player == 0) ? -PLAYER_X : PLAYER_X;
    pose.position.y = pos;

    std::string marker_name = (player == 0) ? "paddle0" : "paddle1";
    server_->setPose(marker_name, pose);
    server_->setPose(marker_name + "_display", pose);
  }

  void processPaddleFeedback(
    size_t player,
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
  {
    if (player > 1) {
      return;
    }

    std::string control_marker_name = feedback->marker_name;
    geometry_msgs::msg::Pose pose = feedback->pose;

    setPaddlePos(player, static_cast<float>(pose.position.y));

    if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_DOWN) {
      player_contexts_[player].active = true;
    }
    if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP) {
      player_contexts_[player].active = false;
    }
  }

  void resetRound()
  {
    speed_ = 6.0f * UPDATE_RATE;
    ball_pos_x_ = 0.0f;
    ball_pos_y_ = 0.0f;
    ball_dir_x_ = ball_dir_x_ > 0.0f ? 1.0f : -1.0f;
    ball_dir_y_ = rand() % 2 ? 1.0f : -1.0f;  // NOLINT(runtime/threadsafe_fn)
    normalizeBallVelocity();
  }

  /// Set length of the ball velocity vector to 1
  void normalizeBallVelocity()
  {
    float l = sqrt(ball_dir_x_ * ball_dir_x_ + ball_dir_y_ * ball_dir_y_);
    ball_dir_x_ /= l;
    ball_dir_y_ /= l;
  }

  /// Compute reflection
  /**
   * t [0...1] says how much the limit has been surpassed, relative to the distance
   * between last_pos and pos
   *
   * \return true if the given limit has been surpassed
   */
  bool reflect(const float & pos, const float & last_pos, const float & limit, float & t)
  {
    if (pos > limit) {
      t = (pos - limit) / (pos - last_pos);
      return true;
    }
    if (-pos > limit) {
      t = (-pos - limit) / (last_pos - pos);
      return true;
    }
    return false;
  }

  /// Update the ball marker
  void updateBall()
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = ball_pos_x_;
    pose.position.y = ball_pos_y_;
    server_->setPose("ball", pose);
  }

  /// Update the score marker
  void updateScore()
  {
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    int_marker.name = "score";

    visualization_msgs::msg::InteractiveMarkerControl control;
    control.always_visible = true;

    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;
    marker.scale.x = 1.5;
    marker.scale.y = 1.5;
    marker.scale.z = 1.5;

    std::ostringstream s;
    s << player_contexts_[0].score;
    marker.text = s.str();
    marker.pose.position.y = FIELD_HEIGHT * 0.5 + 4.0 * BORDER_SIZE;
    marker.pose.position.x = -1.0 * ( FIELD_WIDTH * 0.5 + BORDER_SIZE );
    control.markers.push_back(marker);

    s.str("");
    s << player_contexts_[1].score;
    marker.text = s.str();
    marker.pose.position.x *= -1;
    control.markers.push_back(marker);

    int_marker.controls.push_back(control);

    server_->insert(int_marker);
  }

  void makeFieldMarker()
  {
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    int_marker.name = "field";

    visualization_msgs::msg::InteractiveMarkerControl control;
    control.always_visible = true;

    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

    // Top Border
    marker.scale.x = FIELD_WIDTH + 6.0 * BORDER_SIZE;
    marker.scale.y = BORDER_SIZE;
    marker.scale.z = BORDER_SIZE;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = FIELD_HEIGHT * 0.5 + BORDER_SIZE;
    control.markers.push_back(marker);

    // Bottom Border
    marker.pose.position.y *= -1;
    control.markers.push_back(marker);

    // Left Border
    marker.scale.x = BORDER_SIZE;
    marker.scale.y = FIELD_HEIGHT + 3.0 * BORDER_SIZE;
    marker.scale.z = BORDER_SIZE;
    marker.pose.position.x = FIELD_WIDTH * 0.5 + 2.5 * BORDER_SIZE;
    marker.pose.position.y = 0.0;
    control.markers.push_back(marker);

    // Right Border
    marker.pose.position.x *= -1.0;
    control.markers.push_back(marker);

    // store
    int_marker.controls.push_back(control);
    server_->insert(int_marker);
  }

  void makePaddleMarkers()
  {
    using namespace std::placeholders;

    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";

    // Add a control for moving the paddle
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.always_visible = false;
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    tf2::Quaternion orien(0.0, 0.0, 1.0, 1.0);
    orien.normalize();
    control.orientation = tf2::toMsg(orien);

    // Add a visualization marker
    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 0.0f;
    marker.scale.x = BORDER_SIZE + 0.1;
    marker.scale.y = PADDLE_SIZE + 0.1;
    marker.scale.z = BORDER_SIZE + 0.1;
    marker.pose.position.z = 0.0;
    marker.pose.position.y = 0.0;

    control.markers.push_back(marker);

    int_marker.controls.push_back(control);

    // Control for player 1
    int_marker.name = "paddle0";
    int_marker.pose.position.x = -PLAYER_X;
    server_->insert(int_marker);
    server_->setCallback(
      int_marker.name, std::bind(&PongGameNode::processPaddleFeedback, this, 0, _1));

    // Control for player 2
    int_marker.name = "paddle1";
    int_marker.pose.position.x = PLAYER_X;
    server_->insert(int_marker);
    server_->setCallback(
      int_marker.name, std::bind(&PongGameNode::processPaddleFeedback, this, 1, _1));

    // Make display markers
    marker.scale.x = BORDER_SIZE;
    marker.scale.y = PADDLE_SIZE;
    marker.scale.z = BORDER_SIZE;
    marker.color.r = 0.5f;
    marker.color.a = 1.0f;

    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::NONE;
    control.always_visible = true;

    // Display for player 1
    int_marker.name = "paddle0_display";
    int_marker.pose.position.x = -PLAYER_X;

    marker.color.g = 1.0f;
    marker.color.b = 0.5f;

    int_marker.controls.clear();
    control.markers.clear();
    control.markers.push_back(marker);
    int_marker.controls.push_back(control);
    server_->insert(int_marker);

    // Display for player 2
    int_marker.name = "paddle1_display";
    int_marker.pose.position.x = PLAYER_X;

    marker.color.g = 0.5f;
    marker.color.b = 1.0f;

    int_marker.controls.clear();
    control.markers.clear();
    control.markers.push_back(marker);
    int_marker.controls.push_back(control);
    server_->insert(int_marker);
  }

  void makeBallMarker()
  {
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";

    visualization_msgs::msg::InteractiveMarkerControl control;
    control.always_visible = true;

    // Ball
    int_marker.name = "ball";

    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::NONE;
    tf2::Quaternion orien(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    control.orientation = tf2::toMsg(orien);

    visualization_msgs::msg::Marker marker;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.scale.x = BORDER_SIZE;
    marker.scale.y = BORDER_SIZE;
    marker.scale.z = BORDER_SIZE;
    control.markers.push_back(marker);

    int_marker.controls.push_back(control);

    server_->insert(int_marker);
  }

  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;

  rclcpp::TimerBase::SharedPtr game_loop_timer_;

  visualization_msgs::msg::InteractiveMarker field_marker_;

  struct PlayerContext
  {
    PlayerContext()
    : pos(0), active(false), score(0) {}
    float pos;
    bool active;
    int score;
  };

  std::vector<PlayerContext> player_contexts_;

  float last_ball_pos_x_;
  float last_ball_pos_y_;

  float ball_pos_x_;
  float ball_pos_y_;

  float ball_dir_x_;
  float ball_dir_y_;
  float speed_;
};  // class PongGameNode

}  // namespace interactive_marker_tutorials

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(interactive_marker_tutorials::PongGameNode)
