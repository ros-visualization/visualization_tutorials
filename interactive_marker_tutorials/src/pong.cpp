/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <ros/ros.h>

#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_interface.h>

#include <actionlib/client/simple_action_client.h>

#include <math.h>

using namespace visualization_msgs;

static const float FIELD_WIDTH = 12.0;
static const float FIELD_HEIGHT = 8.0;
static const float BORDER_SIZE = 0.5;
static const float PADDLE_SIZE = 2.0;


class PongGame
{
public:

  PongGame() :
  interface_("pong", false),
  last_ball_pos_x_(0),
  ball_pos_x_(0),
  ball_pos_y_(0),
  ball_vel_x_(1),
  ball_vel_y_(-1),
  speed_(0.05)
  {
    player_contexts_.resize(2);

    makeFieldMarker();
    makePaddleMarkers();
    makeBallMarker();

    updateScore();

    ros::NodeHandle nh;
    game_loop_timer_ =  nh.createTimer(ros::Duration(1.0 / 100.0), boost::bind( &PongGame::spinOnce, this ) );
  }

  void spinOnce()
  {
    if ( player_contexts_[0].active || player_contexts_[1].active )
    {
      ball_pos_x_ += speed_ * (float)ball_vel_x_;
      ball_pos_y_ += speed_ * (float)ball_vel_y_;

      speed_ += 0.00005;

      // bounce off top / bottom
      reflect ( ball_pos_y_, FIELD_HEIGHT * 0.5, ball_vel_y_ );

      int player = ball_pos_x_ > 0 ? 1 : 0;

      // reflect on paddles
      if ( fabs(last_ball_pos_x_) < FIELD_WIDTH * 0.5 &&
           fabs(ball_pos_x_) >= FIELD_WIDTH * 0.5 )
      {
        // check if the paddle is at the right position
        if ( ball_pos_y_ > player_contexts_[player].pos - PADDLE_SIZE * 0.5 &&
             ball_pos_y_ < player_contexts_[player].pos + PADDLE_SIZE * 0.5 )
        {
          reflect ( ball_pos_x_, FIELD_WIDTH * 0.5, ball_vel_x_ );
        }
      }

      // detect
      if ( fabs(last_ball_pos_x_) > FIELD_WIDTH * 0.5 + 2.0*BORDER_SIZE )
      {
        ball_pos_x_ = 0.0;
        ball_pos_y_ = 0.0;
        player_contexts_[1-player].score++;
        updateScore();
        speed_ = 0.05;
        ros::Time::sleepUntil( ros::Time::now() + ros::Duration(2.0) );
      }

      geometry_msgs::Pose pose;
      pose.position.x = ball_pos_x_;
      pose.position.y = ball_pos_y_;
      interface_.setPose( "ball", pose );
    }

    interface_.publishUpdate();
    last_ball_pos_x_ = ball_pos_x_;
  }

private:

  void reflect( float &pos, float limit, int &vel )
  {
    if ( vel*pos > limit )
    {
      pos = vel*2.0*limit - pos;
      vel *= -1;
    }
  }

  void updateScore()
  {
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "/base_link";
    int_marker.name = "score";

    InteractiveMarkerControl control;
    control.always_visible = true;

    Marker marker;
    marker.type = Marker::TEXT_VIEW_FACING;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    std::ostringstream s;
    s << player_contexts_[0].score;
    marker.text = s.str();
    marker.pose.position.y = FIELD_HEIGHT*0.5 + 4.0*BORDER_SIZE;
    marker.pose.position.x = -1.0 * ( FIELD_WIDTH * 0.5 + BORDER_SIZE );
    control.markers.push_back( marker );

    s.str("");
    s << player_contexts_[1].score;
    marker.text = s.str();
    marker.pose.position.x *= -1;
    control.markers.push_back( marker );

    int_marker.controls.push_back( control );

    interface_.insert( int_marker );
    interface_.publishUpdate();
  }

  void makeFieldMarker()
  {
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "/base_link";
    int_marker.name = "field";

    InteractiveMarkerControl control;
    control.always_visible = true;

    Marker marker;
    marker.type = Marker::CUBE;
    marker.color.r = 0.7;
    marker.color.g = 0.7;
    marker.color.b = 0.7;
    marker.color.a = 1.0;

    // Top Border
    marker.scale.x = FIELD_WIDTH + 6.0 * BORDER_SIZE;
    marker.scale.y = BORDER_SIZE;
    marker.scale.z = BORDER_SIZE;
    marker.pose.position.x = 0;
    marker.pose.position.y = FIELD_HEIGHT*0.5 + BORDER_SIZE;
    control.markers.push_back( marker );

    // Bottom Border
    marker.pose.position.y *= -1;
    control.markers.push_back( marker );

    // Left Border
    marker.scale.x = BORDER_SIZE;
    marker.scale.y = FIELD_HEIGHT + 3.0*BORDER_SIZE;
    marker.scale.z = BORDER_SIZE;
    marker.pose.position.x = FIELD_WIDTH*0.5 + 2.5*BORDER_SIZE;
    marker.pose.position.y = 0;
    control.markers.push_back( marker );

    // Right Border
    marker.pose.position.x *= -1;
    control.markers.push_back( marker );

    // store
    int_marker.controls.push_back( control );
    interface_.insert( int_marker );
  }

  void makePaddleMarkers()
  {
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "/base_link";

    // Add a control for moving the paddle
    InteractiveMarkerControl control;
    control.always_visible = false;
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    control.orientation.w = 1;
    control.orientation.z = 1;

    // Add a visualization marker
    Marker marker;
    marker.type = Marker::CUBE;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 0.0;
    marker.scale.x = BORDER_SIZE * 1.05;
    marker.scale.y = PADDLE_SIZE * 1.05;
    marker.scale.z = BORDER_SIZE * 1.05;
    marker.pose.position.z = 0;
    marker.pose.position.y = 0;

    control.markers.push_back( marker );

    int_marker.controls.push_back( control );

    float player_x = FIELD_WIDTH * 0.5 + BORDER_SIZE;

    // Control for player 1
    int_marker.name = "player_1_control";
    int_marker.pose.position.x = -player_x;
    interface_.insert( int_marker, boost::bind( &PongGame::playerPosUpdated, this, _1 ) );

    // Control for player 2
    int_marker.name = "player_2_control";
    int_marker.pose.position.x = player_x;
    interface_.insert( int_marker, boost::bind( &PongGame::playerPosUpdated, this, _1 ) );

    // Make display markers
    int_marker.controls.clear();
    control.markers.clear();
    marker.scale.x = BORDER_SIZE;
    marker.scale.y = PADDLE_SIZE;
    marker.scale.z = BORDER_SIZE;
    marker.color.r = 0.7;
    marker.color.g = 0.7;
    marker.color.b = 0.7;
    marker.color.a = 1.0;

    control.interaction_mode = InteractiveMarkerControl::NONE;
    control.always_visible = true;

    control.markers.push_back( marker );

    int_marker.controls.push_back( control );

    // Display for player 1
    int_marker.name = "player_1_display";
    int_marker.pose.position.x = -player_x;
    interface_.insert( int_marker );

    // Display for player 2
    int_marker.name = "player_2_display";
    int_marker.pose.position.x = player_x;
    interface_.insert( int_marker );
  }

  void makeBallMarker()
  {
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "/base_link";

    InteractiveMarkerControl control;
    control.always_visible = true;

    // Ball
    int_marker.name = "ball";

    control.interaction_mode = InteractiveMarkerControl::NONE;
    control.orientation.w = 1;
    control.orientation.y = 1;

    Marker marker;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.type = Marker::CYLINDER;
    marker.scale.x = BORDER_SIZE;
    marker.scale.y = BORDER_SIZE;
    marker.scale.z = BORDER_SIZE;
    control.markers.push_back( marker );

    int_marker.controls.push_back( control );

    interface_.insert( int_marker );
  }

  void playerPosUpdated( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
  {
    std::string control_marker_name = feedback->marker_name;

    int player;
    std::string display_marker_name;

    if ( control_marker_name == "player_1_control" )
    {
      player = 0;
      display_marker_name = "player_1_display";
    }
    else if ( control_marker_name == "player_2_control" )
    {
      player = 1;
      display_marker_name = "player_2_display";
    }
    else
    {
      return;
    }

    geometry_msgs::Pose pose = feedback->pose;

    if ( pose.position.y > (FIELD_HEIGHT - PADDLE_SIZE) * 0.5 )
    {
      pose.position.y = (FIELD_HEIGHT - PADDLE_SIZE) * 0.5;
      interface_.setPose( control_marker_name, pose );
    }
    if ( pose.position.y < (FIELD_HEIGHT - PADDLE_SIZE) * -0.5 )
    {
      pose.position.y = (FIELD_HEIGHT - PADDLE_SIZE) * -0.5;
      interface_.setPose( control_marker_name, pose );
    }

    player_contexts_[player].pos = pose.position.y;
    player_contexts_[player].active = feedback->dragging;

    interface_.setPose( display_marker_name, pose );

    switch ( feedback->event_type )
    {
      case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
        break;

      default:
        break;
    }
  }

  interactive_markers::InteractiveMarkerInterface interface_;

  ros::Timer game_loop_timer_;

  InteractiveMarker field_marker_;

  struct PlayerContext
  {
    PlayerContext(): pos(0),active(false),score(0) {}
    float pos;
    bool active;
    int score;
  };

  std::vector<PlayerContext> player_contexts_;

  float last_ball_pos_x_;
  float ball_pos_x_;
  float ball_pos_y_;

  int ball_vel_x_;
  int ball_vel_y_;
  float speed_;
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "pong");

  PongGame pong_game;
  ros::spin();
  ROS_INFO("Exiting..");
}
