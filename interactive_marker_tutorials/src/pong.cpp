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

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

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
  interface_("pong", false)
  {
    makeMarkers();
    player_pos_.resize(2);
  }

private:

  void makeMarkers()
  {
    InteractiveMarker int_marker;
    int_marker.name = "field";
    int_marker.header.frame_id = "/base_link";

    InteractiveMarkerControl control;
    control.always_visible = true;

    Marker marker;
    marker.type = Marker::CUBE;
    marker.color.r = 0.7;
    marker.color.g = 0.7;
    marker.color.b = 0.7;
    marker.color.a = 1.0;

    // Left Border
    marker.scale.x = BORDER_SIZE;
    marker.scale.y = FIELD_WIDTH + 5.0 * BORDER_SIZE;
    marker.scale.z = BORDER_SIZE;
    marker.pose.position.z = FIELD_HEIGHT*0.5 + 2.0*BORDER_SIZE;
    control.markers.push_back( marker );

    // Right Border
    marker.pose.position.z *= -1;
    control.markers.push_back( marker );

    // Top Border
    marker.scale.x = BORDER_SIZE;
    marker.scale.y = BORDER_SIZE;
    marker.scale.z = FIELD_HEIGHT + 5.0 * BORDER_SIZE;
    marker.pose.position.z = 0;
    marker.pose.position.y = FIELD_WIDTH*0.5 + 2.0*BORDER_SIZE;
    control.markers.push_back( marker );

    // Bottom Border
    marker.pose.position.y *= -1;
    control.markers.push_back( marker );

    int_marker.controls.push_back( control );

    field_marker_ = int_marker;
    interface_.insert( int_marker );

    player_marker_.reserve(2);

    // Player 1
    int_marker.name = "player_1";
    int_marker.pose.position.y = -1.0 * (FIELD_WIDTH + BORDER_SIZE) * 0.5;
    int_marker.controls.clear();

    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    control.orientation.w = 1;
    control.orientation.y = 1;
    control.markers.clear();

    marker.scale.x = BORDER_SIZE;
    marker.scale.y = BORDER_SIZE;
    marker.scale.z = PADDLE_SIZE;
    marker.pose.position.z = 0;
    marker.pose.position.y = 0;
    control.markers.push_back( marker );

    marker.pose.position.y *= -1;
    control.markers.push_back( marker );

    int_marker.controls.push_back( control );

    player_marker_.push_back( int_marker );
    interface_.insert( int_marker, boost::bind( &PongGame::playerPosUpdated, this, _1 ) );

    // Player 2
    int_marker.name = "player_2";
    int_marker.pose.position.y *= -1;

    player_marker_.push_back( int_marker );
    interface_.insert( int_marker );

    // Ball
    int_marker.name = "Ball";
    int_marker.pose.position.x = 0.0;
    int_marker.pose.position.y = 0.0;
    int_marker.controls.clear();

    control.interaction_mode = InteractiveMarkerControl::NONE;
    control.orientation.w = 1;
    control.orientation.y = 1;
    control.markers.clear();

    marker.type = Marker::CYLINDER;
    marker.scale.x = BORDER_SIZE;
    marker.scale.y = BORDER_SIZE;
    marker.scale.z = BORDER_SIZE;
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.y = 1.0;
    control.markers.push_back( marker );

    int_marker.controls.push_back( control );

    ball_marker_ = int_marker;
    interface_.insert( int_marker );
  }

  void playerPosUpdated( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
  {
    std::string marker_name = feedback->marker_name;
    ROS_INFO( "%s pos is %f", marker_name.c_str(), feedback->pose.position.z );

    geometry_msgs::Pose pose = feedback->pose;

    switch ( feedback->event_type )
    {
      case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
        if ( pose.position.z > FIELD_HEIGHT * 0.5 )
        {
          pose.position.z = FIELD_HEIGHT * 0.5;
          interface_.setPose( marker_name, pose );
        }
        if ( pose.position.z < FIELD_HEIGHT * -0.5 )
        {
          pose.position.z = FIELD_HEIGHT * -0.5;
          interface_.setPose( marker_name, pose );
        }
        break;

      default:
        break;
    }

    if ( marker_name == player_marker_[0].name )
    {
      player_pos_[0] = pose.position.z;
    }
    if ( marker_name == player_marker_[1].name )
    {
      player_pos_[1] = pose.position.z;
    }
  }

  interactive_markers::InteractiveMarkerInterface interface_;

  InteractiveMarker ball_marker_;
  InteractiveMarker field_marker_;
  std::vector<InteractiveMarker> player_marker_;

  std::vector<float> player_pos_;

  float ball_x_;
  float ball_y_;
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "pong");

  PongGame pong_game;

  ros::spin();
}
