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

#include <interactive_markers/interactive_marker_server.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>

using namespace visualization_msgs;

interactive_markers::InteractiveMarkerServer *server;
float marker_pos = 0;


void frameCallback(const ros::TimerEvent&)
{
  static uint32_t counter = 0;

  static tf::TransformBroadcaster br;

  tf::Transform t;

  t.setOrigin(tf::Vector3(0.0, 0.0, sin(float(counter)/140.0) * 2.0));
  t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "base_link", "moving_frame"));

  t.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  t.setRotation(tf::createQuaternionFromRPY(0.0, float(counter)/100.0, 0.0));
  br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "base_link", "rotating_frame"));

  ++counter;
}

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM( feedback->marker_name << " was clicked on." );
      break;
    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      if ( feedback->selected_menu_entry.size() == 1 )
      {
        ROS_INFO_STREAM( feedback->marker_name << ": selected menu entry "
            << feedback->selected_menu_entry[0] );
      }
      else if ( feedback->selected_menu_entry.size() == 2 )
      {
        ROS_INFO_STREAM( feedback->marker_name << ": selected menu entry "
            << feedback->selected_menu_entry[0] << "/"
            << feedback->selected_menu_entry[1] );
      }
      else
      {
        ROS_INFO_STREAM( feedback->marker_name << ": invalid feedback!" );
      }
      break;
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      ROS_INFO_STREAM( feedback->marker_name << ":"
          << " position = "
          << feedback->pose.position.x
          << ", " << feedback->pose.position.y
          << ", " << feedback->pose.position.z
          << " orientation = "
          << feedback->pose.orientation.w
          << ", " << feedback->pose.orientation.x
          << ", " << feedback->pose.orientation.y
          << ", " << feedback->pose.orientation.z );
      break;
  }
  server->publishUpdate();
}

void alignMarker( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      geometry_msgs::Pose pose = feedback->pose;
      pose.position.x = round(pose.position.x);
      pose.position.y = round(pose.position.y);
      pose.position.z = round(pose.position.z);
      ROS_INFO_STREAM( feedback->marker_name << ":"
          << " aligning position = "
          << feedback->pose.position.x
          << ", " << feedback->pose.position.y
          << ", " << feedback->pose.position.z
          << " to "
          << pose.position.x
          << ", " << pose.position.y
          << ", " << pose.position.z );
      server->setPose( feedback->marker_name, pose );
      break;
  }
  server->publishUpdate();
}

double rand( double min, double max )
{
  double t = (double)rand() / (double)RAND_MAX;
  return min + t*(max-min);
}

Marker makeBox( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

InteractiveMarker makeEmptyMarker( bool dummyBox=true )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "/base_link";
  int_marker.pose.position.y = -3.0 * marker_pos++;;
  int_marker.frame_locked = true;
  int_marker.scale = 1;

  return int_marker;
}

void saveMarker( InteractiveMarker int_marker )
{
  server->insert(int_marker, &processFeedback);
}

////////////////////////////////////////////////////////////////////////////////////


void make6DofMarker( bool fixed )
{
  InteractiveMarker int_marker = makeEmptyMarker();

  int_marker.name = "simple_6dof";
  int_marker.description = "Simple 6-DOF Control";

  int_marker.scale = 1.0;

  makeBoxControl(int_marker);

  InteractiveMarkerControl control;

  if ( fixed )
  {
    int_marker.name += "_fixed";
    int_marker.description += "\n(fixed orientation)";
    control.orientation_mode = InteractiveMarkerControl::FIXED;
  }

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  saveMarker( int_marker );
}

void makeRandomDofMarker( )
{
  InteractiveMarker int_marker = makeEmptyMarker();
  int_marker.name = "6dof_random_axes";
  int_marker.description = "6-DOF\n(Arbitrary Axes)";

  makeBoxControl(int_marker);

  InteractiveMarkerControl control;

  for ( int i=0; i<3; i++ )
  {
    control.orientation.w = rand(-1,1);
    control.orientation.x = rand(-1,1);
    control.orientation.y = rand(-1,1);
    control.orientation.z = rand(-1,1);
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  saveMarker( int_marker );
}


void makeViewFacingMarker( )
{
  InteractiveMarker int_marker = makeEmptyMarker();
  int_marker.name = "view_facing";
  int_marker.description = "View Facing 6-DOF";

  InteractiveMarkerControl control;

  // make a control that rotates around the view axis
  control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation.w = 1;

  int_marker.controls.push_back(control);

  // create a box in the center which should not be view facing,
  // but move in the camera plane.
  control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  control.independent_marker_orientation = true;

  control.markers.push_back( makeBox(int_marker) );
  control.always_visible = true;

  int_marker.controls.push_back(control);


  saveMarker( int_marker );
}


void makeQuadrocopterMarker( )
{
  InteractiveMarker int_marker = makeEmptyMarker();
  int_marker.name = "quadrocopter";
  int_marker.description = "Quadrocopter Mode\n(Dog Walk + Elevation)";

  makeBoxControl(int_marker);

  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
  int_marker.controls.push_back(control);
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  saveMarker( int_marker );
}

void makeChessPieceMarker( )
{
  InteractiveMarker int_marker = makeEmptyMarker();
  int_marker.name = "chess_piece";
  int_marker.description = "Chess Piece\n(2D Move + Alignment)";

  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  int_marker.controls.push_back(control);

  // make a box which also moves in the plane
  control.markers.push_back( makeBox(int_marker) );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  // we want to use our special callback function
  server->insert(int_marker, &alignMarker);
}

void makePanTiltMarker( )
{
  InteractiveMarker int_marker = makeEmptyMarker();
  int_marker.name = "pan_tilt";
  int_marker.description = "Pan / Tilt";

  makeBoxControl(int_marker);

  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation_mode = InteractiveMarkerControl::FIXED;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation_mode = InteractiveMarkerControl::INHERIT;
  int_marker.controls.push_back(control);

  saveMarker( int_marker );
}

void makeMenuMarker()
{
  InteractiveMarker int_marker = makeEmptyMarker();
  int_marker.name = "context_menu";
  int_marker.description = "Context Menu\n(Right Click)";

  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  control.interaction_mode = InteractiveMarkerControl::MENU;
  control.always_visible = true;
  control.markers.push_back( makeBox(int_marker) );
  int_marker.controls.push_back(control);

  visualization_msgs::Menu menu;

  menu.title = "First Entry";
  int_marker.menu.push_back( menu );

  menu.title = "Second Entry";
  int_marker.menu.push_back( menu );

  menu.title = "Submenu";
  menu.entries.push_back("First Submenu Entry");
  menu.entries.push_back("Second Submenu Entry");
  int_marker.menu.push_back( menu );

  saveMarker( int_marker );
}


void makeMovingMarker()
{
  InteractiveMarker int_marker = makeEmptyMarker();
  int_marker.header.frame_id = "/moving_frame";
  int_marker.name = "moving";
  int_marker.description = "Marker Attached to a\nMoving Frame";

  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  control.interaction_mode = InteractiveMarkerControl::MENU;
  control.always_visible = true;
  control.markers.push_back( makeBox(int_marker) );
  int_marker.controls.push_back(control);

  visualization_msgs::Menu menu;

  menu.title = "First Entry";
  int_marker.menu.push_back( menu );

  menu.title = "Second Entry";
  int_marker.menu.push_back( menu );

  menu.title = "Submenu";
  menu.entries.push_back("First Submenu Entry");
  menu.entries.push_back("Second Submenu Entry");
  int_marker.menu.push_back( menu );

  saveMarker( int_marker );
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "basic_controls");
  ros::NodeHandle n;

  // create a timer to update the published transforms
  ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);

  server = new interactive_markers::InteractiveMarkerServer("basic_controls");

  ros::Duration(0.1).sleep();

  make6DofMarker( false );
  make6DofMarker( true );
  makeRandomDofMarker( );
  makeViewFacingMarker( );
  makeQuadrocopterMarker( );
  makeChessPieceMarker( );
  makePanTiltMarker( );
  makeMenuMarker( );
  makeMovingMarker( );

  server->publishUpdate();

  ros::spin();
}
