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


// %Tag(fullSource)%
#include <math.h>

#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>

namespace vm = visualization_msgs;

void processFeedback( const vm::InteractiveMarkerFeedbackConstPtr &feedback )
{
  uint8_t type = feedback->event_type;

  if( type == vm::InteractiveMarkerFeedback::BUTTON_CLICK ||
      type == vm::InteractiveMarkerFeedback::MOUSE_DOWN ||
      type == vm::InteractiveMarkerFeedback::MOUSE_UP )
  {
    const char* type_str = (type == vm::InteractiveMarkerFeedback::BUTTON_CLICK ? "button click" :
                            (type == vm::InteractiveMarkerFeedback::MOUSE_DOWN ? "mouse down" : "mouse up"));

    if( feedback->mouse_point_valid )
    {
      ROS_INFO( "%s at %f, %f, %f in frame %s",
                type_str,
                feedback->mouse_point.x, feedback->mouse_point.y, feedback->mouse_point.z,
                feedback->header.frame_id.c_str() );
    }
    else
    {
      ROS_INFO( "%s", type_str );
    }
  }
  else if( type == vm::InteractiveMarkerFeedback::POSE_UPDATE )
  {
    ROS_INFO_STREAM( feedback->marker_name << " is now at "
                     << feedback->pose.position.x << ", " << feedback->pose.position.y
                     << ", " << feedback->pose.position.z );
  }
}

void makePoints( std::vector<geometry_msgs::Point>& points_out, int num_points )
{
  double radius = 3;
  points_out.resize(num_points);
  for( int i = 0; i < num_points; i++ )
  {
    double angle = (i / (double) num_points * 50 * M_PI);
    double height = (i / (double) num_points * 10);
    points_out[i].x = radius * cos( angle );
    points_out[i].y = radius * sin( angle );
    points_out[i].z = height;
  }
}

vm::InteractiveMarker makeMarker( std::string name, std::string description, int32_t type, float x, int num_points = 10000, float scale = 0.1f )
{
  // create an interactive marker for our server
  vm::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.name = name;
  int_marker.description = description;

  // create a point cloud marker
  vm::Marker points_marker;
  points_marker.type = type;
  points_marker.scale.x = scale;
  points_marker.scale.y = scale;
  points_marker.scale.z = scale;
  points_marker.color.r = 0.5;
  points_marker.color.g = 0.5;
  points_marker.color.b = 0.5;
  points_marker.color.a = 1.0;
  makePoints( points_marker.points, num_points );

  // create a control which contains the point cloud which acts like a button.
  vm::InteractiveMarkerControl points_control;
  points_control.always_visible = true;
  points_control.interaction_mode = vm::InteractiveMarkerControl::BUTTON;
  points_control.markers.push_back( points_marker );

  // add the control to the interactive marker
  int_marker.controls.push_back( points_control );

  // create a control which will move the box
  // this control does not contain any markers,
  // which will cause RViz to insert two arrows
  vm::InteractiveMarkerControl rotate_control;
  rotate_control.name = "move_x";
  rotate_control.interaction_mode =
      vm::InteractiveMarkerControl::MOVE_AXIS;

  // add the control to the interactive marker
  int_marker.controls.push_back(rotate_control);

  int_marker.pose.position.x = x;

  return int_marker;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "point_cloud");

  // create an interactive marker server on the topic namespace simple_marker
  interactive_markers::InteractiveMarkerServer server("point_cloud");

  server.insert(makeMarker("points", "Points marker", vm::Marker::POINTS, 0), &processFeedback);
  // LINE_STRIP and LINE_LIST are not actually selectable, and they won't highlight or detect mouse clicks like the others (yet).
  server.insert(makeMarker("line_strip", "Line Strip marker", vm::Marker::LINE_STRIP, 10, 1000), &processFeedback);
  server.insert(makeMarker("line_list", "Line List marker", vm::Marker::LINE_LIST, 20), &processFeedback);
  server.insert(makeMarker("cube_list", "Cube List marker", vm::Marker::CUBE_LIST, 30), &processFeedback);
  server.insert(makeMarker("sphere_list", "Sphere List marker", vm::Marker::SPHERE_LIST, 40), &processFeedback);
  server.insert(makeMarker("triangle_list", "Triangle List marker", vm::Marker::TRIANGLE_LIST, 50, 201, 1.0f), &processFeedback);

  // 'commit' changes and send to all clients
  server.applyChanges();

  // start the ROS main loop
  ros::spin();
}
// %Tag(fullSource)%
