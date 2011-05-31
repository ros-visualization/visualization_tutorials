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

#include <math.h>

#include <LinearMath/btVector3.h>


using namespace visualization_msgs;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

std::vector< btVector3 > positions;

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    {
      /*
      server->setPose( feedback->marker_name, pose );
      btVector3 pos0(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
      btVector3 pos0(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);

      for ( int count=0; count<positions.size(); count++ )
      {
        geometry_msgs::Pose pose = feedback->pose;

        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;

        positions[count] = btVector3(x,y,z);

        std::stringstream s;
        s << count;
        int_marker.name = s.str();
      }
      */
      break;
    }
  }
  server->publishUpdate();
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  control.independent_marker_orientation = true;

  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale;
  marker.scale.y = msg.scale;
  marker.scale.z = msg.scale;
  marker.color.r = 0.65+0.7*msg.pose.position.x;
  marker.color.g = 0.65+0.7*msg.pose.position.y;
  marker.color.b = 0.65+0.7*msg.pose.position.z;
  marker.color.a = 1.0;

  control.markers.push_back( marker );
  msg.controls.push_back( control );

  return msg.controls.back();
}


void makeCube( )
{
  int side_length = 10;
  float step = 1.0/ (float)side_length;
  int count = 0;

  positions.reserve( side_length*side_length*side_length );

  for ( double x=-0.5; x<0.5; x+=step )
  {
    for ( double y=-0.5; y<0.5; y+=step )
    {
      for ( double z=-0.5; z<0.5; z+=step )
      {
        InteractiveMarker int_marker;
        int_marker.header.frame_id = "/base_link";
        int_marker.scale = step;

        int_marker.pose.position.x = x;
        int_marker.pose.position.y = y;
        int_marker.pose.position.z = z;

        positions.push_back( btVector3(x,y,z) );

        std::stringstream s;
        s << count;
        int_marker.name = s.str();

        makeBoxControl(int_marker);

        server->insert( int_marker );
        server->setCallback( int_marker.name, &processFeedback );

        count++;
      }
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cube");

  server.reset( new interactive_markers::InteractiveMarkerServer("cube") );

  ros::Duration(0.1).sleep();

  ROS_INFO("initializing..");
  makeCube();
  server->publishUpdate();
  ROS_INFO("ready.");

  ros::spin();

  server.reset();
}
