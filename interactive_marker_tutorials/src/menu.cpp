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
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>

using namespace visualization_msgs;
using namespace interactive_markers;

boost::shared_ptr<InteractiveMarkerServer> server;
float marker_pos = 0;

MenuHandler menu_handler;

MenuHandler::EntryHandle h_first_entry;
MenuHandler::EntryHandle h_mode_last;


void enableCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  MenuHandler::EntryHandle handle = feedback->menu_entry_id;
  MenuHandler::CheckState state;
  menu_handler.getCheckState( handle, state );

  if ( state == MenuHandler::CHECKED )
  {
    menu_handler.setCheckState( handle, MenuHandler::UNCHECKED );
    ROS_INFO("Hiding first menu entry");
    menu_handler.setVisible( h_first_entry, false );
  }
  else
  {
    menu_handler.setCheckState( handle, MenuHandler::CHECKED );
    ROS_INFO("Showing first menu entry");
    menu_handler.setVisible( h_first_entry, true );
  }
  menu_handler.reApply( *server );
  ros::Duration(2.0).sleep();
  ROS_INFO("update");
  server->applyChanges();
}

void modeCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  menu_handler.setCheckState( h_mode_last, MenuHandler::UNCHECKED );
  h_mode_last = feedback->menu_entry_id;
  menu_handler.setCheckState( h_mode_last, MenuHandler::CHECKED );

  ROS_INFO("Switching to menu entry #%d", h_mode_last);

  menu_handler.reApply( *server );
  server->applyChanges();
}



Marker makeBox( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
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
  int_marker.header.frame_id = "base_link";
  int_marker.pose.position.y = -3.0 * marker_pos++;;
  int_marker.scale = 1;

  return int_marker;
}

void makeMenuMarker( std::string name )
{
  InteractiveMarker int_marker = makeEmptyMarker();
  int_marker.name = name;

  InteractiveMarkerControl control;

  control.interaction_mode = InteractiveMarkerControl::BUTTON;
  control.always_visible = true;

  control.markers.push_back( makeBox( int_marker ) );
  int_marker.controls.push_back(control);

  server->insert( int_marker );
}

void deepCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO("The deep sub-menu has been found.");
}

void initMenu()
{
  h_first_entry = menu_handler.insert( "First Entry" );
  MenuHandler::EntryHandle entry = menu_handler.insert( h_first_entry, "deep" );
  entry = menu_handler.insert( entry, "sub" );
  entry = menu_handler.insert( entry, "menu", &deepCb );
  
  menu_handler.setCheckState( menu_handler.insert( "Show First Entry", &enableCb ), MenuHandler::CHECKED );

  MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert( "Switch" );

  for ( int i=0; i<5; i++ )
  {
    std::ostringstream s;
    s << "Mode " << i;
    h_mode_last = menu_handler.insert( sub_menu_handle, s.str(), &modeCb );
    menu_handler.setCheckState( h_mode_last, MenuHandler::UNCHECKED );
  }
  //check the very last entry
  menu_handler.setCheckState( h_mode_last, MenuHandler::CHECKED );
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "menu");

  server.reset( new InteractiveMarkerServer("menu","",false) );

  initMenu();

  makeMenuMarker( "marker1" );
  makeMenuMarker( "marker2" );

  menu_handler.apply( *server, "marker1" );
  menu_handler.apply( *server, "marker2" );
  server->applyChanges();

  ros::spin();

  server.reset();
}
