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
#include <ros/ros.h>
#include <stdlib.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/tools.h>

#include <tf/LinearMath/Vector3.h>
#include <tf/tf.h>

bool testPointAgainstAabb2(const tf::Vector3 &aabbMin1, const tf::Vector3 &aabbMax1,
                           const tf::Vector3 &point)
{
	bool overlap = true;
	overlap = (aabbMin1.getX() > point.getX() || aabbMax1.getX() < point.getX()) ? false : overlap;
	overlap = (aabbMin1.getZ() > point.getZ() || aabbMax1.getZ() < point.getZ()) ? false : overlap;
	overlap = (aabbMin1.getY() > point.getY() || aabbMax1.getY() < point.getY()) ? false : overlap;
	return overlap;
}

namespace vm = visualization_msgs;

class PointCouldSelector
{
public:
	PointCouldSelector( boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
	    std::vector<tf::Vector3>& points ) :
	      server_( server ),
        min_sel_( -1, -1, -1 ),
        max_sel_( 1, 1, 1 ),
        points_( points )
	{
	  updateBox( );
	  updatePointClouds();

	  makeSizeHandles();
	}

	void processAxisFeedback(
	    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
	{
	  ROS_INFO_STREAM( feedback->marker_name << " is now at "
	      << feedback->pose.position.x << ", " << feedback->pose.position.y
	      << ", " << feedback->pose.position.z );

    if ( feedback->marker_name == "min_x" ) min_sel_.setX( feedback->pose.position.x );
    if ( feedback->marker_name == "max_x" ) max_sel_.setX( feedback->pose.position.x );
    if ( feedback->marker_name == "min_y" ) min_sel_.setY( feedback->pose.position.y );
    if ( feedback->marker_name == "max_y" ) max_sel_.setY( feedback->pose.position.y );
    if ( feedback->marker_name == "min_z" ) min_sel_.setZ( feedback->pose.position.z );
    if ( feedback->marker_name == "max_z" ) max_sel_.setZ( feedback->pose.position.z );

    updateBox( );
    updateSizeHandles();

    if ( feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP )
    {
      updatePointClouds();
    }

    server_->applyChanges();
	}

	vm::Marker makeBox( vm::InteractiveMarker &msg,
	    tf::Vector3 min_bound, tf::Vector3 max_bound )
	{
	  vm::Marker marker;

	  marker.type = vm::Marker::CUBE;
	  marker.scale.x = max_bound.x() - min_bound.x();
	  marker.scale.y = max_bound.y() - min_bound.y();
	  marker.scale.z = max_bound.z() - min_bound.z();
    marker.pose.position.x = 0.5 * ( max_bound.x() + min_bound.x() );
    marker.pose.position.y = 0.5 * ( max_bound.y() + min_bound.y() );
    marker.pose.position.z = 0.5 * ( max_bound.z() + min_bound.z() );
	  marker.color.r = 0.5;
	  marker.color.g = 0.5;
	  marker.color.b = 0.5;
	  marker.color.a = 0.5;

	  return marker;
	}

	void updateBox( )
	{
	  vm::InteractiveMarker msg;
	  msg.header.frame_id = "base_link";

	  vm::InteractiveMarkerControl control;
	  control.always_visible = false;
	  control.markers.push_back( makeBox(msg, min_sel_, max_sel_) );
	  msg.controls.push_back( control );

	  server_->insert( msg );
	}

	void updatePointCloud( std::string name, std_msgs::ColorRGBA color, std::vector<tf::Vector3> &points )
	{
	  // create an interactive marker for our server
	  vm::InteractiveMarker int_marker;
	  int_marker.header.frame_id = "base_link";
	  int_marker.name = name;

	  // create a point cloud marker
	  vm::Marker points_marker;
	  points_marker.type = vm::Marker::SPHERE_LIST;
	  points_marker.scale.x = 0.05;
	  points_marker.scale.y = 0.05;
	  points_marker.scale.z = 0.05;
	  points_marker.color = color;

	  for ( unsigned i=0; i<points.size(); i++ )
	  {
	    geometry_msgs::Point p;
      p.x = points[i].x();
      p.y = points[i].y();
      p.z = points[i].z();
	    points_marker.points.push_back( p );
	  }

	  // create container control
	  vm::InteractiveMarkerControl points_control;
	  points_control.always_visible = true;
	  points_control.interaction_mode = vm::InteractiveMarkerControl::NONE;
	  points_control.markers.push_back( points_marker );

	  // add the control to the interactive marker
	  int_marker.controls.push_back( points_control );

	  server_->insert( int_marker );
	}

	void updatePointClouds()
	{
	  std::vector<tf::Vector3> points_in, points_out;
    points_in.reserve( points_.size() );
    points_out.reserve( points_.size() );

    // determine which points are selected (i.e. inside the selection box)
	  for ( unsigned i=0; i<points_.size(); i++ )
	  {
	    if ( testPointAgainstAabb2( min_sel_, max_sel_, points_[i] ) )
	    {
	      points_in.push_back( points_[i] );
	    }
	    else
	    {
        points_out.push_back( points_[i] );
	    }
	  }

    std_msgs::ColorRGBA in_color;
    in_color.r = 1.0;
    in_color.g = 0.8;
    in_color.b = 0.0;
    in_color.a = 1.0;

    std_msgs::ColorRGBA out_color;
    out_color.r = 0.5;
    out_color.g = 0.5;
    out_color.b = 0.5;
    out_color.a = 1.0;

    updatePointCloud( "selected_points", in_color, points_in );
    updatePointCloud( "unselected_points", out_color, points_out );
	}

  void makeSizeHandles( )
  {
    for ( int axis=0; axis<3; axis++ )
    {
      for ( int sign=-1; sign<=1; sign+=2 )
      {
        vm::InteractiveMarker int_marker;
        int_marker.header.frame_id = "base_link";
        int_marker.scale = 1.0;

        vm::InteractiveMarkerControl control;
        control.interaction_mode = vm::InteractiveMarkerControl::MOVE_AXIS;
        control.orientation_mode = vm::InteractiveMarkerControl::INHERIT;
        control.always_visible = false;

        tf::Quaternion orien;

        switch ( axis )
        {
        case 0:
          int_marker.name = sign>0 ? "max_x" : "min_x";
          int_marker.pose.position.x = sign>0 ? max_sel_.x() : min_sel_.x();
          int_marker.pose.position.y = 0.5 * ( max_sel_.y() + min_sel_.y() );
          int_marker.pose.position.z = 0.5 * ( max_sel_.z() + min_sel_.z() );
          orien = tf::Quaternion(1.0, 0.0, 0.0, 1.0);
          orien.normalize();
          tf::quaternionTFToMsg(orien, control.orientation);
          break;
        case 1:
          int_marker.name = sign>0 ? "max_y" : "min_y";
          int_marker.pose.position.x = 0.5 * ( max_sel_.x() + min_sel_.x() );
          int_marker.pose.position.y = sign>0 ? max_sel_.y() : min_sel_.y();
          int_marker.pose.position.z = 0.5 * ( max_sel_.z() + min_sel_.z() );
          orien = tf::Quaternion(0.0, 0.0, 1.0, 1.0);
          orien.normalize();
          tf::quaternionTFToMsg(orien, control.orientation);
          break;
        default:
          int_marker.name = sign>0 ? "max_z" : "min_z";
          int_marker.pose.position.x = 0.5 * ( max_sel_.x() + min_sel_.x() );
          int_marker.pose.position.y = 0.5 * ( max_sel_.y() + min_sel_.y() );
          int_marker.pose.position.z = sign>0 ? max_sel_.z() : min_sel_.z();
          orien = tf::Quaternion(0.0, -1.0, 0.0, 1.0);
          orien.normalize();
          tf::quaternionTFToMsg(orien, control.orientation);
          break;
        }

        interactive_markers::makeArrow( int_marker, control, 0.5 * sign );

        int_marker.controls.push_back( control );
        server_->insert( int_marker, boost::bind( &PointCouldSelector::processAxisFeedback, this, _1 ) );
      }
    }
  }

  void updateSizeHandles( )
  {
    for ( int axis=0; axis<3; axis++ )
    {
      for ( int sign=-1; sign<=1; sign+=2 )
      {
        std::string name;
        geometry_msgs::Pose pose;

        switch ( axis )
        {
        case 0:
          name = sign>0 ? "max_x" : "min_x";
          pose.position.x = sign>0 ? max_sel_.x() : min_sel_.x();
          pose.position.y = 0.5 * ( max_sel_.y() + min_sel_.y() );
          pose.position.z = 0.5 * ( max_sel_.z() + min_sel_.z() );
          break;
        case 1:
          name = sign>0 ? "max_y" : "min_y";
          pose.position.x = 0.5 * ( max_sel_.x() + min_sel_.x() );
          pose.position.y = sign>0 ? max_sel_.y() : min_sel_.y();
          pose.position.z = 0.5 * ( max_sel_.z() + min_sel_.z() );
          break;
        default:
          name = sign>0 ? "max_z" : "min_z";
          pose.position.x = 0.5 * ( max_sel_.x() + min_sel_.x() );
          pose.position.y = 0.5 * ( max_sel_.y() + min_sel_.y() );
          pose.position.z = sign>0 ? max_sel_.z() : min_sel_.z();
          break;
        }

        server_->setPose( name, pose );
      }
    }
  }

private:
	boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

	tf::Vector3 min_sel_, max_sel_;
	std::vector<tf::Vector3> points_;

	vm::InteractiveMarker sel_points_marker_;
	vm::InteractiveMarker unsel_points_marker_;
};




double rand( double min, double max )
{
  double t = (double)rand() / (double)RAND_MAX;
  return min + t*(max-min);
}


void makePoints( std::vector<tf::Vector3>& points_out, int num_points )
{
  double radius = 3;
  double scale = 0.2;
  points_out.resize(num_points);
  for( int i = 0; i < num_points; i++ )
  {
    points_out[i].setX( scale * rand( -radius, radius ) );
	  points_out[i].setY( scale * rand( -radius, radius ) );
    points_out[i].setZ( scale * radius * 0.2 * ( sin( 10.0 / radius * points_out[i].x() ) + cos( 10.0 / radius * points_out[i].y() ) ) );
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "selection");

  // create an interactive marker server on the topic namespace simple_marker
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server(
      new interactive_markers::InteractiveMarkerServer("selection") );

  std::vector<tf::Vector3> points;
  makePoints( points, 10000 );

  PointCouldSelector selector( server, points );

  // 'commit' changes and send to all clients
  server->applyChanges();

  // start the ROS main loop
  ros::spin();
}
// %Tag(fullSource)%
