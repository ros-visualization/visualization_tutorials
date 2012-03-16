/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/property.h>
#include <rviz/properties/property_manager.h>
#include <rviz/frame_manager.h>
#include <rviz/validate_floats.h>
#include <rviz/ogre_helpers/shape.h>

#include "covariance_ellipsoids_display.h"

namespace rviz_plugin_tutorials
{

CovarianceEllipsoidDisplay::CovarianceEllipsoidDisplay()
  : Display()
  , color_( .8, .2, .8 )
  , alpha_( 1.0 )
{
}

void CovarianceEllipsoidDisplay::onInitialize()
{
  tf_filter_ = new tf::MessageFilter<sensor_msgs::Range>(*vis_manager_->getTFClient(), "", 10, update_nh_);

  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  scene_node_->setVisible( false );
  
  setBuffer( 1 );
  Ogre::Vector3 scale( 0, 0, 0);

  tf_filter_->connectInput(sub_);
  tf_filter_->registerCallback(boost::bind(&CovarianceEllipsoidDisplay::incomingMessage, this, _1));
  vis_manager_->getFrameManager()->registerFilterForTransformStatusCheck(tf_filter_, this);
  setAlpha( 0.5f );
}

CovarianceEllipsoidDisplay::~CovarianceEllipsoidDisplay()
{
  unsubscribe();
  clear();
  for (size_t i = 0; i < cones_.size(); i++) {
    delete cones_[i];
  }

  delete tf_filter_;
}

void CovarianceEllipsoidDisplay::clear()
{
  setBuffer( cones_.size() );
  tf_filter_->clear();
  messages_received_ = 0;
  setStatus(rviz::status_levels::Warn, "Topic", "No messages received");
}

void CovarianceEllipsoidDisplay::setTopic( const std::string& topic )
{
  unsubscribe();

  topic_ = topic;

  subscribe();

  propertyChanged(topic_property_);

  causeRender();
}

void CovarianceEllipsoidDisplay::setColor( const rviz::Color& color )
{
  color_ = color;

  propertyChanged(color_property_);

  processMessage(current_message_);
  causeRender();
}

void CovarianceEllipsoidDisplay::setBuffer( int buffer )
{
  if(buffer < 1)
    buffer = 1;
  buffer_len_ = buffer;

  propertyChanged(bufferLen_property_);
  
  for (size_t i = 0; i < cones_.size(); i++) {
    delete cones_[i];
  }
  cones_.resize(buffer_len_);
  for (size_t i = 0; i < cones_.size(); i++) {
    cones_[i] = new Shape(Shape::Cone, vis_manager_->getSceneManager(), scene_node_);
    Shape* cone = cones_[i];
    
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    geometry_msgs::Pose pose;
    pose.position.z = pose.position.y = 0;
    pose.position.x = 0;
    pose.orientation.x = 0;
    pose.orientation.z = 0;
    cone->setPosition(position);
    cone->setOrientation(orientation); 
    Ogre::Vector3 scale( 0, 0, 0);
    cone->setScale(scale);
    cone->setColor(color_.r_, color_.g_, color_.b_, 0);
    
  }
  
}

void CovarianceEllipsoidDisplay::setAlpha( float alpha )
{
  alpha_ = alpha;

  propertyChanged(alpha_property_);

  processMessage(current_message_);
  causeRender();
}

void CovarianceEllipsoidDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  try
  {
    sub_.subscribe(update_nh_, topic_, 10);
    setStatus(status_levels::Ok, "Topic", "OK");
  }
  catch (ros::Exception& e)
  {
    setStatus(status_levels::Error, "Topic", std::string("Error subscribing: ") + e.what());
  }
}

void CovarianceEllipsoidDisplay::unsubscribe()
{
  sub_.unsubscribe();
}

void CovarianceEllipsoidDisplay::onEnable()
{
  scene_node_->setVisible( true );
  subscribe();
}

void CovarianceEllipsoidDisplay::onDisable()
{
  unsubscribe();
  clear();
  scene_node_->setVisible( false );
}

void CovarianceEllipsoidDisplay::fixedFrameChanged()
{
  tf_filter_->setTargetFrame( fixed_frame_ );
  clear();
}

void CovarianceEllipsoidDisplay::update(float wall_dt, float ros_dt)
{
}


void CovarianceEllipsoidDisplay::processMessage(const sensor_msgs::Range::ConstPtr& msg)
{
  if (!msg)
  {
    return;
  }

  ++messages_received_;
  
  Shape* cone = cones_[messages_received_ % buffer_len_];

  {
    std::stringstream ss;
    ss << messages_received_ << " messages received";
    setStatus(rviz::status_levels::Ok, "Topic", ss.str());
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  geometry_msgs::Pose pose;
  pose.position.z = pose.position.y = 0;
  pose.position.x = msg->range/2 - .008824 * msg->range; // .008824 fudge factor measured, must be inaccuracy of cone model.
  pose.orientation.z = 0.707;
  pose.orientation.w = 0.707;
  if( !vis_manager_->getFrameManager()->transform( msg->header.frame_id, msg->header.stamp, pose, position, orientation ))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), fixed_frame_.c_str() );
  }

  cone->setPosition(position);
  cone->setOrientation(orientation); 

  double cone_width = 2.0 * msg->range * tan( msg->field_of_view / 2.0 );
  Ogre::Vector3 scale( cone_width, msg->range, cone_width );
  cone->setScale(scale);
  cone->setColor(color_.r_, color_.g_, color_.b_, alpha_);

}

void CovarianceEllipsoidDisplay::incomingMessage(const sensor_msgs::Range::ConstPtr& msg)
{
  processMessage(msg);
}

void CovarianceEllipsoidDisplay::reset()
{
  Display::reset();
  clear();
}

void CovarianceEllipsoidDisplay::createProperties()
{
  topic_property_ = property_manager_->createProperty<rviz::ROSTopicStringProperty>( "Topic", property_prefix_, boost::bind( &CovarianceEllipsoidDisplay::getTopic, this ),
                                                                                boost::bind( &CovarianceEllipsoidDisplay::setTopic, this, _1 ), parent_category_, this );
  setPropertyHelpText(topic_property_, "sensor_msgs::Range topic to subscribe to.");
  rviz::ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(ros::message_traits::datatype<sensor_msgs::Range>());
  color_property_ = property_manager_->createProperty<rviz::ColorProperty>( "Color", property_prefix_, boost::bind( &CovarianceEllipsoidDisplay::getColor, this ),
                                                                      boost::bind( &CovarianceEllipsoidDisplay::setColor, this, _1 ), parent_category_, this );
  setPropertyHelpText(color_property_, "Color to draw the range.");
  alpha_property_ = property_manager_->createProperty<rviz::FloatProperty>( "Alpha", property_prefix_, boost::bind( &CovarianceEllipsoidDisplay::getAlpha, this ),
                                                                       boost::bind( &CovarianceEllipsoidDisplay::setAlpha, this, _1 ), parent_category_, this );
  setPropertyHelpText(alpha_property_, "Amount of transparency to apply to the range.");
  bufferLen_property_ = property_manager_->createProperty<rviz::IntProperty>( "Buffer Length", property_prefix_, boost::bind( &CovarianceEllipsoidDisplay::getBuffer, this ),
                                                                       boost::bind( &CovarianceEllipsoidDisplay::setBuffer, this, _1 ), parent_category_, this );
  setPropertyHelpText(bufferLen_property_, "Number of prior measurements to display.");
  
}

const char* CovarianceEllipsoidDisplay::getDescription()
{
  return "Displays position, orientation, and position uncertainty from a"
    " geometry_msgs::PoseWithCovarianceStamped message as an ellipsoid and an axis.";
}

} // end namespace rviz_plugin_tutorials
