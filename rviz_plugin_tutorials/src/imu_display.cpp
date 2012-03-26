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

// %Tag(FULL_SOURCE)%
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/property.h>
#include <rviz/properties/property_manager.h>
#include <rviz/frame_manager.h>

#include "imu_visual.h"

#include "imu_display.h"

namespace rviz_plugin_tutorials
{

// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
ImuDisplay::ImuDisplay()
  : Display()
  , scene_node_( NULL )
  , messages_received_( 0 )
  , color_( .8, .2, .8 )       // Default color is bright purple.
  , alpha_( 1.0 )              // Default alpha is completely opaque.
{
}

// After the parent rviz::Display::initialize() does its own setup, it
// calls the subclass's onInitialize() function.  This is where we
// instantiate all the workings of the class.
void ImuDisplay::onInitialize()
{
  // Make an Ogre::SceneNode to contain all our visuals.
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  
  // Set the default history length and resize the visuals_ array.
  setHistoryLength( 1 );

  // A tf::MessageFilter listens to ROS messages and calls our
  // callback with them when they can be matched up with valid tf
  // transform data.
  tf_filter_ = new tf::MessageFilter<sensor_msgs::Imu>( *vis_manager_->getTFClient(), "", 100, update_nh_ );
  tf_filter_->connectInput( sub_ );
  tf_filter_->registerCallback( boost::bind( &ImuDisplay::incomingMessage, this, _1 ));

  // FrameManager has some built-in functions to set the status of a
  // Display based on callbacks from a tf::MessageFilter.  These work
  // fine for this simple display.
  vis_manager_->getFrameManager()->registerFilterForTransformStatusCheck( tf_filter_, this );
}

ImuDisplay::~ImuDisplay()
{
  unsubscribe();
  clear();
  for( size_t i = 0; i < visuals_.size(); i++ )
  {
    delete visuals_[ i ];
  }

  delete tf_filter_;
}

// Clear the visuals by deleting their objects.
void ImuDisplay::clear()
{
  for( size_t i = 0; i < visuals_.size(); i++ )
  {
    delete visuals_[ i ];
    visuals_[ i ] = NULL;
  }
  tf_filter_->clear();
  messages_received_ = 0;
  setStatus( rviz::status_levels::Warn, "Topic", "No messages received" );
}

void ImuDisplay::setTopic( const std::string& topic )
{
  unsubscribe();
  clear();
  topic_ = topic;
  subscribe();

  // Broadcast the fact that the variable has changed.
  propertyChanged( topic_property_ );

  // Make sure rviz renders the next time it gets a chance.
  causeRender();
}

void ImuDisplay::setColor( const rviz::Color& color )
{
  color_ = color;

  propertyChanged( color_property_ );
  updateColorAndAlpha();
  causeRender();
}

void ImuDisplay::setAlpha( float alpha )
{
  alpha_ = alpha;

  propertyChanged( alpha_property_ );
  updateColorAndAlpha();
  causeRender();
}

// Set the current color and alpha values for each visual.
void ImuDisplay::updateColorAndAlpha()
{
  for( size_t i = 0; i < visuals_.size(); i++ )
  {
    if( visuals_[ i ] )
    {
      visuals_[ i ]->setColor( color_.r_, color_.g_, color_.b_, alpha_ );
    }
  }
}

// Set the number of past visuals to show.
void ImuDisplay::setHistoryLength( int length )
{
  // Don't let people enter invalid values.
  if( length < 1 )
  {
    length = 1;
  }
  // If the length is not changing, we don't need to do anything.
  if( history_length_ == length )
  {
    return;
  }

  // Set the actual variable.
  history_length_ = length;
  propertyChanged( history_length_property_ );
  
  // Create a new array of visual pointers, all NULL.
  std::vector<ImuVisual*> new_visuals( history_length_, (ImuVisual*)0 );

  // Copy the contents from the old array to the new.
  size_t copy_len = ( new_visuals.size() > visuals_.size() ) ? visuals_.size() : new_visuals.size(); // minimum of 2 lengths
  for( size_t i = 0; i < copy_len; i++ )
  {
    int new_index = (messages_received_ - i) % new_visuals.size();
    int old_index = (messages_received_ - i) % visuals_.size();
    new_visuals[ new_index ] = visuals_[ old_index ];
    visuals_[ old_index ] = NULL;
  }

  // Delete any remaining old visuals
  for( size_t i = 0; i < visuals_.size(); i++ ) {
    delete visuals_[ i ];
  }

  // We don't need to create any new visuals here, they are created as
  // needed when messages are received.

  // Put the new vector into the member variable version and let the
  // old one go out of scope.
  visuals_.swap( new_visuals );
}

void ImuDisplay::subscribe()
{
  // If we are not actually enabled, don't do it.
  if ( !isEnabled() )
  {
    return;
  }

  // Try to subscribe to the current topic name (in topic_).  Make
  // sure to catch exceptions and set the status to a descriptive
  // error message.
  try
  {
    sub_.subscribe( update_nh_, topic_, 10 );
    setStatus( rviz::status_levels::Ok, "Topic", "OK" );
  }
  catch( ros::Exception& e )
  {
    setStatus( rviz::status_levels::Error, "Topic", std::string( "Error subscribing: " ) + e.what() );
  }
}

void ImuDisplay::unsubscribe()
{
  sub_.unsubscribe();
}

void ImuDisplay::onEnable()
{
  subscribe();
}

void ImuDisplay::onDisable()
{
  unsubscribe();
  clear();
}

// When the "Fixed Frame" changes, we need to update our
// tf::MessageFilter and erase existing visuals.
void ImuDisplay::fixedFrameChanged()
{
  tf_filter_->setTargetFrame( fixed_frame_ );
  clear();
}

// This is our callback to handle an incoming message.
void ImuDisplay::incomingMessage( const sensor_msgs::Imu::ConstPtr& msg )
{
  ++messages_received_;
  
  // Each display can have multiple status lines.  This one is called
  // "Topic" and says how many messages have been received in this case.
  std::stringstream ss;
  ss << messages_received_ << " messages received";
  setStatus( rviz::status_levels::Ok, "Topic", ss.str() );

  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this Imu message.  If
  // it fails, we can't do anything else so we return.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if( !vis_manager_->getFrameManager()->getTransform( msg->header.frame_id, msg->header.stamp, position, orientation ))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), fixed_frame_.c_str() );
    return;
  }

  // We are keeping a circular buffer of visual pointers.  This gets
  // the next one, or creates and stores it if it was missing.
  ImuVisual* visual = visuals_[ messages_received_ % history_length_ ];
  if( visual == NULL )
  {
    visual = new ImuVisual( vis_manager_->getSceneManager(), scene_node_ );
    visuals_[ messages_received_ % history_length_ ] = visual;
  }

  // Now set or update the contents of the chosen visual.
  visual->setMessage( msg );
  visual->setFramePosition( position );
  visual->setFrameOrientation( orientation );
  visual->setColor( color_.r_, color_.g_, color_.b_, alpha_ );
}

// Override rviz::Display's reset() function to add a call to clear().
void ImuDisplay::reset()
{
  Display::reset();
  clear();
}

// Override createProperties() to build and configure a Property
// object for each user-editable property.  property_manager_,
// property_prefix_, and parent_category_ are all initialized before
// this is called.
void ImuDisplay::createProperties()
{
  topic_property_ =
    property_manager_->createProperty<rviz::ROSTopicStringProperty>( "Topic",
                                                                     property_prefix_,
                                                                     boost::bind( &ImuDisplay::getTopic, this ),
                                                                     boost::bind( &ImuDisplay::setTopic, this, _1 ),
                                                                     parent_category_,
                                                                     this );
  setPropertyHelpText( topic_property_, "sensor_msgs::Imu topic to subscribe to." );
  rviz::ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType( ros::message_traits::datatype<sensor_msgs::Imu>() );

  color_property_ =
    property_manager_->createProperty<rviz::ColorProperty>( "Color",
                                                            property_prefix_,
                                                            boost::bind( &ImuDisplay::getColor, this ),
                                                            boost::bind( &ImuDisplay::setColor, this, _1 ),
                                                            parent_category_,
                                                            this );
  setPropertyHelpText( color_property_, "Color to draw the acceleration arrows." );

  alpha_property_ =
    property_manager_->createProperty<rviz::FloatProperty>( "Alpha",
                                                            property_prefix_,
                                                            boost::bind( &ImuDisplay::getAlpha, this ),
                                                            boost::bind( &ImuDisplay::setAlpha, this, _1 ),
                                                            parent_category_,
                                                            this );
  setPropertyHelpText( alpha_property_, "0 is fully transparent, 1.0 is fully opaque." );

  history_length_property_ =
    property_manager_->createProperty<rviz::IntProperty>( "History Length",
                                                          property_prefix_,
                                                          boost::bind( &ImuDisplay::getHistoryLength, this ),
                                                          boost::bind( &ImuDisplay::setHistoryLength, this, _1 ),
                                                          parent_category_,
                                                          this );
  setPropertyHelpText( history_length_property_, "Number of prior measurements to display." );
}

} // end namespace rviz_plugin_tutorials

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS( rviz_plugin_tutorials, Imu, rviz_plugin_tutorials::ImuDisplay, rviz::Display )
// %EndTag(FULL_SOURCE)%
