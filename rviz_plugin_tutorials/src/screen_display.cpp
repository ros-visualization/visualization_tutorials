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

#include <sstream>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreManualObject.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/property.h>
#include <rviz/properties/property_manager.h>
#include <rviz/frame_manager.h>

#include "screen_display.h"

namespace rviz_plugin_tutorials
{


ScreenDisplay::ScreenDisplay()
  : Display()
  , texture_( update_nh_ )
  , scene_node_( NULL )
  , scale_( 0.01 )
{
}

// After the parent rviz::Display::initialize() does its own setup, it
// calls the subclass's onInitialize() function.  This is where we
// instantiate all the workings of the class.
void ScreenDisplay::onInitialize()
{
  // Make an Ogre::SceneNode to contain all our visuals.
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  
  // Create a unique name for the material.
  static int count = 0;
  std::stringstream ss;
  ss << "ScreenDisplay" << count++ << "Material";

  // Create the material and connect it to the texture_.
  image_material_ =
    Ogre::MaterialManager::getSingleton().create( ss.str(),
                                                  Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
  image_material_->getTechnique( 0 )->setLightingEnabled( false );
  Ogre::TextureUnitState* tu = image_material_->getTechnique( 0 )->getPass( 0 )->createTextureUnitState();
  tu->setTextureName( texture_.getTexture()->getName() );
  tu->setTextureFiltering( Ogre::TFO_NONE );
  tu->setAlphaOperation( Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, 0.0 );
  image_material_->setCullingMode( Ogre::CULL_NONE );
  image_material_->setSceneBlending( Ogre::SBT_REPLACE );

  // Create the rectangle object
  screen_object_ = scene_manager_->createManualObject();
  scene_node_->attachObject( screen_object_ );
  screen_object_->estimateVertexCount( 4 );
  screen_object_->begin( image_material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST );
  screen_object_->position( -0.5, -0.5, 0 );
  screen_object_->textureCoord( 0, 1 );
  screen_object_->position( 0.5, -0.5, 0 );
  screen_object_->textureCoord( 1, 1 );
  screen_object_->position( 0.5, 0.5, 0 );
  screen_object_->textureCoord( 1, 0 );
  screen_object_->position( -0.5, 0.5, 0 );
  screen_object_->textureCoord( 0, 0 );
  screen_object_->quad( 0, 1, 2, 3 );
  screen_object_->end();
}

ScreenDisplay::~ScreenDisplay()
{
  unsubscribe();

  scene_manager_->destroySceneNode( scene_node_ );
  scene_manager_->destroyManualObject( screen_object_ );
  image_material_->unload();
}

void ScreenDisplay::setTopic( const std::string& topic )
{
  unsubscribe();
  topic_ = topic;
  subscribe();

  // Broadcast the fact that the variable has changed.
  propertyChanged( topic_property_ );

  // Make sure rviz renders the next time it gets a chance.
  causeRender();
}

void ScreenDisplay::subscribe()
{
  // If we are not actually enabled, don't do it.
  if ( !isEnabled() )
  {
    return;
  }

  try
  {
    texture_.setTopic( topic_ );
    setStatus( rviz::status_levels::Ok, "Topic", "OK" );
  }
  catch( ros::Exception& e )
  {
    setStatus( rviz::status_levels::Error, "Topic", std::string("Error subscribing: ") + e.what() );
  }
}

void ScreenDisplay::unsubscribe()
{
  texture_.setTopic( "" );
}

void ScreenDisplay::onEnable()
{
  subscribe();
}

void ScreenDisplay::onDisable()
{
  unsubscribe();
}

void ScreenDisplay::update( float dt, float ros_dt )
{
  if (texture_.getImageCount() == 0)
  {
    setStatus(rviz::status_levels::Warn, "Image", "No image received");
  }
  else
  {
    std::stringstream ss;
    ss << texture_.getImageCount() << " images received";
    setStatus(rviz::status_levels::Ok, "Image", ss.str());
  }

  texture_.update();

  sensor_msgs::Image::ConstPtr current_image = texture_.getImage();
  if( current_image )
  {
    scene_node_->setScale( current_image->width * scale_,
                           current_image->height * scale_, 1 );

    std::string image_frame = current_image->header.frame_id;
    if( image_frame == "" )
    {
      image_frame = fixed_frame_;
      setStatus(rviz::status_levels::Warn, "Frame", "Image header has empty frame_id.");
    }

    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if( vis_manager_->getFrameManager()->getTransform( image_frame, ros::Time(), position, orientation ))
    {
      scene_node_->setPosition( position );
      scene_node_->setOrientation( orientation );
    }
    else
    {
      setStatus( rviz::status_levels::Error,
                 "Frame", "Can't find a transform from " + fixed_frame_ + " to " + image_frame + "." );
    }
  }
}

void ScreenDisplay::setScale( float scale )
{
  scale_ = scale;
  propertyChanged( scale_property_ );
}

// Override createProperties() to build and configure a Property
// object for each user-editable property.  ``property_manager_``,
// ``property_prefix_``, and ``parent_category_`` are all initialized before
// this is called.
void ScreenDisplay::createProperties()
{
  topic_property_ =
    property_manager_->createProperty<rviz::ROSTopicStringProperty>( "Topic",
                                                                     property_prefix_,
                                                                     boost::bind( &ScreenDisplay::getTopic, this ),
                                                                     boost::bind( &ScreenDisplay::setTopic, this, _1 ),
                                                                     parent_category_,
                                                                     this );
  setPropertyHelpText( topic_property_, "sensor_msgs::Image topic to subscribe to." );
  rviz::ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType( ros::message_traits::datatype<sensor_msgs::Image>() );

  scale_property_ =
    property_manager_->createProperty<rviz::FloatProperty>( "Scale",
                                                            property_prefix_,
                                                            boost::bind( &ScreenDisplay::getScale, this ),
                                                            boost::bind( &ScreenDisplay::setScale, this, _1 ),
                                                            parent_category_,
                                                            this );
  setPropertyHelpText( scale_property_, "Scale of image, in meters per pixel." );
}

} // end namespace rviz_plugin_tutorials

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS( rviz_plugin_tutorials, Screen, rviz_plugin_tutorials::ScreenDisplay, rviz::Display )
// END_TUTORIAL
