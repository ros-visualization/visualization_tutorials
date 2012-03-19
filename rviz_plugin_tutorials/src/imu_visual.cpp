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

#include "imu_visual.h"

namespace rviz_plugin_tutorials
{

ImuVisual::ImuVisual( sensor_msgs::Imu::ConstPtr& msg, Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
{
  scene_manager_ = scene_manager;
  arrow_node_ = parent_node->createChildSceneNode();
  acceleration_arrow_ = new rviz::Arrow( scene_manager_, arrow_node_ );

  const geometry_msgs::Vector3& a = msg.linear_acceleration;
  float length = fsqrtf( a.x*a.x + a.y*a.y + a.z*a.z );
  Ogre::Vector3 scale( length, length, length );
  acceleration_arrow_->setScale( scale );

  
}

ImuVisual::~ImuVisual()
{
  delete acceleration_arrow_;
  scene_manager_->destroySceneNode( arrow_node_ );
}

void ImuVisual::setFramePosition( Ogre::Vector3 position )
{
  arrow_node_->setPosition( position );
}

void ImuVisual::setFrameOrientation( Ogre::Quaternion orientation )
{
  arrow_node_->setOrientation( orientation );
}

void ImuVisual::setColor( float r, float g, float b, float a )
{
  acceleration_arrow_->setColor( r, g, b, a );
}

} // end namespace rviz_plugin_tutorials
