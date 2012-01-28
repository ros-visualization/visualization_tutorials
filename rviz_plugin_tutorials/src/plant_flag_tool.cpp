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

#include "plant_flag_tool.h"

namespace rviz_plugin_tutorials
{

PlantFlagTool::PlantFlagTool()
{
}

PlantFlagTool::~PlantFlagTool()
{
}

void PlantFlagTool::onInitialize()
{
  scene_node_ = vis_manager_->getSceneManager()->getRootSceneNode()->createChildSceneNode();
  scene_node_->setVisible( false );

  std::string flag_resource = "package://media/flag.dae";

  if( loadMeshFromResource( flag_resource ).isNull() )
  {
    ROS_WARN( "PlantFlagTool: failed to load model resource '%s'.", flag_resource.c_str() );
    return;
  }

  entity_ = vis_manager_->getSceneManager()->createEntity( flag_resource );
  scene_node_->attachObject( entity_ );
}

void PlantFlagTool::activate()
{
  scene_node_->setVisible( true );
}

void PlantFlagTool::deactivate()
{
  scene_node_->setVisible( false );
}

int PlantFlagTool::processMouseEvent( ViewportMouseEvent& event )
{
  Ogre::Vector3 flag_rel_fixed = rviz::PoseTool::getPositionFromMouseXY( event.viewport, event.x, event.y );

  scene_node_->setPosition( flag_rel_fixed );
}

} // end namespace rviz_plugin_tutorials
