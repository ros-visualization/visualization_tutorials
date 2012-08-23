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

#include <QDockWidget>

#include <OGRE/OgreRenderWindow.h>

#include <rviz/bit_allocator.h>
#include <rviz/display_context.h>
#include <rviz/display_group.h>
#include <rviz/render_panel.h>
#include <rviz/window_manager_interface.h>

#include "selective_camera_display.h"

namespace rviz_plugin_tutorials
{

SelectiveCameraDisplay::SelectiveCameraDisplay()
  : panel_container_( NULL )
  , render_panel_( NULL )
  , my_bit_( 0 )
{
}

SelectiveCameraDisplay::~SelectiveCameraDisplay()
{
  delete render_panel_;
  delete panel_container_;
  if( context_ )
  {
    context_->visibilityBits()->freeBits( my_bit_ );
  }
}

void SelectiveCameraDisplay::onInitialize()
{
  my_bit_ = context_->visibilityBits()->allocBit();

  render_panel_ = new rviz::RenderPanel();
  render_panel_->getRenderWindow()->setAutoUpdated( false );
  render_panel_->getRenderWindow()->setActive( true );
  render_panel_->resize( 640, 480 );
  render_panel_->initialize( context_->getSceneManager(), context_ );
  render_panel_->setAutoRender( true );
  rviz::WindowManagerInterface* wm = context_->getWindowManager();
  if( wm )
  {
    panel_container_ = wm->addPane( getName(), render_panel_);
  }
  render_panel_->getViewport()->setVisibilityMask( my_bit_ );
}

void SelectiveCameraDisplay::update( float wall_dt, float ros_dt )
{
  rviz::DisplayGroup* displays = context_->getRootDisplayGroup();
  for( int i = 0; i < displays->numDisplays(); i++ )
  {
    rviz::Display* display = displays->getDisplayAt( i );
    // If this display does *not* meet the criteria, unset our corresponding visibility bit.
    if( !display->getName().startsWith( "G" ))
    {
      display->unsetVisibilityBits( my_bit_ );
    }
    else
    {
      display->setVisibilityBits( my_bit_ );
    }
  }

  render_panel_->getRenderWindow()->update();
}

} // end namespace rviz_plugin_tutorials

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS( rviz_plugin_tutorials, SelectiveCamera, rviz_plugin_tutorials::SelectiveCameraDisplay, rviz::Display )
