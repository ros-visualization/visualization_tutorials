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

#include <QSlider>
#include <QLabel>
#include <QGridLayout>
#include <QVBoxLayout>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display_wrapper.h"
#include "rviz/default_plugin/grid_display.h"
#include "rviz/ogre_helpers/grid.h"

#include "myviz.h"

// BEGIN_TUTORIAL
// Constructor for MyViz.  This does most of the work of the class.
MyViz::MyViz( QWidget* parent )
  : QWidget( parent )
{
  // Construct and lay out labels and slider controls.
  QLabel* thickness_label = new QLabel( "Line Thickness" );
  QSlider* thickness_slider = new QSlider( Qt::Horizontal );
  thickness_slider->setMinimum( 1 );
  thickness_slider->setMaximum( 100 );
  QLabel* cell_size_label = new QLabel( "Cell Size" );
  QSlider* cell_size_slider = new QSlider( Qt::Horizontal );
  cell_size_slider->setMinimum( 1 );
  cell_size_slider->setMaximum( 100 );
  QGridLayout* controls_layout = new QGridLayout();
  controls_layout->addWidget( thickness_label, 0, 0 );
  controls_layout->addWidget( thickness_slider, 0, 1 );
  controls_layout->addWidget( cell_size_label, 1, 0 );
  controls_layout->addWidget( cell_size_slider, 1, 1 );

  // Construct and lay out render panel.
  render_panel_ = new rviz::RenderPanel();
  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addLayout( controls_layout );
  main_layout->addWidget( render_panel_ );

  // Set the top-level layout for this MyViz widget.
  setLayout( main_layout );

  // Make signal/slot connections.
  connect( thickness_slider, SIGNAL( valueChanged( int )), this, SLOT( setThickness( int )));
  connect( cell_size_slider, SIGNAL( valueChanged( int )), this, SLOT( setCellSize( int )));

  // Next we initialize the main RViz classes.
  //
  // The VisualizationManager is the container for Display objects,
  // holds the main Ogre scene, holds the ViewController, etc.  It is
  // very central and we will probably need one in every usage of
  // librviz.
  manager_ = new rviz::VisualizationManager( render_panel_ );
  render_panel_->initialize( manager_->getSceneManager(), manager_ );
  manager_->initialize();
  manager_->startUpdate();

  // Create a Grid display.
  rviz::DisplayWrapper* wrapper = manager_->createDisplay( "rviz/Grid", "adjustable grid", true );
  ROS_ASSERT( wrapper != NULL );

  // Unwrap it.
  rviz::Display* display = wrapper->getDisplay();
  ROS_ASSERT( display != NULL );

  // Downcast it to the type we think we know it is.
  //
  // (This is one part I would like to improve in the future.  For
  // this to work currently, we need to link against the plugin
  // library containing GridDisplay (libdefault_plugin.so) in addition
  // to linking against librviz.so.  This pretty much negates the
  // benefits of the plugin architecture.)
  grid_ = dynamic_cast<rviz::GridDisplay*>( display );
  ROS_ASSERT( grid_ != NULL );

  // Configure the GridDisplay the way we like it.
  grid_->setStyle( rviz::Grid::Billboards ); // Fat lines.
  grid_->setColor( rviz::Color( 1.0f, 1.0f, 0.0f )); // I like yellow.

  // Initialize the slider values.
  thickness_slider->setValue( 25 );
  cell_size_slider->setValue( 10 );
}

// Destructor for MyViz.  The complexity here is something I would
// like to avoid in future versions.
//
// Removing all the displays first is not technically required for
// this example, but if we used a PropertyTreeWidget it would be
// required, so it is a good idea.
//
// It would be required because Display objects own Properties, and
// Properties own children of PropertyTreeWidget
// (PropertyWidgetItems).  PropertyTreeWidget notices when
// PropertyWidgetItems are destroyed, but Properties don't notice when
// PropertyWidgetItems are destroyed, so must destroy from the Display
// (and thus Property) side first.
//
// The ``render_panel_`` is a child widget of MyViz and so would be
// deleted naturally by the QWidget destructor, but that would be
// after we deleted ``manager_``.  Instead we must delete
// ``render_panel_`` *before* ``manager_`` because
// ``~VisualizationManager()`` destroys ogre SceneManager which
// destroys all attached SceneNodes.  RenderPanel indirectly holds
// pointers to SceneNodes which it destroys.  RenderPanel doesn't know
// when Ogre destroys its SceneNodes, so RenderPanel would cause a
// segfault during its destructor.
MyViz::~MyViz()
{
  if( manager_ != NULL )
  {
    manager_->removeAllDisplays();
  }
  delete render_panel_;
  delete manager_;
}

// This function is a Qt slot connected to a QSlider's valueChanged()
// signal.  It calls a property setter function on the GridDisplay,
// setLineWidth().
void MyViz::setThickness( int thickness_percent )
{
  if( grid_ != NULL )
  {
    grid_->setLineWidth( thickness_percent / 100.0f );
  }
}

// This function is a Qt slot connected to a QSlider's valueChanged()
// signal.  It calls a property setter function on the GridDisplay,
// setCellSize().
void MyViz::setCellSize( int cell_size_percent )
{
  if( grid_ != NULL )
  {
    grid_->setCellSize( cell_size_percent / 10.0f );
  }
}
