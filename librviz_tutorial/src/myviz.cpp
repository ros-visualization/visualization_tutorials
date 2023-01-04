// Copyright (c) 2011, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "myviz.hpp"

#include <QColor>
#include <QGridLayout>
#include <QLabel>
#include <QSlider>
#include <QVBoxLayout>

#include <memory>

#include "rviz_common/display.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"
#include "rviz_common/visualization_manager.hpp"
#include "rviz_common/window_manager_interface.hpp"
#include "rviz_rendering/render_window.hpp"

// BEGIN_TUTORIAL
// Constructor for MyViz.  This does most of the work of the class.
MyViz::MyViz(
  QApplication * app,
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node,
  QWidget * parent)
: QMainWindow(parent), app_(app), rviz_ros_node_(rviz_ros_node)
{
  // Construct and lay out labels and slider controls.
  QLabel * thickness_label = new QLabel("Line Thickness");
  QSlider * thickness_slider = new QSlider(Qt::Horizontal);
  thickness_slider->setMinimum(1);
  thickness_slider->setMaximum(100);
  QLabel * cell_size_label = new QLabel("Cell Size");
  QSlider * cell_size_slider = new QSlider(Qt::Horizontal);
  cell_size_slider->setMinimum(1);
  cell_size_slider->setMaximum(100);
  QGridLayout * controls_layout = new QGridLayout();
  controls_layout->addWidget(thickness_label, 0, 0);
  controls_layout->addWidget(thickness_slider, 0, 1);
  controls_layout->addWidget(cell_size_label, 1, 0);
  controls_layout->addWidget(cell_size_slider, 1, 1);

  // Add visualization.
  QVBoxLayout * main_layout = new QVBoxLayout;
  QWidget * central_widget = new QWidget();
  main_layout->addLayout(controls_layout);
  main_layout->setSpacing(0);
  main_layout->setMargin(0);

  // Next we initialize the main RViz classes.
  //
  // The VisualizationManager is the container for Display objects,
  // holds the main Ogre scene, holds the ViewController, etc.  It is
  // very central and we will probably need one in every usage of
  // librviz.
  app_->processEvents();
  render_panel_ = std::make_shared<rviz_common::RenderPanel>(central_widget);
  app_->processEvents();
  render_panel_->getRenderWindow()->initialize();
  auto clock = rviz_ros_node_.lock()->get_raw_node()->get_clock();
  rviz_common::WindowManagerInterface * wm = nullptr;
  manager_ = std::make_shared<rviz_common::VisualizationManager>(
    render_panel_.get(), rviz_ros_node_, wm, clock);
  render_panel_->initialize(manager_.get());
  app_->processEvents();
  manager_->initialize();
  manager_->startUpdate();

  // Set the top-level layout for this MyViz widget.
  central_widget->setLayout(main_layout);
  setCentralWidget(central_widget);
  main_layout->addWidget(render_panel_.get());

  // Make signal/slot connections.
  connect(thickness_slider, SIGNAL(valueChanged(int)), this, SLOT(setThickness(int)));
  connect(cell_size_slider, SIGNAL(valueChanged(int)), this, SLOT(setCellSize(int)));

  // Create a Grid display.
  grid_ = manager_->createDisplay("rviz_default_plugins/Grid", "adjustable grid", true);
  if (grid_ == NULL) {
    throw std::runtime_error("Error creating grid display");
  }

  // Configure the GridDisplay the way we like it.
  grid_->subProp("Line Style")->setValue("Billboards");
  grid_->subProp("Color")->setValue(QColor(Qt::yellow));

  // Initialize the slider values.
  thickness_slider->setValue(25);
  cell_size_slider->setValue(10);
}

// This function is a Qt slot connected to a QSlider's valueChanged() signal.
// It sets the line thickness of the grid by changing the grid's "Line Width" property.
void MyViz::setThickness(int thickness_percent)
{
  if (grid_ != NULL) {
    grid_->subProp("Line Style")->subProp("Line Width")->setValue(thickness_percent / 100.0f);
  }
}

// This function is a Qt slot connected to a QSlider's valueChanged() signal.
// It sets the cell size of the grid by changing the grid's "Cell Size" Property.
void MyViz::setCellSize(int cell_size_percent)
{
  if (grid_ != NULL) {
    grid_->subProp("Cell Size")->setValue(cell_size_percent / 10.0f);
  }
}

void MyViz::closeEvent(QCloseEvent * event)
{
  QMainWindow::closeEvent(event);
  rclcpp::shutdown();
}
