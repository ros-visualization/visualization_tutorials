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

#ifndef MYVIZ_HPP_
#define MYVIZ_HPP_

#include <QApplication>
#include <QMainWindow>
#include <QWidget>

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/display.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"

namespace rviz_common
{
class Display;
class RenderPanel;
class VisualizationManager;
}

// BEGIN_TUTORIAL
// Class "MyViz" implements the top level widget for this example.
class MyViz : public QMainWindow
{
  Q_OBJECT

public:
  MyViz(
    QApplication * app,
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node,
    QWidget * parent = 0);

private Q_SLOTS:
  void setThickness(int thickness_percent);
  void setCellSize(int cell_size_percent);
  void closeEvent(QCloseEvent * event);

private:
  QApplication * app_;
  std::shared_ptr<rviz_common::VisualizationManager> manager_;
  std::shared_ptr<rviz_common::RenderPanel> render_panel_;
  rviz_common::Display * grid_;
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
};
// END_TUTORIAL
#endif  // MYVIZ_HPP_
