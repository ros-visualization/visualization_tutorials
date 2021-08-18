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

#ifndef IMU_DISPLAY_HPP_
#define IMU_DISPLAY_HPP_

#ifndef Q_MOC_RUN
#include <deque>
#include <memory>

#include "rviz_common/message_filter_display.hpp"
#include "sensor_msgs/msg/imu.hpp"
#endif

namespace Ogre
{
class SceneNode;
}

namespace rviz_common::properties
{
class ColorProperty;
class FloatProperty;
class IntProperty;
}

// All the source in this plugin is in its own namespace.  This is not
// required but is good practice.
namespace rviz_plugin_tutorials
{

class ImuVisual;

// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz_common::Display.  Every display
// which can be listed in the "Displays" panel is a subclass of
// rviz_common::Display.
//
// ImuDisplay will show a 3D arrow showing the direction and magnitude
// of the IMU acceleration vector.  The base of the arrow will be at
// the frame listed in the header of the Imu message, and the
// direction of the arrow will be relative to the orientation of that
// frame.  It will also optionally show a history of recent
// acceleration vectors, which will be stored in a deque.
//
// The ImuDisplay class itself just implements the deque,
// editable parameters, and Display subclass machinery.  The visuals
// themselves are represented by a separate class, ImuVisual.  The
// idiom for the visuals is that when the objects exist, they appear
// in the scene, and when they are deleted, they disappear.
class ImuDisplay : public rviz_common::MessageFilterDisplay<sensor_msgs::msg::Imu>
{
  Q_OBJECT

public:
  // Constructor.  pluginlib::ClassLoader creates instances by calling
  // the default constructor, so make sure you have one.
  ImuDisplay();
  virtual ~ImuDisplay();

  // Overrides of protected virtual functions from Display.  As much
  // as possible, when Displays are not enabled, they should not be
  // subscribed to incoming data and should not show anything in the
  // 3D view.  These functions are where these connections are made
  // and broken.

protected:
  virtual void onInitialize();

  // A helper to clear this display back to the initial state.
  virtual void reset();

// These Qt slots get connected to signals indicating changes in the user-editable properties.

private Q_SLOTS:
  void updateColorAndAlpha();
  void updateHistoryLength();

private:
  // Function to handle an incoming ROS message.
  void processMessage(sensor_msgs::msg::Imu::ConstSharedPtr msg);

  // Storage for the list of visuals. It is a deque where
  // data gets popped from the back (oldest) and pushed to the front (newest).
  std::deque<std::shared_ptr<ImuVisual>> visuals_;
  std::size_t history_length_{1};

  // User-editable property variables.
  rviz_common::properties::ColorProperty * color_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::IntProperty * history_length_property_;
};
// END_TUTORIAL

}  // end namespace rviz_plugin_tutorials

#endif  // IMU_DISPLAY_HPP_
