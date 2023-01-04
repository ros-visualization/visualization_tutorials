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

#include "imu_display.hpp"

#include <memory>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/visualization_manager.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "tf2_ros/transform_listener.h"

#include "imu_visual.hpp"

namespace rviz_plugin_tutorials
{

// BEGIN_TUTORIAL
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
ImuDisplay::ImuDisplay()
{
  color_property_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(204, 51, 204),
    "Color to draw the acceleration arrows.",
    this, SLOT(updateColorAndAlpha()));

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1.0,
    "0 is fully transparent, 1.0 is fully opaque.",
    this, SLOT(updateColorAndAlpha()));

  history_length_property_ = new rviz_common::properties::IntProperty(
    "History Length", 1,
    "Number of prior measurements to display.",
    this, SLOT(updateHistoryLength()));
  history_length_property_->setMin(1);
  history_length_property_->setMax(100000);
}

// After the top-level rviz_common::Display::initialize() does its own setup,
// it calls the subclass's onInitialize() function.  This is where we
// instantiate all the workings of the class.  We make sure to also
// call our immediate super-class's onInitialize() function, since it
// does important stuff setting up the message filter.
//
// Note that "MFDClass" is a typedef of
// ``MessageFilterDisplay<message type>``, to save typing that long
// templated class name every time you need to refer to the
// superclass.
void ImuDisplay::onInitialize()
{
  MFDClass::onInitialize();
  updateHistoryLength();
}

ImuDisplay::~ImuDisplay()
{
}

// Clear the visuals by deleting their objects.
void ImuDisplay::reset()
{
  MFDClass::reset();
  visuals_.clear();
}

// Set the current color and alpha values for each visual.
void ImuDisplay::updateColorAndAlpha()
{
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();

  for (size_t i = 0; i < visuals_.size(); i++) {
    visuals_[i]->setColor(color.r, color.g, color.b, alpha);
  }
}

// Set the number of past visuals to show.
void ImuDisplay::updateHistoryLength()
{
  history_length_ = static_cast<std::size_t>(history_length_property_->getInt());
  if (visuals_.size() > history_length_) {
    visuals_.resize(history_length_);
  }
}

// This is our callback to handle an incoming message.
void ImuDisplay::processMessage(sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  // Here we call the rviz_common::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this Imu message.  If
  // it fails, we can't do anything else so we return.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(
      msg->header.frame_id, msg->header.stamp, position, orientation))
  {
    RCLCPP_INFO(
      rclcpp::get_logger("imu_display"),
      "Error transforming from frame '%s' to frame '%s'",
      msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }

  // Set the contents of the visual.
  std::shared_ptr<ImuVisual> visual;
  visual.reset(new ImuVisual(context_->getSceneManager(), scene_node_));
  visual->setMessage(msg);
  visual->setFramePosition(position);
  visual->setFrameOrientation(orientation);

  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();
  visual->setColor(color.r, color.g, color.b, alpha);

  // We are keeping a deque of visual pointers.
  // This removes the oldest visual from the back if the capacity has been reached,
  // and adds our new visual to the front.
  if (visuals_.size() == history_length_) {
    visuals_.pop_back();
  }
  visuals_.push_front(visual);
}

}  // end namespace rviz_plugin_tutorials

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
PLUGINLIB_EXPORT_CLASS(rviz_plugin_tutorials::ImuDisplay, rviz_common::Display)
// END_TUTORIAL
