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

#ifndef SCREEN_DISPLAY_H
#define SCREEN_DISPLAY_H

#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <rviz/display.h>
#include <rviz/image/ros_image_texture.h>

namespace Ogre
{
class SceneNode;
class ManualObject;
}

// All the source in this plugin is in its own namespace.  This is not
// required but is good practice.
namespace rviz_plugin_tutorials
{

/** @brief Display an image *in* the 3D scene, at the pose of the
 * frame published in the sensor_msgs/Image. */
class ScreenDisplay: public rviz::Display
{
public:
  // Constructor.  pluginlib::ClassLoader creates instances by calling
  // the default constructor, so make sure you have one.
  ScreenDisplay();
  virtual ~ScreenDisplay();

  // Overrides of public virtual functions from the Display class.
  virtual void onInitialize();
  virtual void fixedFrameChanged();
  virtual void reset();
  virtual void createProperties();
  virtual void update( float dt, float ros_dt );

  // Setter and getter functions for user-editable properties.
  void setTopic(const std::string& topic);
  const std::string& getTopic() { return topic_; }

  void setScale( float scale );
  float getScale() { return scale_; }

  // Overrides of protected virtual functions from Display.  As much
  // as possible, when Displays are not enabled, they should not be
  // subscribed to incoming data and should not show anything in the
  // 3D view.  These functions are where these connections are made
  // and broken.
protected:
  virtual void onEnable();
  virtual void onDisable();

private:
  // Internal helpers which do the work of subscribing and
  // unsubscribing from the ROS topic.
  void subscribe();
  void unsubscribe();

  // A helper to clear this display back to the initial state.
  void clear();

  Ogre::MaterialPtr image_material_;
  Ogre::ManualObject* screen_object_;
  rviz::ROSImageTexture texture_;

  // A node in the Ogre scene tree to be the parent of all our visuals.
  Ogre::SceneNode* scene_node_;

  // User-editable property variables.
  std::string topic_;
  /** @brief Scale at which to display the image, in meters-per-image-pixel. */
  float scale_;

  // Property objects for user-editable properties.
  rviz::ROSTopicStringPropertyWPtr topic_property_;
  rviz::FloatPropertyWPtr scale_property_;
};

} // end namespace rviz_plugin_tutorials

#endif // SCREEN_DISPLAY_H
