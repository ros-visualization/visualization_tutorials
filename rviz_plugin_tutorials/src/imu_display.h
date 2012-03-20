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
#ifndef IMU_DISPLAY_H
#define IMU_DISPLAY_H

#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <sensor_msgs/Imu.h>
#include <rviz/display.h>

namespace Ogre
{
class SceneNode;
}

// All the source in this plugin is in its own namespace.  This is not
// required but is good practice.
namespace rviz_plugin_tutorials
{

class ImuVisual;

// Here we declare our new subclass of rviz::Display.  Every display
// which can be listed in the "Displays" panel is a subclass of
// rviz::Display.
class ImuDisplay: public rviz::Display
{
public:
  // Constructor.  pluginlib::ClassLoader creates instances by calling
  // the default constructor, so make sure you have one.
  ImuDisplay();
  virtual ~ImuDisplay();

  // Overrides of public Display virtual functions.
  virtual void onInitialize();
  virtual void fixedFrameChanged();
  virtual void reset();
  virtual void createProperties();

  // Setter and getter functions for user-editable properties.
  void setTopic(const std::string& topic);
  const std::string& getTopic() { return topic_; }

  void setColor( const rviz::Color& color );
  const rviz::Color& getColor() { return color_; }

  void setAlpha( float alpha );
  float getAlpha() { return alpha_; }

  void setHistoryLength( int history_length );
  int getHistoryLength() const { return history_length_; }

protected:
  // Overrides of protected virtual functions from Display.  As much
  // as possible, when Displays are not enabled, they should not be
  // subscribed to incoming data and should not show anything in the
  // 3D view.  These functions are where these connections are made
  // and broken.
  virtual void onEnable();
  virtual void onDisable();

private:
  // Function to handle an incoming ROS message.
  void incomingMessage( const sensor_msgs::Imu::ConstPtr& msg );

  // Internal helpers which do the work of subscribing and
  // unsubscribing from the ROS topic.
  void subscribe();
  void unsubscribe();

  void clear();

  void updateColorAndAlpha();

  // Storage for the list of visuals.  This display supports an
  // adjustable history length, so we need one visual per history
  // item.
  std::vector<ImuVisual*> visuals_;

  // A node in the Ogre scene tree to be the parent of all our visuals.
  Ogre::SceneNode* scene_node_;

  // Data input: Subscriber and tf message filter.
  message_filters::Subscriber<sensor_msgs::Imu> sub_;
  tf::MessageFilter<sensor_msgs::Imu>* tf_filter_;
  int messages_received_;

  // User-editable property variables.
  rviz::Color color_;
  std::string topic_;
  float alpha_;
  int history_length_;

  // Property objects for user-editable properties.
  rviz::ColorPropertyWPtr color_property_;
  rviz::ROSTopicStringPropertyWPtr topic_property_;
  rviz::FloatPropertyWPtr alpha_property_;
  rviz::IntPropertyWPtr history_length_property_;
};

} // end namespace rviz_plugin_tutorials

#endif // IMU_DISPLAY_H
