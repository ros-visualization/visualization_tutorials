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
#ifndef TELEOP_PANEL_H
#define TELEOP_PANEL_H

#include <string>

#include <ros/ros.h>

#include <rviz/panel.h>

class QLineEdit;

namespace rviz_plugin_tutorials
{

class DriveWidget;

class TeleopPanel: public rviz::Panel
{
Q_OBJECT
public:
  TeleopPanel( QWidget* parent = 0 );

  /**
   * Save panel's internal data (the topic name).
   */
  virtual void saveToConfig( const std::string& key_prefix, const boost::shared_ptr<rviz::Config>& config );

  /**
   * Load panel's internal data (the topic name).
   */
  virtual void loadFromConfig( const std::string& key_prefix, const boost::shared_ptr<rviz::Config>& config );

public Q_SLOTS:
  void setVel( float linear_velocity_, float angular_velocity_ );
  void setTopic( const std::string& topic );

protected Q_SLOTS:
  void sendVel();
  void updateTopic();

protected:
  DriveWidget* drive_widget_;
  QLineEdit* output_topic_editor_;
  std::string output_topic_;
  ros::Publisher velocity_publisher_;
  ros::NodeHandle nh_;
  float linear_velocity_;
  float angular_velocity_;
};

} // end namespace rviz_plugin_tutorials

#endif // TELEOP_PANEL_H
