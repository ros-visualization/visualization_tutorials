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

#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <geometry_msgs/Twist.h>

#include "drive_widget.h"
#include "teleop_panel.h"

namespace rviz_plugin_tutorials
{

TeleopPanel::TeleopPanel( QWidget* parent )
  : Panel( parent )
  , linear_velocity_( 0 )
  , angular_velocity_( 0 )
{
  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget( new QLabel( "Output Topic:" ));
  output_topic_editor_ = new QLineEdit;
  topic_layout->addWidget( output_topic_editor_ );

  drive_widget_ = new DriveWidget;
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout( topic_layout );
  layout->addWidget( drive_widget_ );
  setLayout( layout );

  QTimer* output_timer = new QTimer( this );

  connect( drive_widget_, SIGNAL( outputVelocity( float, float )), this, SLOT( setVel( float, float )));
  connect( output_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));

  output_timer->start( 100 );
  setTopic( "" );
}

void TeleopPanel::setVel( float lin, float ang )
{
  linear_velocity_ = lin;
  angular_velocity_ = ang;
}

void TeleopPanel::updateTopic()
{
  setTopic( output_topic_editor_->text().toStdString() );
}

void TeleopPanel::setTopic( const std::string& new_topic )
{
  if( new_topic != output_topic_ )
  {
    output_topic_ = new_topic;
    if( output_topic_ == "" )
    {
      velocity_publisher_.shutdown();
    }
    else
    {
      velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>( output_topic_, 1 );
    }
  }

  drive_widget_->setEnabled( output_topic_ != "" );
}

void TeleopPanel::sendVel()
{
  if( ros::ok() && velocity_publisher_ )
  {
    geometry_msgs::Twist msg;
    msg.linear.x = linear_velocity_;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = angular_velocity_;
    velocity_publisher_.publish( msg );
  }
}

void TeleopPanel::saveToConfig( const std::string& key_prefix, const boost::shared_ptr<rviz::Config>& config )
{
  config->set( key_prefix + "/Topic", output_topic_ );
}

void TeleopPanel::loadFromConfig( const std::string& key_prefix, const boost::shared_ptr<rviz::Config>& config )
{
  std::string topic;
  config->get( key_prefix + "/Topic", &topic );
  output_topic_editor_->setText( QString::fromStdString( topic ));
  updateTopic();
}

} // end namespace rviz_plugin_tutorials
