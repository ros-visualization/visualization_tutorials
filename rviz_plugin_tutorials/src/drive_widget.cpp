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
#include <math.h>

#include <QPainter>
#include <QMouseEvent>

#include "drive_widget.h"

namespace rviz_plugin_tutorials
{

DriveWidget::DriveWidget( QWidget* parent )
  : QWidget( parent )
  , linear_max_( 10 )
  , angular_max_( 10 )
{
}

void DriveWidget::paintEvent( QPaintEvent* event )
{
  QColor background;
  QColor crosshair;
  if( isEnabled() )
  {
    background = Qt::white;
    crosshair = Qt::black;
  }
  else
  {
    background = Qt::lightGray;
    crosshair = Qt::darkGray;
  }
  QPainter painter( this );
  painter.setBrush( background );
  painter.drawRect( rect() );
  painter.setPen( crosshair );
  painter.drawLine( 0, height() / 2, width(), height() / 2 );
  painter.drawLine( width() / 2, 0, width() / 2, height() );

  if( isEnabled() && (angular_velocity_ != 0 || linear_velocity_ != 0 ))
  {
    int w = width();
    int h = height();

    float lin = h/2 * linear_velocity_ / linear_max_;
    float ang = angular_velocity_;

    QPen arrow;
    int line_width = 2 + abs( int( lin / 5.0 ));
    arrow.setWidth( h/20 );
    arrow.setColor( Qt::green );
    arrow.setCapStyle( Qt::RoundCap );
    painter.setPen( arrow );

    int arrowhead_x, arrowhead_y;
    float arrowhead_ang;

    if( angular_velocity_ == 0 )
    {
      painter.drawLine( w/2, h/2, w/2, h/2 - lin );
      arrowhead_x = w/2;
      arrowhead_y = h/2 - lin;
      if( lin < 0 )
      {
        arrowhead_ang = M_PI;
      }
      else
      {
        arrowhead_ang = 0;
      }
    }
    else
    {
      int radius = abs( int( lin / ang ));
      int arc_pi = 180 * 16;
      int start_ang;
      int span_ang = abs( int( ang*180.0/M_PI*16 ));
      int x;
      if( ang > 0 )
      {
        if( lin > 0 )
        {
          x = w/2 - 2*radius;
          start_ang = 0;
          arrowhead_ang = ang;
          arrowhead_x = x + radius + radius * cosf( ang );
          arrowhead_y = h/2 - radius * sinf( ang );
        }
        else
        {
          x = w/2;
          start_ang = arc_pi;
          arrowhead_ang = M_PI + ang;
          arrowhead_x = x + radius - radius * cosf( ang );
          arrowhead_y = h/2 + radius * sinf( ang );
        }
      }
      else
      {
        if( lin > 0 )
        {
          x = w/2;
          start_ang = arc_pi - span_ang;
          arrowhead_ang = ang;
          arrowhead_x = x + radius - radius * cosf( -ang );
          arrowhead_y = h/2 - radius * sinf( -ang );
        }
        else
        {
          x = w/2 - 2*radius;
          start_ang = -span_ang;
          arrowhead_ang = M_PI + ang;
          arrowhead_x = x + radius + radius * cosf( -ang );
          arrowhead_y = h/2 + radius * sinf( -ang );
        }
      }
      painter.drawArc( x, h/2 - radius, 2*radius, 2*radius, start_ang, span_ang );
    }
    float tip_spread = M_PI/3;
    float head_l_ang = arrowhead_ang + M_PI * 1.5 - tip_spread / 2;
    float head_r_ang = arrowhead_ang + M_PI * 1.5 + tip_spread / 2;
    float head_size = line_width * 2;
    painter.drawLine( arrowhead_x + head_size * cosf( head_l_ang ), arrowhead_y - head_size * sinf( head_l_ang ),
                      arrowhead_x, arrowhead_y );
    painter.drawLine( arrowhead_x, arrowhead_y,
                      arrowhead_x + head_size * cosf( head_r_ang ), arrowhead_y - head_size * sinf( head_r_ang ));
  }
}

void DriveWidget::mouseMoveEvent( QMouseEvent* event )
{
  sendVelocitiesFromMouse( event->x(), event->y(), width(), height() );
}

void DriveWidget::mousePressEvent( QMouseEvent* event )
{
  sendVelocitiesFromMouse( event->x(), event->y(), width(), height() );
}

void DriveWidget::sendVelocitiesFromMouse( int x, int y, int width, int height )
{  
  linear_velocity_ = (1.0 - float( y ) / float( height / 2 )) * linear_max_;
  angular_velocity_ = (1.0 - float( x ) / float( width / 2 )) * angular_max_;
  update();
  Q_EMIT outputVelocity( linear_velocity_, angular_velocity_ );
}

void DriveWidget::mouseReleaseEvent( QMouseEvent* event )
{
  linear_velocity_ = 0;
  angular_velocity_ = 0;
  update();
  Q_EMIT outputVelocity( linear_velocity_, angular_velocity_ );
}

} // end namespace rviz_plugin_tutorials
