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

// #include <Eigen/Dense>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/property.h>
#include <rviz/properties/property_manager.h>
#include <rviz/frame_manager.h>
#include <rviz/validate_floats.h>
#include <rviz/ogre_helpers/shape.h>

#include "covariance_ellipsoid_display.h"

namespace rviz_plugin_tutorials
{

CovarianceEllipsoidDisplay::CovarianceEllipsoidDisplay()
  : Display()
  , messages_received_( 0 )
  , scene_node_( NULL )
  , color_( .8, .2, .8 )
  , alpha_( 1.0 )
{
}

void CovarianceEllipsoidDisplay::onInitialize()
{
  tf_filter_ = new tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>( *vis_manager_->getTFClient(), "", 100, update_nh_ );

  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  scene_node_->setVisible( false );
  
  setHistoryLength( 1 );
  Ogre::Vector3 scale( 0, 0, 0 );

  tf_filter_->connectInput( sub_ );
  tf_filter_->registerCallback( boost::bind( &CovarianceEllipsoidDisplay::incomingMessage, this, _1 ));
  vis_manager_->getFrameManager()->registerFilterForTransformStatusCheck( tf_filter_, this );
}

CovarianceEllipsoidDisplay::~CovarianceEllipsoidDisplay()
{
  unsubscribe();
  clear();
  for( size_t i = 0; i < ellipsoids_.size(); i++ )
  {
    delete ellipsoids_[ i ];
  }

  delete tf_filter_;
}

void CovarianceEllipsoidDisplay::clear()
{
  for( size_t i = 0; i < ellipsoids_.size(); i++ )
  {
    delete ellipsoids_[ i ];
    ellipsoids_[ i ] = NULL;
  }
  tf_filter_->clear();
  messages_received_ = 0;
  setStatus( rviz::status_levels::Warn, "Topic", "No messages received" );
}

void CovarianceEllipsoidDisplay::setTopic( const std::string& topic )
{
  unsubscribe();

  topic_ = topic;

  subscribe();

  propertyChanged( topic_property_ );

  causeRender();
}

void CovarianceEllipsoidDisplay::setColor( const rviz::Color& color )
{
  color_ = color;

  propertyChanged( color_property_ );

  updateColorAndAlpha();

  causeRender();
}

void CovarianceEllipsoidDisplay::updateColorAndAlpha()
{
  for( size_t i = 0; i < ellipsoids_.size(); i++ ) {
    if( ellipsoids_[ i ] )
    {
      ellipsoids_[ i ]->setColor( color_.r_, color_.g_, color_.b_, alpha_ );
    }
  }
}

void CovarianceEllipsoidDisplay::setHistoryLength( int length )
{
  if( length < 1 )
  {
    length = 1;
  }
  if( history_length_ == length )
  {
    return;
  }
  history_length_ = length;

  propertyChanged( history_length_property_ );
  
  // Create a new array of ellipsoid pointers, all NULL.
  std::vector<rviz::Shape*> new_ellipsoids( history_length_, NULL );

  // Copy the contents from the old array to the new.
  size_t copy_len = ( new_ellipsoids.size() > ellipsoids_.size() ) ? ellipsoids_.size() : new_ellipsoids.size(); // minimum of 2 lengths
  for( size_t i = 0; i < copy_len; i++ )
  {
    int new_index = (messages_received_ - i) % new_ellipsoids.size();
    int old_index = (messages_received_ - i) % ellipsoids_.size();
    new_ellipsoids[ new_index ] = ellipsoids_[ old_index ];
    ellipsoids_[ old_index ] = NULL;
  }

  // Delete any remaining old ellipsoids
  for( size_t i = 0; i < ellipsoids_.size(); i++ ) {
    delete ellipsoids_[ i ];
  }

  // Put the new vector into the member variable version and let the
  // old one go out of scope.
  ellipsoids_.swap( new_ellipsoids );
}

void CovarianceEllipsoidDisplay::setAlpha( float alpha )
{
  alpha_ = alpha;

  propertyChanged( alpha_property_ );

  updateColorAndAlpha();

  causeRender();
}

void CovarianceEllipsoidDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  try
  {
    sub_.subscribe( update_nh_, topic_, 10 );
    setStatus( rviz::status_levels::Ok, "Topic", "OK" );
  }
  catch( ros::Exception& e )
  {
    setStatus( rviz::status_levels::Error, "Topic", std::string( "Error subscribing: " ) + e.what() );
  }
}

void CovarianceEllipsoidDisplay::unsubscribe()
{
  sub_.unsubscribe();
}

void CovarianceEllipsoidDisplay::onEnable()
{
  scene_node_->setVisible( true );
  subscribe();
}

void CovarianceEllipsoidDisplay::onDisable()
{
  unsubscribe();
  clear();
  scene_node_->setVisible( false );
}

void CovarianceEllipsoidDisplay::fixedFrameChanged()
{
  tf_filter_->setTargetFrame( fixed_frame_ );
  clear();
}

void CovarianceEllipsoidDisplay::incomingMessage( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg )
{
  ++messages_received_;
  
  rviz::Shape* ellipsoid = ellipsoids_[ messages_received_ % history_length_ ];
  if( ellipsoid == NULL )
  {
    ellipsoid = new rviz::Shape( rviz::Shape::Sphere, vis_manager_->getSceneManager(), scene_node_ );
    ellipsoids_[ messages_received_ % history_length_ ] = ellipsoid;
  }

  std::stringstream ss;
  ss << messages_received_ << " messages received";
  setStatus( rviz::status_levels::Ok, "Topic", ss.str() );

#if 0
  // Copy the 3D position part of the covariance matrix.
  Eigen::Matrix3f cov;
  cov << msg->pose.covariance[ 0 ] << msg->pose.covariance[ 1 ] << msg->pose.covariance[ 2 ]
      << msg->pose.covariance[ 6 ] << msg->pose.covariance[ 7 ] << msg->pose.covariance[ 8 ]
      << msg->pose.covariance[ 12 ] << msg->pose.covariance[ 13 ] << msg->pose.covariance[ 14 ];

  // Compute
  Eigen::SelfAdjointEigenSolver<Matrix3f> eigensolver( cov );
  if( eigensolver.info() != Eigen::Success )
  {
    setStatus( rviz::status_levels::Error, "Covariance", std::string( "Could not find eigenvalues of incoming covariance matrix." ));
    return;
  }
  const Eigen::RealVectorType& eigenvalues = eigensolver.eigenvalues();
  const Eigen::MatrixType& eigenvectors = eigensolver.eigenvectors();

//   cout << "Here's a matrix whose columns are eigenvectors of A \n"
//        << "corresponding to these eigenvalues:\n"
//        << eigensolver.eigenvectors() << endl;
  ................. TODO: finish this conversion..........
#endif // 0

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if( !vis_manager_->getFrameManager()->transform( msg->header.frame_id, msg->header.stamp, msg->pose.pose, position, orientation ))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), fixed_frame_.c_str() );
  }

  ellipsoid->setPosition( position );
  ellipsoid->setOrientation( orientation );

  Ogre::Vector3 scale( 1, 1, 1 );
  ellipsoid->setScale( scale );
  ellipsoid->setColor( color_.r_, color_.g_, color_.b_, alpha_ );
}

void CovarianceEllipsoidDisplay::reset()
{
  Display::reset();
  clear();
}

void CovarianceEllipsoidDisplay::createProperties()
{
  topic_property_ =
    property_manager_->createProperty<rviz::ROSTopicStringProperty>( "Topic",
                                                                     property_prefix_,
                                                                     boost::bind( &CovarianceEllipsoidDisplay::getTopic, this ),
                                                                     boost::bind( &CovarianceEllipsoidDisplay::setTopic, this, _1 ),
                                                                     parent_category_,
                                                                     this );
  setPropertyHelpText( topic_property_, "geometry_msgs::PoseWithCovarianceStamped topic to subscribe to." );
  rviz::ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType( ros::message_traits::datatype<geometry_msgs::PoseWithCovarianceStamped>() );

  color_property_ =
    property_manager_->createProperty<rviz::ColorProperty>( "Color",
                                                            property_prefix_,
                                                            boost::bind( &CovarianceEllipsoidDisplay::getColor, this ),
                                                            boost::bind( &CovarianceEllipsoidDisplay::setColor, this, _1 ),
                                                            parent_category_,
                                                            this );
  setPropertyHelpText( color_property_, "Color to draw the ellipsoids." );

  alpha_property_ =
    property_manager_->createProperty<rviz::FloatProperty>( "Alpha",
                                                            property_prefix_,
                                                            boost::bind( &CovarianceEllipsoidDisplay::getAlpha, this ),
                                                            boost::bind( &CovarianceEllipsoidDisplay::setAlpha, this, _1 ),
                                                            parent_category_,
                                                            this );
  setPropertyHelpText( alpha_property_, "Amount of transparency to use with the ellipsoids." );

  history_length_property_ =
    property_manager_->createProperty<rviz::IntProperty>( "History Length",
                                                          property_prefix_,
                                                          boost::bind( &CovarianceEllipsoidDisplay::getHistoryLength, this ),
                                                          boost::bind( &CovarianceEllipsoidDisplay::setHistoryLength, this, _1 ),
                                                          parent_category_,
                                                          this );
  setPropertyHelpText( history_length_property_, "Number of prior measurements to display." );
}

} // end namespace rviz_plugin_tutorials

// Tell pluginlib about this class.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS( rviz_plugin_tutorials, CovarianceEllipsoid, rviz_plugin_tutorials::CovarianceEllipsoidDisplay, rviz::Display )
