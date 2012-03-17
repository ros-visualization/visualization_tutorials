#!/usr/bin/env python

import roslib; roslib.load_manifest( 'rviz_plugin_tutorials' )
from geometry_msgs.msg import PoseWithCovarianceStamped
import rospy
from math import cos, sin

topic = 'test_pwcs'
publisher = rospy.Publisher( topic, PoseWithCovarianceStamped )

rospy.init_node( 'pwcs_test' )

dist = 3
r = 5
angle = 0
while not rospy.is_shutdown():

   p = PoseWithCovarianceStamped()
   p.header.frame_id = "/base_link"
   p.header.stamp = rospy.Time.now()
   
   p.pose.pose.position.x = r * cos( angle )
   p.pose.pose.position.y = r * sin( angle )
   p.pose.pose.position.z = 0
   p.pose.pose.orientation.x = 0
   p.pose.pose.orientation.y = 0
   p.pose.pose.orientation.z = 0
   p.pose.pose.orientation.w = 1

   # Remember covariance matrices are symmetric.
   p.pose.covariance = [ sin(2*angle), sin(3*angle), sin(4*angle), 0, 0, 0,
                         sin(3*angle), sin(5*angle), sin(6*angle), 0, 0, 0,
                         sin(4*angle), sin(6*angle), sin(7*angle), 0, 0, 0,
                         0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0 ]

   publisher.publish( p )

   rospy.sleep(0.1)

   angle += 0.02
