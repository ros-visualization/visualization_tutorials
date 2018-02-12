#!/usr/bin/env python

import roslib; roslib.load_manifest( 'rviz_plugin_tutorials' )
from sensor_msgs.msg import Imu
import rospy
from math import cos, sin
import tf

topic = 'test_imu'
publisher = rospy.Publisher( topic, Imu, queue_size=5 )

rospy.init_node( 'test_imu' )

br = tf.TransformBroadcaster()
rate = rospy.Rate(10)
radius = 5
angle = 0

dist = 3
while not rospy.is_shutdown():

    imu = Imu()
    imu.header.frame_id = "/base_link"
    imu.header.stamp = rospy.Time.now()
   
    imu.linear_acceleration.x = sin( 10 * angle )
    imu.linear_acceleration.y = sin( 20 * angle )
    imu.linear_acceleration.z = sin( 40 * angle )

    publisher.publish( imu )

    br.sendTransform((radius * cos(angle), radius * sin(angle), 0),
                     tf.transformations.quaternion_from_euler(0, 0, angle),
                     rospy.Time.now(),
                     "base_link",
                     "map")
    angle += .01
    rate.sleep()

