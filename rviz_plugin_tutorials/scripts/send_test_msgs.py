#!/usr/bin/env python3

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from math import cos, sin
from threading import Thread

from geometry_msgs.msg import Quaternion, TransformStamped
import rclpy
from sensor_msgs.msg import Imu
from tf2_ros.transform_broadcaster import TransformBroadcaster


def quaternion_from_euler(yaw, pitch, roll):
    quat = Quaternion()
    quat.x = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    quat.y = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    quat.z = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    quat.w = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return quat


rclpy.init()

topic = 'test_imu'
node = rclpy.create_node('test_imu_node')
publisher = node.create_publisher(Imu, topic, 5)

br = TransformBroadcaster(node)

# Spin in a separate thread
thread = Thread(target=rclpy.spin, args=(node, ), daemon=True)
thread.start()
rate = node.create_rate(10)

radius = 5
angle = 0
dist = 3

while rclpy.ok():
    imu = Imu()
    imu.header.frame_id = '/base_link'
    imu.header.stamp = node.get_clock().now().to_msg()

    imu.linear_acceleration.x = sin(10 * angle)
    imu.linear_acceleration.y = sin(20 * angle)
    imu.linear_acceleration.z = sin(40 * angle)

    publisher.publish(imu)

    tf = TransformStamped()
    tf.header.frame_id = '/map'
    tf.header.stamp = node.get_clock().now().to_msg()
    tf.child_frame_id = '/base_link'

    tf.transform.translation.x = radius * cos(angle)
    tf.transform.translation.y = radius * sin(angle)
    tf.transform.translation.z = 0.0
    tf.transform.rotation = quaternion_from_euler(0, 0, angle)
    br.sendTransform(tf)

    angle += .01
    rate.sleep()

rclpy.shutdown()
thread.join()
