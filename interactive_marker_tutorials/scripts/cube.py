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

from math import sqrt
import sys

from geometry_msgs.msg import Pose
from interactive_markers import InteractiveMarkerServer
import rclpy
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.msg import Marker

g_logger = None
positions = []


def processFeedback(feedback):
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        # compute difference vector for this cube
        x = feedback.pose.position.x
        y = feedback.pose.position.y
        z = feedback.pose.position.z
        index = int(feedback.marker_name)

        if index > len(positions):
            return

        dx = x - positions[index][0]
        dy = y - positions[index][1]
        dz = z - positions[index][2]

        # move all markers in that direction
        for i in range(len(positions)):
            (mx, my, mz) = positions[i]
            d = sqrt(sqrt((x - mx)**2 + (y - my)**2)**2 + (z - mz)**2)
            t = 1 / (d * 5.0 + 1.0) - 0.2
            if t < 0.0:
                t = 0.0
            positions[i][0] += t * dx
            positions[i][1] += t * dy
            positions[i][2] += t * dz

            if i == index:
                g_logger.info(str(d))
                positions[i][0] = x
                positions[i][1] = y
                positions[i][2] = z

            pose = Pose()
            pose.position.x = positions[i][0]
            pose.position.y = positions[i][1]
            pose.position.z = positions[i][2]

            server.setPose(str(i), pose)
        server.applyChanges()


def makeBoxControl(msg):
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.orientation_mode = InteractiveMarkerControl.VIEW_FACING
    control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    control.independent_marker_orientation = True

    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = msg.scale
    marker.scale.y = msg.scale
    marker.scale.z = msg.scale
    marker.color.r = 0.65 + 0.7 * msg.pose.position.x
    marker.color.g = 0.65 + 0.7 * msg.pose.position.y
    marker.color.b = 0.65 + 0.7 * msg.pose.position.z
    marker.color.a = 1.0

    control.markers.append(marker)
    msg.controls.append(control)
    return control


def makeCube():
    side_length = 10
    step = 1.0 / side_length
    count = 0

    for i in range(side_length):
        x = -0.5 + step * i
        for j in range(side_length):
            y = -0.5 + step * j
            for k in range(side_length):
                z = step * k
                marker = InteractiveMarker()
                marker.header.frame_id = 'base_link'
                marker.scale = step

                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = z

                positions.append([x, y, z])

                marker.name = str(count)
                makeBoxControl(marker)

                server.insert(marker, feedback_callback=processFeedback)
                count += 1


if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('cube')
    g_logger = node.get_logger()

    server = InteractiveMarkerServer(node, 'cube')

    makeCube()
    server.applyChanges()

    rclpy.spin(node)
    server.shutdown()
