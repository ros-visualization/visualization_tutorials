#!/usr/bin/env python

"""
Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

import roslib; roslib.load_manifest("interactive_markers")
import rospy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *

server = None
marker_pos = 0

menu_handler = MenuHandler()

h_first_entry = 0
h_mode_last = 0

def enableCb( feedback ):
    handle = feedback.menu_entry_id
    state = menu_handler.getCheckState( handle )

    if state == MenuHandler.CHECKED:
        menu_handler.setCheckState( handle, MenuHandler.UNCHECKED )
        rospy.loginfo("Hiding first menu entry")
        menu_handler.setVisible( h_first_entry, False )
    else:
        menu_handler.setCheckState( handle, MenuHandler.CHECKED )
        rospy.loginfo("Showing first menu entry")
        menu_handler.setVisible( h_first_entry, True )

    menu_handler.reApply( server )
    rospy.loginfo("update")
    server.applyChanges()

def modeCb(feedback):
    global h_mode_last
    menu_handler.setCheckState( h_mode_last, MenuHandler.UNCHECKED )
    h_mode_last = feedback.menu_entry_id
    menu_handler.setCheckState( h_mode_last, MenuHandler.CHECKED )

    rospy.loginfo("Switching to menu entry #" + str(h_mode_last))
    menu_handler.reApply( server )
    print "DONE"
    server.applyChanges()

def makeBox( msg ):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0

    return marker

def makeBoxControl( msg ):
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeBox(msg) )
    msg.controls.append( control )
    return control

def makeEmptyMarker( dummyBox=True ):
    global marker_pos
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "/base_link"
    int_marker.pose.position.y = -3.0 * marker_pos
    marker_pos += 1
    int_marker.scale = 1
    return int_marker

def makeMenuMarker( name ):
    int_marker = makeEmptyMarker()
    int_marker.name = name

    control = InteractiveMarkerControl()

    control.interaction_mode = InteractiveMarkerControl.BUTTON
    control.always_visible = True

    control.markers.append( makeBox( int_marker ) )
    int_marker.controls.append(control)

    server.insert( int_marker )

def deepCb( feedback ):
    rospy.loginfo("The deep sub-menu has been found.")

def initMenu():
    global h_first_entry, h_mode_last
    h_first_entry = menu_handler.insert( "First Entry" )
    entry = menu_handler.insert( "deep", parent=h_first_entry)
    entry = menu_handler.insert( "sub", parent=entry );
    entry = menu_handler.insert( "menu", parent=entry, callback=deepCb );

    menu_handler.setCheckState( menu_handler.insert( "Show First Entry", callback=enableCb ), MenuHandler.CHECKED )

    sub_menu_handle = menu_handler.insert( "Switch" )
    for i in range(5):
        s = "Mode " + str(i)
        h_mode_last = menu_handler.insert( s, parent=sub_menu_handle, callback=modeCb )
        menu_handler.setCheckState( h_mode_last, MenuHandler.UNCHECKED)
    # check the very last entry
    menu_handler.setCheckState( h_mode_last, MenuHandler.CHECKED )

if __name__=="__main__":
    rospy.init_node("menu")
    
    server = InteractiveMarkerServer("menu")

    initMenu()
    
    makeMenuMarker( "marker1" )
    makeMenuMarker( "marker2" )

    menu_handler.apply( server, "marker1" )
    menu_handler.apply( server, "marker2" )
    server.applyChanges()

    rospy.spin()

