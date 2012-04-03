ImuDisplay
==========

The RViz plugin API and library API are preliminary in Fuerte. We
welcome feedback about how to make them more powerful and easier to
program with. We expect the APIs to change (possibly significantly)
between Fuerte and Groovy.

Overview
--------

This tutorial shows how to write a simple Display plugin for RViz.

RViz does not currently have a way to display sensor_msgs/Imu messages
directly. The code in this tutorial implements a subclass of
rviz::Display to do so.

The source code for this tutorial is in the rviz_plugin_tutorials
package. You can check out the source directly or (if you use Ubuntu)
you can just apt-get install the pre-compiled Debian package like so::

    sudo apt-get install ros-fuerte-visualization-tutorials

Here is what the new ImuDisplay output looks like, showing a sequence of
sensor_msgs/Imu messages from the test script:

.. image:: imu_arrows.png

The Plugin Code
---------------

The code for ImuDisplay is in these files: 
:svndir:`src/imu_display.h`,
:svndir:`src/imu_display.cpp`,
:svndir:`src/imu_visual.h`, and
:svndir:`src/imu_visual.cpp`.

imu_display.h
^^^^^^^^^^^^^

The full text of imu_display.h is here: :svndir:`src/imu_display.h`

.. tutorial-formatter:: ../imu_display.h

imu_display.cpp
^^^^^^^^^^^^^^^

The full text of imu_display.cpp is here: :svndir:`src/imu_display.cpp`

.. tutorial-formatter:: ../imu_display.cpp

imu_visual.h
^^^^^^^^^^^^

The full text of imu_visual.h is here: :svndir:`src/imu_visual.h`

.. tutorial-formatter:: ../imu_visual.h

imu_visual.cpp
^^^^^^^^^^^^^^

The full text of imu_visual.cpp is here: :svndir:`src/imu_visual.cpp`

.. tutorial-formatter:: ../imu_visual.cpp

Building the Plugin
-------------------

.. tutorial-formatter:: ../../CMakeLists.txt

To build the plugin, just do the normal "rosmake" thing::

    rosmake rviz_plugin_tutorials

Exporting the Plugin
--------------------

For the plugin to be found and understood by other ROS packages (in
this case, rviz), it needs a "plugin_description.xml" file.  This file
can be named anything you like, as it is specified in the plugin
package's "manifest.xml" file like so:

.. code-block:: xml

  <export>
      <rviz plugin="${prefix}/plugin_description.xml"/>
  </export>

The contents of plugin_description.xml then look like this:

.. code-block:: xml

  <library path="lib/librviz_plugin_tutorials">
    <class name="rviz_plugin_tutorials/Teleop"
           type="rviz_plugin_tutorials::TeleopPanel"
           base_class_type="rviz::Panel">
      <description>
        A panel widget allowing simple diff-drive style robot base control.
      </description>
    </class>
    <class name="rviz_plugin_tutorials/Imu"
           type="rviz_plugin_tutorials::ImuDisplay"
           base_class_type="rviz::Display">
      <description>
        Displays direction and scale of accelerations from sensor_msgs/Imu messages.
      </description>
    </class>
  </library>

The first line says that the compiled library lives in
lib/librviz_plugin_tutorials (the ".so" ending is appended by
pluginlib according to the OS).  This path is relative to the top
directory of the package:

.. code-block:: xml

  <library path="lib/librviz_plugin_tutorials">

The next section is a ``class`` entry describing the TeleopPanel:

.. code-block:: xml

    <class name="rviz_plugin_tutorials/Teleop"
           type="rviz_plugin_tutorials::TeleopPanel"
           base_class_type="rviz::Panel">
      <description>
        A panel widget allowing simple diff-drive style robot base control.
      </description>
    </class>

This specifies the name, type, base class, and description of the
class.  The *name* field must be a combination of the first two
strings given to the ``PLUGINLIB_DECLARE_CLASS()`` macro in the source
file.  It must be the "package" name, a "/" slash, then the "display
name" for the class.

The *type* entry must be the fully-qualified class name, including any
namespace(s) it is inside.

The *base_class_type* is either ``rviz::Panel`` for a panel class, or
``rviz::Display`` for a display class.

The *description* subsection is a simple text description of the
class, which is shown in the class-chooser dialog and in the Displays
panel help area.  This section can contain HTML, including hyperlinks,
but the markup must be escaped to avoid being interpreted as XML
markup.  For example a link tag might look like: ``&lt;a
href="my-web-page.html"&gt;``.

Trying It Out
-------------

Once your RViz plugin is compiled and exported, simply run rviz normally::

    rosrun rviz rviz

and rviz will use pluginlib to find all the plugins exported to it.

Add an ImuDisplay by clicking the "Add" button at the bottom of the
"Displays" panel (or by typing Control-N), then scrolling down through
the available displays until you see "Imu" under your plugin package
name (here it is "rviz_plugin_tutorials").

.. image:: imu_plugin.png

If "Imu" is not in your list of Display Types, look through RViz's
console output for error messages relating to plugin loading.  Some common
problems are:

- not having a plugin_description.xml file,
- not exporting it in the manifest.xml file, or
- not properly referencing the library file (like
  librviz_plugin_tutorials.so) from plugin_description.xml.

Once you've added the Imu display to RViz, you just need to set the
topic name of the display to a source of sensor_msgs/Imu messages.

If you don't happen to have an IMU or other source of sensor_msgs/Imu
messages, you can test the plugin with a Python script like this:
:svndir:`scripts/send_test_msgs.py`.

The script publishes on the "/test_imu" topic, so enter that.

The script publishes both Imu messages and a moving TF frame
("/base_link" relative to "/map"), so make sure your "Fixed Frame" is
set to "/map".

Finally, adjust the "History Length" parameter of the Imu display to
10 and you should see something like the picture at the top of this
page.

Note: If you use this to visualize messages from an *actual* IMU, the
arrows are going to be huge compared to most robots:

.. image:: real_imu.png

(Note the PR2 robot at the base of the purple arrow.) This is because
the Imu acceleration units are meters per second squared, and gravity
is 9.8 m/s^2, and we haven't applied any scaling or gravity
compensation to the acceleration vectors.

Next Steps
----------

This ImuDisplay is not yet a terribly useful Display class.  Extensions to make it more useful might be:

- Add a gravity-compensation option to the acceleration vector.
- Visualize more of the data in the Imu messages.

To add a gravity compensation option, you might take steps like these:

- Add a new ``bool gravity_compensation_on_`` property to ImuDisplay to store whether the option is on or off.
- Add getter and setter functions for it.
- Add a new ``rviz::BoolPropertyWPtr`` to ImuDisplay to show the property in the property editor.
- Add a new ``setGravityCompensation(bool)`` function to
  ImuVisual to tell the visual whether to subtract out gravity or not.
- Compute the direction of gravity relative to the Imu frame
  orientation (as set in ImuVisual::setFrameOrientation()) and
  subtract it from the acceleration vector each time in
  ImuVisual::setMessage().

Since ImuVisual takes complete Imu messages as input, adding
visualizations of more of the Imu data only needs modifications to
ImuVisual.  Imu data displays might look like:

- orientation: An rviz::Axes object at the Imu reference frame, turned to show the orientation.
- angular_velocity: Maybe a line to show the axis of rotation and a 3D arrow curving around it to show the speed of rotation?
- orientation_covariance: Maybe this is an ellipse at the end of each of the X, Y, and Z axes showing the orientation?
- linear_acceleration_covariance: Maybe this is an ellipsoid at the end of the acceleration arrow?

As all this might be visually cluttered, it may make sense to include boolean options to enable or disable some of them.
