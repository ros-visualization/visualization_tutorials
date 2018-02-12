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
package's "package.xml" file like so:

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
      <message_type>sensor_msgs/Imu</message_type>
    </class>
    <class name="rviz_plugin_tutorials/PlantFlag"
           type="rviz_plugin_tutorials::PlantFlagTool"
           base_class_type="rviz::Tool">
      <description>
        Tool for planting flags on the ground plane in rviz.
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
name" for the class.  The "display name" is the name used for the
class in the user interface.

The *type* entry must be the fully-qualified class name, including any
namespace(s) it is inside.

The *base_class_type* is usually one of ``rviz::Panel``,
``rviz::Display``, ``rviz::Tool``, or ``rviz::ViewController``.

The *description* subsection is a simple text description of the
class, which is shown in the class-chooser dialog and in the Displays
panel help area.  This section can contain HTML, including hyperlinks,
but the markup must be escaped to avoid being interpreted as XML
markup.  For example a link tag might look like: ``&lt;a
href="my-web-page.html"&gt;``.

Display plugins can have multiple *message_type* tags, which are used
by RViz when you add a Display by selecting it's topic first.

