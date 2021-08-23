Librviz Tutorial
================

Overview
--------

RViz is not just a visualizer application, it is also a library!
Much of RViz's functionality can be accessed within your own application by linking against librviz.so (or whatever your OS likes to call it).

This tutorial shows a very simple example of creating a 3D visualizer widget (rviz_common::RenderPanel), programmatically creating a new Grid display within it, then using Qt slider controls to adjust a couple of the grid's properties.
The app is called "myviz".

The source code for this tutorial is in the librviz_tutorial package.
You can check out the source directly or (if you use Ubuntu) you can just apt-get install the pre-compiled Debian package like so::

    sudo apt-get install ros-galactic-visualization-tutorials

The running application looks like this:

.. image:: myviz.png

The Code
--------

The code for myviz is in these files: 
:codedir:`src/main.cpp`,
:codedir:`src/myviz.hpp`, and
:codedir:`src/myviz.cpp`.

main.cpp
^^^^^^^^

The full text of main.cpp is here: :codedir:`src/main.cpp`

.. tutorial-formatter:: ../main.cpp

myviz.hpp
^^^^^^^^^

The full text of myviz.hpp is here: :codedir:`src/myviz.hpp`

.. tutorial-formatter:: ../myviz.hpp

myviz.cpp
^^^^^^^^^

The full text of myviz.cpp is here: :codedir:`src/myviz.cpp`

.. tutorial-formatter:: ../myviz.cpp

Building
--------

The full text of CMakeLists.txt is here: :codedir:`CMakeLists.txt`

.. tutorial-formatter:: ../../CMakeLists.txt

Running
-------

Just type::

    ros2 run librviz_tutorial myviz
