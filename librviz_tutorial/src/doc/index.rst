Librviz Tutorial
================

RViz is not just a visualizer application, it is also a library!  Much
of RViz's functionality can be accessed within your own application by
linking against librviz.so (or whatever your OS likes to call it).

This tutorial shows a very simple example of creating a 3D visualizer
widget (rviz::RenderPanel), programmatically creating a new Grid
display within it, then using Qt slider controls to adjust a couple of
the grid's properties.
