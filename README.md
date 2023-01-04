# visualization_tutorials

This repository contains the following tutorial packages for visualization in RViz:

1. `rviz_plugin_tutorials`
    - shows how to develop plugins for RViz including displays, panels, and tools.
2. `visualization_marker_tutorials`
    - shows how to use the Marker Display to render basic markers.
3. `interactive_marker_tutorials`
    - shows how to use the Interactive Marker Display to render markers that the user can interact with.
4. `librviz_tutorial`
    - shows how to write your own RViz application.

To build the tutorial documentation for each package, run the following command:

```
$ sphinx-build -b html src_dir build_dir
```

Where `src_dir` is the path to the package docs and `build_dir` is the directory where you want to build the docs.
