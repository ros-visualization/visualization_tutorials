^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package librviz_tutorial
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.11.0 (2020-05-13)
-------------------

0.10.4 (2020-05-13)
-------------------
* Updated required CMake version to avoid CMP0048 warning (`#57 <https://github.com/ros-visualization/visualization_tutorials/issues/57>`_)
* Contributors: Alejandro Hernández Cordero

0.10.3 (2018-05-09)
-------------------

0.10.2 (2018-01-05)
-------------------
* Unified find_package for Qt4 and Qt5. (`#33 <https://github.com/ros-visualization/visualization_tutorials//issues/33>`_)
* Contributors: Robert Haschke, William Woodall

0.10.1 (2016-04-21)
-------------------
* Added qt5 dependencies to the package.xml.
* Contributors: William Woodall

0.10.0 (2016-04-21)
-------------------
* Added support Qt5 in Kinetic.
* Contributors: William Woodall

0.9.2 (2015-09-21)
------------------

0.9.1 (2015-01-26)
------------------
* Renamed a CMake variable to avoid colliding with built-in name.
* librviz_tutorial now installs it's executable ``myviz``.
* Removed explicit default_plugin library to fix "ld: cannot find -ldefault_plugin" isolated build error
* Contributors: Honore Doktorr, Kei Okada, William Woodall

0.9.0 (2014-03-24)
------------------
* set myself (william) as maintainer
* Contributors: William Woodall
