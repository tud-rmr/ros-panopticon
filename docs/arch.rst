Architecture
============

Frames
------

This section describes the differnt coordinate systems used. There are two special marker, *map* and *center*. *map* is the origin of the global frame (0,0), *center* has the property to be seen by all cameras. Every camera has its own frame, as do the *center* and *map*.


.. figure:: images/tf_frames.svg
    :align: center

The coordinate systems (frames) used can be seen in this illustration. The four cameras are depicted by C0 to C3 and are connected via the center marker. C0 can see the marker which represents the origin. When a marker is now seen in C1, C2 or C3, the position in the *map* frame can be computed by going C0->center->map . The arrow have to be read like *a->b* means *a is parent of b*.

For dealing with the different coordinate systems, `tf2`_ is used, the de facto standard transform library for ROS. In the current setup, *map* and *center* have to be seen at all time. If they are moved, then the coordinate system changes because the relation between them changes. But as long as they adhere to their position requirements, the package runs.


Cameras
-------

We use four ceiling mounted Logitech USB cameras. They are interfaced with the `usb_cam`_ package and launched by a custom launch file *launch/usb_cam_by_id.xml* . They have to have overlapping field of view, at least one marker, the *center*, has to be seen by all four cameras. For accurate results, the cameras have to be calibrated.

Right now, the cameras are not mounted to a fixed device address, i.e. camera IDs can change when restarting the computer. Therefore, make sure that *usb_cam0* is the one which can see the origin marker.

Marker
------

The marker used are from ArUco and detected by vanilla `ar_sys`_. We use a slightly changed multi_board version launch file (see *launch/aruco_multi.xml*). The list of boards which can be detected is in *config/boards.yml*. The origin and center marker **HAVE** to be called *map* and *center* respectively. Other marker have to be prefixed *marker*. Each board consists of two configuration files: one in pixel and one in meter. We only need the later. These can be found in *config/boards* and can be generated. To add a new board, either do the way of ArUco and use the command line tools directly or use the scripts in this package. More on that in :doc:`scripts`.

.. figure:: images/marker_344.png
    :align: center

    A typical ArUco marker. Each marker encodes a number.

A good overview on ArUco marker can be found in this `OpenCV ArUco`_ tutorial or directly on the `authors homepage`_.



Subscribed topics
-----------------

Published topics
----------------

.. _tf2: http://wiki.ros.org/tf2
.. _usb_cam: http://wiki.ros.org/usb_cam
.. _ar_sys: http://wiki.ros.org/ar_sys
.. _OpenCV ArUco: http://docs.opencv.org/3.1.0/d5/dae/tutorial_aruco_detection.html#gsc.tab=0
.. _authors homepage: http://www.uco.es/investiga/grupos/ava/node/26