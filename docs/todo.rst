ToDo
==========

Camera mount point
------------------

At the moment, the cameras are not mounted to a fixed device adress. In order to make sure that the origin is always in the same camera, fix the mount by udev rules. A guide for that can be found in `libuvc_camera`_ . The device ID of the cameras can be found by the following neat snippet: ::
	
	sudo lsusb -v -d 046d:082c | grep -i serial

Easiest way to get it done is removing all cameras but one, check which area it sees, and then assign a fixed device ID. Repeat until all for have been assigned.

Data fusion
-----------

Right now, for every marker of interest, the system returns at most four estimated positions (depends on how many cameras see the marker). To make it more accurate and useful, this information has to be merged into one. There are many different approaches, from ad-hoc trial-and-error to dedicated ROS packages, like `robot_localization`_ . A launch file for that has already been started, some work is missing there. Another idea is to write a custom node that weights a (moving) average with the time of the positions and the distance to the corners.

Calibration
-----------

The cameras have to be calibrated in order to get a good position estimation. In the current setup, only one camera is calibrated, all other use the same data. To make it more accurate, each camera has to be calibrated on its own. This calibration can best be stored in a `calibration` folder in the package and given to the *aruco_multi* launch file.

Visualization
-------------

In order to get a grasp about the marker positions in the world grid, a good visualization is handy. The camera, map and center frames itself can be already seen in rviz (see :doc:`page` for an example). The marker are not yet there and have to be implemented. That can be done by sending messages to *rviz*, as described for instance in this `rviz tutorial`_.

.. _robot_localization: http://wiki.ros.org/robot_localization
.. _libuvc_camera: http://wiki.ros.org/libuvc_camera
.. _rviz tutorial: http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes

Make map and center marker removable
-----------------------------

In the current setup, the *map* and *center* marker have to be always in position. But in general, they have to be only detected once to get the relation between the cameras. This can be saved and published to keep the frame in tf2 active, whereas the marker can then be removed until the next restart of the package. This also prevents errors like occlusion of the center marker by a robot.




