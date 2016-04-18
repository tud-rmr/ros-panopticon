Usage
=====

The goal for this package is to publish the position of previously defined markers in a global world frame. These markers can be in view of only some of the cameras. The output is therefore the position of each detected marker in world coordinates for every camera attached.

Start the package
-----------------

In order to launch the ``usb_cam`` and ``ar_sys`` nodes together with the ``panopticon`` node, a handy launch file is provided. Just run the following in the root of the package: ::

	roscd panopticon
	roslaunch launch/panopticon.xml

Make sure that the ``map`` marker can be seen by ``usb_cam0``. Make sure that the ``center`` marker can be seen by all four cameras. The pose of all other markers defined and detected then are published under ``pose/marker$(markerId)/cam$[1-4]`` .

Configuration
-------------

``usb_cam`` and ``ar_sys`` can be heavily configured. See their respective docmentations and change the launch files under **launch** as your require.

Adding markers
--------------

There are two steps needed to add markers: Generate a ``board_configuration`` and add an entry to ``config\boards.yml`` . Make sure that you specify the right ID and right size. A script has been provided to generate the configuration, see :doc:`scripts` for more. Markers used for localization have to be prefixed with ``marker`` !

