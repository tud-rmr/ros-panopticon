Installing
==========

The following chapter describes the steps necessary to install this package.

Installing dependencies
-----------------------

This package requires two external ROS packages, *ar_sys* and *usb_cam*. These have binary distributions in Ubuntu and can be installed via::

	sudo apt install ros-indigo-ar-sys
	sudo apt install ros-indigo-usb-cam 

If they should be installed only locally, then one needs to download them manually in the used catkin workspace. This requires ``git`` installed and an already existing and sourced catkin workspace: ::

	cd <your_catkin_workspace>
	mkdir -p src
	cd src
	git clone https://github.com/Sahloul/ar_sys.git
	git clone https://github.com/bosch-ros-pkg/usb_cam.git
	cd ..
	catkin_make

Installing the package itself
-----------------------------

As the source code for this package is also hosted on Github, installing simply means checking its repository out:
	
	cd <your_catkin_workspace>
	mkdir -p src
	cd src
	git clone https://github.com/Rentier/ros-panopticon
	cd .. 
	catkin_make