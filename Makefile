all:
	pushd
	cd ../..
	catkin_make
	popd

trans:
	rosrun tf view_frames && evince frames.pdf