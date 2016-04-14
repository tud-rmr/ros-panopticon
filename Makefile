SHELL := /bin/bash

all:
	pushd ../..
	catkin_make
	popd

trans:
	rosrun tf view_frames && evince frames.pdf