#!/bin/bash
# Script camera settings

# set focus in the height next to ground
v4l2-ctl --device=/dev/video0 --set-ctrl=focus_absolute=17
v4l2-ctl --device=/dev/video1 --set-ctrl=focus_absolute=17
v4l2-ctl --device=/dev/video2 --set-ctrl=focus_absolute=17
v4l2-ctl --device=/dev/video3 --set-ctrl=focus_absolute=17

# set exposure time
v4l2-ctl --device=/dev/video0 --set-ctrl=exposure_absolute=20
v4l2-ctl --device=/dev/video1 --set-ctrl=exposure_absolute=20
v4l2-ctl --device=/dev/video2 --set-ctrl=exposure_absolute=20
v4l2-ctl --device=/dev/video3 --set-ctrl=exposure_absolute=20
