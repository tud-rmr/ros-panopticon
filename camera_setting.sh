#!/bin/bash
# Script camera settings


#set camera0 parameters
v4l2-ctl -d /dev/camera0 -c exposure_auto=1
v4l2-ctl -d /dev/camera0 -c exposure_auto_priority=0
v4l2-ctl -d /dev/camera0 -c exposure_absolute=30
v4l2-ctl -d /dev/camera0 -c sharpness=31
v4l2-ctl -d /dev/camera0 -c focus_auto=0
v4l2-ctl -d /dev/camera0 -c focus_absolute=0
v4l2-ctl -d /dev/camera0 -c power_line_frequency=1
v4l2-ctl -d /dev/camera0 -c brightness=55
v4l2-ctl -d /dev/camera0 -c contrast=75
v4l2-ctl -d /dev/camera0 -c gain=60



#set camera1 parameters
v4l2-ctl -d /dev/camera1 -c exposure_auto=1
v4l2-ctl -d /dev/camera1 -c exposure_auto_priority=0
v4l2-ctl -d /dev/camera1 -c exposure_absolute=30
v4l2-ctl -d /dev/camera1 -c sharpness=31
v4l2-ctl -d /dev/camera1 -c focus_auto=0
v4l2-ctl -d /dev/camera1 -c focus_absolute=0
v4l2-ctl -d /dev/camera1 -c power_line_frequency=1
v4l2-ctl -d /dev/camera1 -c brightness=55
v4l2-ctl -d /dev/camera1 -c contrast=75
v4l2-ctl -d /dev/camera1 -c gain=60

#set camera2 parameters
v4l2-ctl -d /dev/camera2 -c exposure_auto=1
v4l2-ctl -d /dev/camera2 -c exposure_auto_priority=0
v4l2-ctl -d /dev/camera2 -c exposure_absolute=50
v4l2-ctl -d /dev/camera2 -c sharpness=31
v4l2-ctl -d /dev/camera2 -c focus_auto=0
v4l2-ctl -d /dev/camera2 -c focus_absolute=0
v4l2-ctl -d /dev/camera2 -c power_line_frequency=1
v4l2-ctl -d /dev/camera2 -c brightness=100
v4l2-ctl -d /dev/camera2 -c contrast=43
v4l2-ctl -d /dev/camera2 -c gain=60

#set camera3 parameters
v4l2-ctl -d /dev/camera3 -c exposure_auto=1
v4l2-ctl -d /dev/camera3 -c exposure_auto_priority=0
v4l2-ctl -d /dev/camera3 -c exposure_absolute=50
v4l2-ctl -d /dev/camera3 -c sharpness=31
v4l2-ctl -d /dev/camera3 -c focus_auto=0
v4l2-ctl -d /dev/camera3 -c focus_absolute=0
v4l2-ctl -d /dev/camera3 -c power_line_frequency=1
v4l2-ctl -d /dev/camera3 -c brightness=100
v4l2-ctl -d /dev/camera3 -c contrast=43
v4l2-ctl -d /dev/camera3 -c gain=65



# set focus in the height next to ground
#v4l2-ctl --device=/dev/camera0 --set-ctrl=focus_absolute=17
#v4l2-ctl --device=/dev/camera1 --set-ctrl=focus_absolute=17
#v4l2-ctl --device=/dev/camera2 --set-ctrl=focus_absolute=17
#v4l2-ctl --device=/dev/camera3 --set-ctrl=focus_absolute=17

# set exposure time
#v4l2-ctl --device=/dev/camera0 --set-ctrl=exposure_absolute=20
#v4l2-ctl --device=/dev/camera1 --set-ctrl=exposure_absolute=20
#v4l2-ctl --device=/dev/camera2 --set-ctrl=exposure_absolute=20
#v4l2-ctl --device=/dev/camera3 --set-ctrl=exposure_absolute=20
