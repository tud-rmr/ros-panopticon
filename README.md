# ros-panopticon

This package is used to recognize robotinos with markers on their top. The infrastructure in which it will be used has four ceiling cameras which overlapping field of view. Panopticon detecs the robots and gives the position in a global world frame. 

##

	roslaunch launch/panopticon.launch

## Marker

Origin: 122
Middle: 152

122
152
344
591

To add additional markers, generate them with 

	./create_board.sh <MARKER_ID>

from the *scripts* folder. Put the boardconfig into the *config/boards* folder and add a line to *config/boards.yml*. One can adjust which marker is origin, middle and robot in *config/boards.yml*.

## Good to know
	
- When the FPS is too high, the USB is overwhelmed and camera nodes die with (select timeout). Therefore, the framerate was adjusted to 15 FPS. Feel free to experiment with it.

## Helpful commands

### Get USB camera serial numbers (-d vendor:product)

	sudo lsusb -v -d 046d:082c | grep -i serial

## Todo

fancy drawing of coordinate system

