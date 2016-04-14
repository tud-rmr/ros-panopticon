#! /bin/env sh

rosbag record -b 0 --duration=10s -e "/usb_cam.*" --lz4 
