#! /bin/bash
# In order to get a yml file for ar_sys, two steps are needed: 
# 1) Generate a board with the 'aruco_create_board' tool. It generates a configuration yml in PIXEL
# 2) Generate a configuration file in meters, as it is required by ar_sys
#
# An overview about the aruco tools can be found under https://sourceforge.net/projects/aruco/files/

MARKER_ID=${1}
PIXEL_SIZE=500  # in pixel
MARKER_SIZE=0.21 # in meter

MARKER_FILE=marker_${MARKER_ID}.png
CONFIG_FILE_PX=boardconfig_px_${MARKER_ID}.yml
CONFIG_FILE_M=boardconfig_${MARKER_ID}.yml

# "Usage: X:Y boardImage.png boardConfiguration.yml [pixSize] [Type(0: panel,1: chessboard, 2: frame)] [interMarkerDistance(0,1)]"
aruco_create_board 1:1 ${MARKER_FILE} ${CONFIG_FILE_PX} ${PIXEL_SIZE} 0 0.1

# aruco_create_board creates a board with random markers, but I want specific ones. Therefore, we alter the id in the
# generated file and create a new marker with the right ID. tl;dr we just need the config file, rest is created later. 

sed -i -r "s/id:([0-9])+/id:${MARKER_ID}/g" ${CONFIG_FILE_PX}

# "Usage:  in_boardConfiguration.yml markerSize_meters out_boardConfiguration.yml"
aruco_board_pix2meters ${CONFIG_FILE_PX} ${MARKER_SIZE} ${CONFIG_FILE_M}

# "Usage: <markerid(0:1023)> outfile.jpg sizeInPixels"
aruco_create_marker ${MARKER_ID} ${MARKER_FILE} ${PIXEL_SIZE}


