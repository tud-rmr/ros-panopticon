Scripts
=======

Marker generation
-----------------

To add additional markers, generate them with::

	./create_board.sh <MARKER_ID>

from the **scripts** folder. Put the boardconfig into the **config/boards** folder and add a line to **config/boards.yml**. One can adjust which marker is ``map``, ``middle`` and plain marker in ``config/boards.yml``. Change the script for different marker sizes. In order for this script to work, the command line tools of ArUco have to be in path.