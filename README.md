# rosGPSDriver
This repository contains versions of GPS waypoint navigation and PIV controller python scripts for the VRX simulator. 

gpsMove.py tries to get the angle between the direciton it's facing and the target to be zero, while also closing the distance

matrixWaypointNav.py puts the target into the Robot's frame of reference then uses a rotation matrix and some facny math
                  to decide how much power to send to the riht and left thruster
