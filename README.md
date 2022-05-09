# rosGPSDriver
This repository contains versions of GPS waypoint navigation and PIV controller python scripts for the VRX simulator. 

gpsMove.py tries to get the angle between the direciton it's facing and the target to be zero, while also closing the distance

matrixWaypointNav.py puts the target into the Robot's frame of reference then uses a rotation matrix and some facny math
                  to decide how much power to send to the riht and left thruster

sinMove.py is not really a navigator at all, but is simple and could be used to help understand how the python scripts              talk to the boat


<h2> <b>Some notes and usefull commands: </b> </h2>
$:  source ~/vrx_ws/devel/setup.bash
***Note that vrx_ws is he folder that contains our simulation workspace

To run the simlation with the default bot, run:
$: roslaunch vrx_gazebo vrx.launch camera_enabled:=true gps_enabled:=true imu_enabled:=true

***GPS uses sensor_msgs/NavSatFix msgs imu (inertial measurement unit) uses sensor_msgs/Imu
NavSat msgs Docs: https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html
IMU:  http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/Imu.html


I create packages using
$: catkin create pkg <package name> --catkin-deps <dependency1> <dependency2> <dependency3> >etc>
Example:
$: catkin create pkg boat_movement --catkin-deps rospy roscpp std_msgs xacro move_base_msgs geometry_msgs

I have my movement nodes inside boat_movement. In order to add new nodes (in python) 
	I put them in a folder called 'python_scripts' inside th e packages src folder
Then make sure to update the appropriate spot in the CMake_Lists.txt file for the package
	There is a part that alaready has some python files so you can just clook fo rthe .py part
	Should be somehting like 'python_scripts/gps_move.py' somwhere

to create new files, use touch
Example pf creating a new folder and file in a chained command:
$: mkdir python_scripts && cd python_scripts && touch sinMove.py


To open the standard text editor (not nano or one of the terminal ones), aka the gui one, use
$: xdg-open <filename> 
Example:
$: xdg-open sinMove.py 


To run python scripts, make sur eyou save the progress, but you don't need to run Catkin Build every time you make changes
for example:
$: rosrun boat_movement gpsMove.py
