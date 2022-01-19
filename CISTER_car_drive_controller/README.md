# CISTER_image_processing
* Version: 	1.0
* Date:		08/04/2020
* Author: 	Enio Filho

## Requirements
* **OS**: Ubuntu 18.04
* ROS: Melodic
* Gazebo: Gazebo 9

## ROSDRIVE Controller algorithms
- the controller algorithms are inside ~/CISTER_image_processing
- Delete ".cache" files from ~/CISTER_image_processing/Build folder
- Open a new terminal inside ~/CISTER_image_processing
- Compile
	- catkin_make		
	Error and solutions:
	- Install SDL2:	sudo apt-get install libsdl2-dev

* **INFO**: The control algorithm is based in several files, as long as the leader of the platoon follows the line in the road

## Python
- Be sure to make all files as executables 
````
- executable files: 	chmod +x <NAME_OF_THE_FILE>.py
- libraries:		sudo apt-get install python-numpy python-scipy python-matplotlib ipython ipython-notebook python-pandas python-sympy python-nose
````

## Starting the vehicle
````
(LINE DETECTION ALGORITHM)
- Open a new terminal inside ~/CISTER_image_processing
- source devel/setup.launch
- rosrun image_processing lane_lines_detection.py
````
````
(LINE FOLLOWER ALGORITHM)
- Open a new terminal inside ~/CISTER_image_processing
- source devel/setup.launch
- rosrun image_processing simulation_connector.py
````


## Recording Data
````
- Open a new terminal inside ~/CISTER_image_processing
- source devel/setup.launch
- rosrun image_processing listener.py car1 TV		//records the leader and fisrt follower data

````

## Adjustable paramters inside platooning.py:
````
- Steering PID
- Distance PID 
- Minimum distance between vehicles
- Bearing control (ON OFF)
````





