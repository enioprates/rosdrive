# ROSDRIVE README
* Version: 	1.1
* Date:		19/01/2022
* Author: 	Enio Filho

## Requirements
* **OS**: Ubuntu 18.04
* ROS: Melodic
* Gazebo: Gazebo 9

## How to setup the project 

### RosSim Simulator Environment
````
- Download files from github (git clone https://github.com/enioprates/rosdrive)
- Open a new terminal inside ~/CISTER_car_simulator
- catkin_make
- run the simulator: 
	- source devel/setup.launch
	- roslaunch car_demo demo_t.launch
````
````
[OPTIONAL: Changing the Track, vehicles initial position or vehicles number]
car_demo Options:
- Circuit Options:
	- LineTrack_curve_03.world - Circuit with no obstacles
	- LineTrack_curve_overtake4.world - Circuit with obstacles
	- LineTrack_curve_overtake5.world - Circuit with more obstacles

- Launch Vehicles Position
	- launch/cars_t_curve_2.launch

````

* **ATTENTION**: Before running the controller algorithms, PAUSE the simulation and reset the time!!!!


### RosSim Controller algorithms
- the controller algorithms are inside ~/CISTER_image_processing
- Delete build and devel folders from ~/CISTER_image_processing folder
- Open a new terminal inside ~/CISTER_image_processing
- Compile
	- catkin_make		
	Error and solutions:
	- Install SDL2:	sudo apt-get install libsdl2-dev

* **INFO**: The controller algorithm is based in several files, as long as the leader of the vehicles follows the line in the road

## Python 
````
- executable files: 	chmod +x <NAME_OF_THE_FILE>.py
- libraries:		sudo apt-get install python-numpy python-scipy python-matplotlib ipython ipython-notebook python-pandas python-sympy python-nose
````

## Starting the Vehicle
````
(LINE DETECTION ALGORITHM)
- Open a new terminal inside ~/CISTER_car_drive_controller
- source devel/setup.launch
- rosrun image_processing lane_lines_detection.py
````
````
(LINE FOLLOWER ALGORITHM)
- Open a new terminal inside ~/CISTER_car_drive_controller
- source devel/setup.launch
- rosrun image_processing simulation_connector.py
````

````
## Recording Data
````
- Open a new terminal inside ~/CISTER_image_processing
- source devel/setup.launch
- rosrun image_processing listener.py car1 TV     //records the vehicle

````

## Adjustable parameters inside simulation_connector.py:
````
- Heading PID
- Speed PID 
- Desired Speed
- Activate Sonars
````

## Road and Vehicles positions:
````
- demo_t.launch
	- OVAL ROAD: <arg name="world_name" value="$(find car_demo)/worlds/LineTrack_oval2.world"/>
	- Vehicles Initial Position: <include file="$(find car_demo)/launch/cars_t_oval.launch"/>
	
	- CURVE ROAD: <arg name="world_name" value="$(find car_demo)/worlds/LineTrack_curve_03.world"/>
	- Vehicles Initial Position: <include file="$(find car_demo)/launch/cars_t_curve.launch"/>
	
	- CURVE WITH OBSTACLES ROAD: <arg name="world_name" value="$(find car_demo)/worlds/LineTrack_curve_extreme2.world"/>
	- Vehicles Initial Position: <include file="$(find car_demo)/launch/cars_t_curve.launch"/>
	
	- Other Circuit Options:
		- LineTrack_curve_03.world - Circuit with no obstacles
		- LineTrack_curve_overtake4.world - Circuit with obstacles
		- LineTrack_curve_overtake5.world - Circuit with more obstacles

	- Other Vehicles Position
		- launch/cars_t_curve_2.launch

- When one is chosed, the others should be commented
````

## Launch Files:
````
In order to run several files together, we create some launch files:
- Vehicles
	- Open a new terminal inside ~/CISTER_image_processing
	- source devel/setup.launch
	- roslaunch image_processing vehicles.launch
- Recording data
	- Open a new terminal inside ~/CISTER_image_processing
	- source devel/setup.launch
	- roslaunch image_processing listener.launch
````

## Changing the number of VEHICLES
````
To change the number of vehicles in GAZEBO:
- In cars_t_curve_2.launch
	- Add new vehicles
	- change initial position and heading
````
