
# EXPROBLAB - ASSIGNMENT 2
Author : Mattia Musumeci 4670261@studenti.unige.it  
This is the second assignment developed for the <b>Experimental Robotics Laboratory</b> course of the University of Genoa.  
At this <b>[link](https://ilmusu.github.io/Exproblab_Assignment_1/)</b> it is possible to find the documentation for the software contained in this repository.

## 1. INTRODUCTION
The scenario involves a <b>robot</b> deployed in an indoor environment for <b>surveillance purposes</b> whose objective is to visit the different environment locations and explore them for a given amount of time. In this context, the robot in equipped with a rechargeable battery which needs to be recharged in a specific location when required.  
The robot is equipped with a <b>robot arm</b> connected to a <b>camera</b> at the end-effector. That is because, initially, the robot has no information about the environment which it is in and it has to use the camera to identify <b>markers</b> that are placed in its proximity and that give information about the planimetry of the environment.
  
The software contained in this repository has been developed for <b>[ROS Noetic 1.15.9](http://wiki.ros.org/noetic)</b>.  
The <b>Robot Operating System (ROS)</b> is an open-source, meta-operating system for your robot. It provides the services you would expect from an operating system, including hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management. It also provides tools and libraries for obtaining, building, writing, and running code across multiple computers.  The full installation of ROS contains also Gazebo which is the most common 3-dimensional physical simulator used in robotics and Rviz which is a visualization widget for the simulation: together, these two tools allow the user to simulate a specific robot in a specific environment but also to view the simulated robot and environment, analyse log and replay sensor information.
  
The behavior of the robot has been described with a finite state machine developed for <b>[SMACH](http://wiki.ros.org/smach)</b>.  
The <b>SMACH</b> library is a task level architecture for describing and executing complex behaviors. At its core, it is a ROS-independent python library to build hierarchical state machines and the interaction with the ROS environment is obtained by implementing dedicated states inside the finite state machine.  
  
The ontology concepts and reasoning have been implemented with <b>[ARMOR](https://github.com/EmaroLab/armor)</b>.  
A <b>ROS Multi-Ontology Reference (ARMOR)</b> is a powerful and versatile management system for single and multi-ontology architectures under ROS. It allows to load, query and modify multiple ontologies and requires very little knowledge of OWL APIs and Java.  

The markers used in this project are the <b>[ArUco Markers](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)</b>.
The <b>ArUco Markers</B> which are synthetic square markers that are made up of a wide black border and an inner binary matrix that identifies the marker. The black border helps the marker to be quickly detected in an image, and the binary coding allows for error detection and correction. The size of the marker determines the size of the internal matrix. For example, a marker with a size of 4x4 has an internal matrix composed of 16 bits.  

For detecting the markers, the <b>[OpenCV](https://opencv.org/)</b> library has been used.
The <b>Open Source Computer Vision (OpenCV)</b> library is a free and open-source computer vision and machine learning software library. It is aimed at real-time computer vision and has a comprehensive set of algorithms for tasks such as image processing, object detection, and recognition, video analysis, and machine learning.

### Environment
In the following it shown an image of the environment the robot is placed in.
<p align="center">
	<img src="https://i.imgur.com/yVivSbS.png" width="80%">
</p>

### Robot
The robot used in the simulation is a differential robot equipped with a simple robotic arm with only three joints, a camera placed at the end-effector and a laser scanner. The following image shows the rendering of robot.
<p align="center">
	<img src="https://i.imgur.com/dq6hsQy.jpg" width="80%">
</p>

The robot description is located in <b>[this](https://github.com/IlMusu/Exproblab_Assignment_2/tree/master/final_assignment/urdf)</b> folder.

### Markers
In the following images are shown the types and placement of the markers that have been used in the simulation: as can be seen, all the markers are in the same location. The robot will be spawned at the center of that location so that it can identify all the markers by just controlling the joints of the robotic arm.  

<p align="center">
	<img src="https://i.imgur.com/aR28QNP.jpg" width="80%">
</p>

Another image of the same location from a different perspective:  

<p align="center">
	<img src="https://i.imgur.com/asekuqh.jpg" width="80%">
</p>

## 2. INSTALLATION AND RUNNING
### Installation
The software contained in this repository is highly dependant on the architecture developed in the first assignment which can be found in this <b>[github repository](https://github.com/IlMusu/Exproblab_Assignment_1)</b>. After correctly following the <b>INSTALLATION AND RUNNING</b> section of the previous assignment it is possible to follow this installation.  
The software contained in this repository is a ROS package.  
Therefore, in order to install the software, it is necessary to create a workspace.  
Notice that it is also possible to use an already existing workspace.
```bash
mkdir -p [workspace_name]/src
```
Then, clone this repository inside the src folder just created:
```bash
cd [workspace_name]/src/
git clone [this_repo_link] .
```

Then, rebuild the workspace by returning to the workspace folder:
```bash
cd ..
catkin_make
```

The setup.bash file must be sourced so that ROS can find the workspace.  
To do this, the following line must be added at the end of the .bashrc file:
```bash
source [workspace_folder]/devel/setup.bash
export PYTHONPATH=$PYTHONPATH:[workspace_folder]/src
```
### Running
In order to run the scripts, it is necessary to first run the ROS master.  
Open a new console and run the following command:
```bash
roscore
```
Some launch files have been prepared in order to simplify the execution.  
Into different terminals, run the following commands:
```bash
roslaunch final_assignment simulation_enviornment.launch
roslaunch final_assignment armor_builder.launch
roslaunch final_assignment robot_surveillance.launch
```
## 2. SOFTWARE ARCHITECTURE
### Component Diagram
In the  <b>component diagram</b>  are shown all the  <b>blocks</b>  and  <b>interfaces</b>  that have been used or developed in order to obtain the desired software architecture.

<p align="center">
<img src="https://github.com/IlMusu/Exproblab_Assignment_2/blob/documentation/images/components_diagram.svg?raw=true">
</p>

- The `marker_server` nodes provides the necessary information regarding a room through the related ArUco marker id. It interacts with:
	- The `marker_detector` node through the <b>/room_info</b> service.
- The `marker_detector` node performs the preliminary inspection routine to obtain all the necessary ArUco markers id. Then, the ids are used to obtain the information about the topology of the environment. It interacts with:
	- The `marker_server` node through the <b>/room_info</b> service.
	- The `robot_inspection_routine` node through the <b>/robot_inspection_routine</b> action.
	- The `ontology_map_builder` node through the <b>/ontology_map/build_map</b> action.
- The `robot_inspection_routine` node makes the arm of the robot rotate in circles at diffent pitches so that all the ArUco markers around the robot are scanned correctly. It interacts with:
	- The `marker_detector` node through the <b>/robot_inspection_routine</b> action.
- The `ontology_map_builder` node loads the default ontology into ARMOR and builds the map following the user requests. It also contains a mapping between each room and its position with respect to the world frame. Notice that in this context, the "position of a room" is defined as a point inside the room that the robot is able to reach.  It interacts with:
   - The `armor_service` library through the <b>/armor_interface_srv</b> service.  
   - The `marker_detector` node through the <b>/ontology_map/build_map</b> action.
   - The `robot_behavior` node through the <b>/ontology_map/reference_name</b> service.  
   - The `robot_behavior` node through the <b>/ontology_map/room_position</b> service.  
- The `motion_controller` node controls the movement of the robot. It interacts with:  
  - The `robot_behaviour` node through the <b>/follow_path</b> message. 
  - The `move_base` node through the <b>/move_base</b> action. 
- The `move_base` node makes the robot move to a goal pose. It interacts with:
	- The `motion_controller` node through the <b>/move_base</b> action. 

The remaining nodes of the architecture are explained in the <b>[README](https://github.com/IlMusu/Exproblab_Assignment_1/blob/master/README.md)</b> of the previous assignment.
A more detailed explanation of the use of the interfaces is available <b>[here](#ros-messages-services-and-actions)</b>.  

### Sequence Diagram
This <b>sequence diagram</b> shows a possible execution of the software contained in this repository. More in details, this diagram shows the execution in time of all the nodes and the requests/responses between them.  
Notice that this diagram only shows the beginning of the execution, ence, the detection of the ArUco markers and the building of the ontology. That is because the remaining part of the diagram is the same shown in the <b>[README](https://github.com/IlMusu/Exproblab_Assignment_1)</b> of the previous assignment.

<p align="center">
<img src=https://github.com/IlMusu/Exproblab_Assignment_2/blob/documentation/images/sequence_diagram.svg?raw=true">
</p>

This first horizonal line shows that there can be multiple iterations of performing the inspection routine and then communicating the markers id to the marker server. That is because at the ispection routine the robot might not have detected all the markers or some of them might be wrong.  
The second horizonal line shows the end of this sequence diagram and the begin of the sequence diagram shown in the repository of the previous assignment.

### ROS Messages Services And Actions
In order to develop the interfaces between the components:  
- The <b>ontology_map_builder</b> node which:  
  - Provides the <b>`/ontology_map/reference_name`</b> service, of type `ReferenceName.srv`, to provide the reference name of the ontology that is loaded into ARMOR. This is done only once the ontology is fully created and loaded.  
  - Provides the <b>`/ontology_map/room_position`</b> service, of type `RoomPosition.srv`, to provide a position inside the requested room. The position is measured with respect to the world frame.  
  - Subscribes to the <b>`/ontology_map/build_map`</b> topic, of type `OntologyMap.msg`, to received the completed odometry of the environment through the message format.  
- The <b>motion_controller</b> node which:  
	- Creates a <b>`/follow_path`</b> action server, of type `FollowPath.action`, to make the follow a path composed of a ordered list of waypoints.  
	- Subscribes to the <b>`/odom`</b> topic, of type `Odometry`, to retrieve the current position of the rotot.
	- Creates a client for the <b>`/move_base`</b> action server, of type `MoveBase.action` to make the robot move between two waypoints of the path.  
- The <b>planner_node</b> node which:  
	- Creates a client for the <b>`/compute_path`</b> action server, of type `ComputePathAction.action`, for computing a path of waypoints from a start to a goal position.
- The <b>robot_inspection_routine </b> node which:  
	- Subscribes to the <b>`/joint0_position_controller/command`</b> topic, of type `Float64`, for controlling the first joint of the robot arm.
	- Subscribes to the <b>`/joint1_position_controller/command`</b> topic, of type `Float64`, for controlling the second joint of the robot arm.
	- Subscribes to the <b>`/camera_position_controller/command`</b> topic, of type `Float64`, for controlling the camera joint of the robot arm.
	- Creates a <b>`/robot_inspection_routine`</b> action server, of type `RobotInspectionRoutine.action`, to make the robot move the arm is a spherical pattern.
- The <b>marker_detector</b> node which:  
	- Subscribes to the <b>`/camera/image_raw`</b> topic, of type `Image.msg`, for receiving the camera images and retrieving any eventual ArUco marker inside the image.
	- Publishes to the <b>`/cmd_vel`</b> topic, of type` Twist.msg`, for updating the velocity of the robot. This is used for making the robot stand still while it is obtaining for the markers.
	- Publishes to the <b>`/ontology_map/build_map`</b> topic, of type `OntologyMap.msg`, for publishing all the necessary information regarding the ontology that needs to be loaded on ARMOR.
	- Uses the <b> `/room_info`</b> service, of type `RoomInformation.srv`,  for requesting the information about a room which is related to an ArUco marker id.

### ROS Parameters
The `ontology_map_builder` node uses the following parameters:  
- /ontology_reference (string) : The reference name of the ontology.  
- /ontology_path (string) : The global path of the default ontology.  
- /ontology_uri (string) : The uri of the ontology.  
- /rooms (list) : The list of rooms names for building the map.  
- /rooms_doors (list) : At index i, the list of doors belonging to room i.  
- /rooms_positions (list) : At index i, the position of room i.  
- /robot_room (string) : The initial room at which the robot is located.  

The `motion_controller` node uses the following parameters:  
- /goal_threshold (float) : The distance from goal at which the robot is considered to be arrived at goal.

## 4. RUNNING CODE
### Detecting The Markers
<p align="center">
<img src="https://github.com/IlMusu/Exproblab_Assignment_2/blob/documentation/gifs/markers_detection.gif?raw=true">
</p>

In this first gif it is possible to observe how the robot is able to detect the ArUco markers around it self: the arm is composed by a rotational joint connected the main chassis which controls the yaw of the arm, and another rotational joint which controls the pitch of the camera.  
The robot performs yaw rotations of the arm from -π to π at different camera pitches.  
It is supposed that the number of placed ArUco markers is known a priori: the robot continues to perform these rotations, which are referred to as <b>"robot inspection routines"</b> until, all the markers are located correctly. 

In fact, it may happen that a marker is not detected correctly and a wrong id is obtained: the robot simply discards the value and continues to scan the environment. This behavior can be observed at the end of the gif when the markers 147 and 148 are discarded.

### Moving In The Environment
<p align="center">
<img src="https://github.com/IlMusu/Exproblab_Assignment_2/blob/documentation/gifs/moving_in_the_environment.gif?raw=true">
</p>

In this second gif instead, it is possible to observe that, after all the markers have been correctly detected, the robot starts to move in the environment following the behaviour described in the previous assignment.

## 5. FUTURE WORK
These are some of the possible improvements that can be carried on this project:
   - Currently the robotic arm placed on the robot chassis moves almost instantly from one configuration to another, this causes some problems with the physics simulation. This problems might be solveed by better tuning the parametes, using a different PID controller, or using another type controller for the arm. 
   - It might happen that the move_base node does not always move the robot is the best possible way and the robot might get stuck on some walls. This problems might be solved by better tuning the move_base parameters or use another node for moving the robot.
