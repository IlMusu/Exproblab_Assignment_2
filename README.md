
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
<center>
	<img src="https://i.imgur.com/yVivSbS.png" width="80%">
</center>

### Robot
The robot used in the simulation is a differential robot equipped with a simple robotic arm with only two joints, a camera placed at the end-effector and a laser scanner. The following image shows the rendering of robot.
<center>
	<img src="https://i.imgur.com/dq6hsQy.jpg" width="80%">
</center>
The robot description is located in <b>[this](https://github.com/IlMusu/Exproblab_Assignment_2/tree/master/final_assignment/urdf)</b> folder.

### Markers
In the following images are shown the types and placement of the markers that have been used in the simulation: as can be seen, all the markers are in the same location. The robot will be spawned at the center of that location so that it can identify all the markers by just controlling the joints of the robotic arm.
<center>
	<img src="https://i.imgur.com/aR28QNP.jpg" width="80%">
</center>
Another image of the same location from a different perspective:
<center>
	<img src="https://i.imgur.com/asekuqh.jpg" width="80%">
</center>

## 2. INSTALLATION AND RUNNING
### Installation
The software contained in this repository is highly dependant on the architecture developed in the first assignment which can be found in this <b>[github repository](https://ilmusu.github.io/Exproblab_Assignment_2/)</b>. After correctly following the <b>INSTALLATION AND RUNNING</b> section of the previous assignment it is possible to follow this installation.  
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
<img src=https://github.com/IlMusu/Exproblab_Assignment_2/blob/documentation/images/components_diagram.svg?raw=true">
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
- The `robot_behavior` node contains the state machine that describes the desired behavior of the robot. The functionality of this node is based on the interaction with other nodes. In particular, it interacts with:  
  - The `ontology_map_builder` node through the <b>/ontology_map/reference_name</b> service.  
  - The `ontology_map_builder` node through the <b>/ontology_map/room_position</b> service.  
  - The `armor_service` library through the <b>/armor_interface_srv</b> service.  
  - The `battery_controller` node through the <b>/battery_level</b> message.  
  - The `planner_controller` node through the <b>/compute_path</b> action.  
  - The `motion_controller` node through the <b>/follow_path</b> action.  
- The `battery_controller` node controls the level of the battery. It interacts with:  
  - The `robot_behaviour` node through the <b>/battery_level</b> message.  
- The `planner_controller` node constructs a path between two points. It interacts with:  
  - The `robot_behaviour` node through the <b>/compute_path</b> message.  
- The `motion_controller` node controls the movement of the robot. It interacts with:  
  - The `robot_behaviour` node through the <b>/follow_path</b> message.  

A more detailed explanation of the use of the interfaces is available <b>[here](#ros-messages-services-and-actions)</b>.  
