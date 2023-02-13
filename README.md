# inspection_robot

Author: [Davide Leo Parisi](https://github.com/dpareasy)

Contacts:
* personal email: davide.parisi1084@gmail.com
* institutional email: s4329668@studenti.unige.it


## Introduction ##
This repository houses a software that implements the behavior of a patrol robot using ROS (Robot Operating System). The code is written in Python and is an improvement over a previous iteration where the simulation was handled through two separate scripts that randomly generated the robot's path through various waypoints and made it moving through them. In this updated version, the simulation is carried out using Gazebo and the 3D visualization is provided by RViz. The robot is moved around the simulation environment by using [Move Base](https://github.com/CarmineD8/planning.git) and [SLAM gmapping](https://github.com/CarmineD8/SLAM_packages.git).
For a comprehensive understanding of the project's architecture, please refer to the [initial version](https://github.com/dpareasy/Assignment1) of the project.
Hereafter only the changes and inprovements with respect to the old version will be pointed out.

## Gazebo Simulation ##

### Environment ###

In the following figure the simulation environment in GAZEBO is presented. It follows the structure of the previous version.

![gazebo_environment](https://user-images.githubusercontent.com/92155300/218350489-ad910b79-541b-4446-802a-605465db365f.png)

### Robot model ###

The following figure represent ìs the model of the robot used in this simulation.

![robot_model](https://user-images.githubusercontent.com/92155300/218351884-d8267a10-3c7e-4cfe-8bc0-c082b8b9c101.png)


It presents a robotic arm with 3 rotational links and a camera mounted on the end-effector. The base link has two actuated wheels and one castor wheel. The motion is performed with the differential control drive.

### Survey behavior ###

In the following figure the robot in the initial position is presented. As it can be seen the map is not yet explored.


![init_pose](https://user-images.githubusercontent.com/92155300/218581607-a721d1b8-0d52-4684-8a7a-a69471a2df48.png)

Here you can see the map after having visited all the rooms and corridors.


![map_scanned](https://user-images.githubusercontent.com/92155300/218581617-4fa55d36-abd6-4471-9ccd-b13637821479.png)

The behavior is implemented in such a way that the robot moves through the rooms and corridors of the environment. When it enters a room, the base link rotates 360 in order to scan the location. After finishing the survey action it plans the path to reach the next location to visit. Whenever the battery goes down, the robot stops what it is doing and moves to the recharging location.

## Software architecure ##

### Sequence diagram ###

The following figure shows the sequence diagram of the project's architecture.
![sequence_diagram](https://user-images.githubusercontent.com/92155300/218548198-1710d59b-aad7-48a3-bdfc-e5525ed63c76.png)

### Component diagram ###
The following figure shows the component diagram of the architecture.
![UML_image](https://user-images.githubusercontent.com/92155300/218597788-c5d754d8-de62-43fb-b6ca-f6327d61575d.png)

## Project structure ##
You can refer to the previous version to get information about the whole structure. However, some changes have been performed:
1. `msg/`:
    * `MarkerList.msg`: message representing the topic in which marker's id are published.
    * `RoomConnection.msg`: message containing rooms connections.
    * `RoomCoord.msg`: message containing the room with its coordinates.
2. `srv/`:
    * `RoomInformation.srv`: receives marker id as a request and gives all rooms' informations as a response.
3. `action/`: 
    * `MoveArm.action`: receives a boolean input, gives the list of all individuals of the environment (rooms, corridors, doors) as a response and the rooms coordinates as a feedback.
    * `Survey.action`: receives a boolean input to start scanning the room, receives a feedback at the end of the action, no feedbacks are received during the execution in this solution.
4. `scripts/`: two more scripts have been added wrt the previous version and one have been deleted:
    * `arm_controller.py`: responsible of the arm motion and creation of the ontology.
    * `survey_controller.py`: responsible of the scanning action of each room.
5. `src/` folder has been added:
    * `marker_detector.cpp`: scripts for detecting the aruco markers.
    * `marker_server.cpp`: scripts containing all the information of rooms.
6. `worlds/`, `urdf/`,`param/` and `config/`: all the files containing files useful for the simulation.

## Software components ##

### The `state_machine` node ###

The modification made to this file has changed the way the ontology is created. It is now loaded by the `move_arm_as` action server instead of the `CreateOntology` class. The state machine is connected to three different action servers: 
* `move_arm_as`: responsible for moving the arm and loading the map.
* `move_base`: for making the robot move in the environment.
* `surveyor`: for allowing the robot to scan its surroundings.
In addition, a new publisher has been added compared to the previous version. It publishes a positive boolean value on the recharging_status topic to trigger the recharge process when the robot reaches the recharging zone.

### The `robot_state` node ###

This node is responsible for managing the battery. It subscribes to the recharging_status topic and initiates the recharging process when it receives a value of `True`. This approach was adopted to prevent the battery from being recharged while the robot is in motion towards the charging location.

### The `marker_detector` node ###

This node subscribes to the `camera1/image_raw` topic, where the camera publishes images. A processor then analyzes these images to detect the presence of an Aruco marker. If a marker is recognized, its identification number is then published on the `marker_detector/MarkerList` topic.

### The `marker_server` node ###

This node holds all the information related to each room, including its doors, connections, and position. It implements a ROS server that takes as input the marker identification number and returns all the relevant information about the room associated with that marker. The ROS server serves as a centralized repository for information about each room and allows for efficient retrieval of room-related information.

### The `move_arm_controller` node ###

This node implements an action server that controls the robotic arm on the vehicle. The arm is moved to different positions to scan for aruco markers in the environment. The control of the arm is achieved by publishing to the joint topics using ROS control. This node receives updates on the markers detected by the camera on the arm via the `marker_detector/MarkerList` topic. With the information on the detected markers, it then queries the `marker_server` to retrieve all the relevant information of the room associated with the marker's id, which is used to construct an ontology.

### The `survey_controller` node ###

This node also implements an action server that controls the scanning process of a reached room. The server publishes to the `/cmd_vel` topic with a specified angular velocity to rotate the base 360 degrees. This allows for a comprehensive scan of the room to gather data and information.


## Installation & Running ##

### Installation ###

Follow these steps to install the software:
1. Clone this repository inside your workspace (make sure it is sourced in your .bashrc).
2. Run `chmod +x <file_name>` for each file inside the scripts folder.
3. Install ros control:
```
sudo apt-get install ros-[ROS_version]-ros-control ros-[ROS_version]-ros-controllers
```
and 

```
sudo apt-get install ros-[ROS_version]-gazebo-ros-pkgs ros-[ROS_version]-gazebo-ros-control
```
4. clone move_base and slam_gmapping package in your workspace, by typing the following lines in your terminal:
```
sudo apt-get install ros-<ros_distro>-openslam-gmapping 
```
and 

```
sudo apt-get install ros-<ros_distro>-navigation
```
5. Clone [Vision opencv](https://github.com/ros-perception/vision_opencv.git) repository in your workspace.
6. Clone [Aruco Ros](https://github.com/CarmineD8/aruco_ros.git) repository in your workspace, and copy the folder `model` in /root/.gazebo/models.
7. Clone [SLAM gmapping](https://github.com/CarmineD8/SLAM_packages.git) repository in your workspace.
8. Clone [Move Base](https://github.com/CarmineD8/planning.git) repository in your workspace.
9. Follow [Installation & Running](https://github.com/dpareasy/Assignment1) procedure in the readme of the previous version to install all the remaining packages.
10. Run `catkin_make` from the root of your workspace.

To run the simulation, copy the following two lines in the terminal:
```
roslaunch inspection_robot armor.launch
```
and 
```
roslaunch inspection_robot surveillance.launch
```

They are launched separately since some problem have arisen by launching them together.


## System limitations & possible improvements ##

As regarding the ontology and policy, the same [limitations](https://github.com/dpareasy/Assignment1) presented in the previous version are also valid for this project.
For what concerns the simulation instead, the following limitations are present:
* The arm is not moved in a general way around the environment. Thus, the solution implemented is only valid for this specific problem. This choice was driven by the fact that the aruco markers were not perfectly scanned by the camera. To allow a perfect a perfect recognition of the markers, the fov have been reduced so as to frame one marker at a time.
* The arm is controlled with ros control by publishing the desired position of each joint on the corresponding topic, which results in a larer number of instructions needed for movement.
* Base link velocity is very low, causing the robot to take about thirty minutes to visit all rooms.
* The robot may sometimes stop moving and will require the simulation to be restarted. This is likely a bug with move_base. Also, specially from the initial position, the robot gets stuck in a corner of the room.

Considering these limitations, it's evident what can be improved. A general strategy for arm movement would address the first limitation. It is crucial to find the appropriate camera settings to allow an easy recognition of all markers.

Using the [Moveit](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html) tool to control the arm would eliminate the need for publishing on different topics and allow for arm movement using inverse kinematics. 

All the issues with the robot's motion can likely be attributed to the move_base implementation. To prevent the robot from getting stuck in a corner of the room from its starting position, a simple solution was implemented, which involves rotating the base by 180 degrees after all aruco markers have been recognized, using the same logic implemented for the survey action.


## Documentation ##
You can find [here](https://dpareasy.github.io/inspection_robot/) the whole documentation.
Two scripts are missing in the documentation `marker_server.cpp` and `marker_detector.cpp`. The first one was already given while the second one has been reused from [another project](https://github.com/CarmineD8/aruco_ros.git) named `marker_publish.cpp`.
