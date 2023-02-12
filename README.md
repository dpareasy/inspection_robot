# inspection_robot

Author: [Davide Leo Parisi](https://github.com/dpareasy)

Contacts:
* personal email: davide.parisi1084@gmail.com
* institutional email: s4329668@studenti.unige.it


## Introduction ##
This repository houses a software that implements the behavior of a surveillance robot using ROS (Robot Operating System). The code is written in Python and is an improvement over a previous iteration where the simulation was handled through two separate scripts that randomly generated the robot's path through various waypoints. In this updated version, the simulation is carried out using Gazebo and 3D visualization is provided by RViz.
For a comprehensive understanding of the project's architecture, please refer to the [initial version](https://github.com/dpareasy/Assignment1) of the project.

Hereafter only the changes with respect to the old version will be pointed out.

## Project structure ##
You can refer to the previous version to get information about the whole structure. However, some changes have been performed:
1. `msg/`:
    * `MarkerList.msg`: message representing the topic in which marker's id are published.
    * `ReachStatus.msg`: message stating that the charging can start.
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

This file has been modified in such a way that the ontology is no more created using `CreateOntology` class, but the action server `move_arm_as` is now responsible of loading the map. The state machine connects with three different action servers:
* `move_arm_as`: as mentioned before.
* `move_base`: to make the robot moving around the environment.
* `surveyor`: to make the robot scanning the location visited.

### The `move_arm_controller` node ###

This node implements an action server that controls the robotic arm on the vehicle. The arm is moved to different positions to scan for aruco markers in the environment. The control of the arm is achieved by publishing to the joint topics using ROS control. This node receives updates on the markers detected by the camera on the arm via the `marker_detector/MarkerList` topic. With the information on the detected markers, it then queries the `marker_server` to retrieve a list of rooms in the environment and their relevant information, which is used to construct an ontology.

### The `survey_controller` node ###

This node also implements an action server that controls the scanning process of a reached room. The server publishes to the `/cmd_vel` topic with a specified angular velocity to rotate the base 360 degrees. This allows for a comprehensive scan of the room to gather data and information.

## Installation & Running ##

### Installation ###

Follow these steps to install the software:
1. Clone this repository inside your workspace (make sure it is sourced in your .bashrc).
2. Follow the steps for [aRMOR](https://github.com/EmaroLab/armor/issues/7) and [Smach-ROS](http://wiki.ros.org/smach/Tutorials/Getting%20Started) installation.
3. Use [armor_api](https://github.com/EmaroLab/armor_py_api) for server requests, clone the repository in your workspace.
4. Clone inside your workspace the [topological_map](https://github.com/buoncubi/topological_map) repository containing the ontology for this project.
5. Run `chmod +x <file_name>` for each file inside the scripts folder.
6. Run `catkin_make` from the root of your workspace.
7. Install `simple_colors` by copying the following line on your terminal:

```
pip install simple-colors
```

8. Install ros control:
    ```
    sudo apt-get install ros-[ROS_version]-ros-control ros-[ROS_version]-ros-controllers
    ```
     and 

     ```
     sudo apt-get install ros-[ROS_version]-gazebo-ros-pkgs ros-[ROS_version]-gazebo-ros-control
     ```
Some issues have been encountered for creating the ontology. Refer to the 'Installation' section of the [previous version](https://github.com/dpareasy/Assignment1) and follow the procedure to fixe the problem.

### System limitations ###


## Possible improvements ###