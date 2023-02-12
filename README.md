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
    * MarkerList.msg: message representing the topic in which marker's id are published.
    * ReachStatus.msg: message stating that the charging can start.
    * RoomConnection.msg: message containing rooms connections.
    * RoomCoord.msg: message containing the room with its coordinates.
2. `srv/`:
    * RoomInformation.srv: receives marker id as a request and gives all rooms' informations as a response.
3. `action/`: 
    * MoveArm.action: receives a boolean input, gives the list of all individuals of the environment (rooms, corridors, doors) as a response and the rooms coordinates as a feedback.
    * Survey.action: receives a boolean input to start scanning the room, receives a feedback at the end of the action, no feedbacks are received during the execution in this solution.
4. `scripts/`: two more scripts have been added wrt the previous version and one have been deleted:
    * `arm_controller.py`: responsible of the arm motion and creation of the ontology.
    * `survey_controller.py`: responsible of the scanning action of each room.
5. `src/` folder has been added:
    * `marker_detector.cpp`: scripts for detecting the aruco markers.
    * `marker_server.cpp`: scripts containing all the information of rooms.
6. `worlds/`, `urdf/`,`param/` and `config/`: all the files containing files useful for the simulation.

## Software components ##

### The `state_machine` Node ###

This file has been modified in such a way that the ontology is no more created using `CreateOntology` class, but the action server `ArmControllerServer` is now responsible of loading the map.
