#!/usr/bin/env python3
"""
.. module:: my_state_machine
   :platform: Unix
   :synopsis: Python module for Moving the arm and create the ontology

.. moduleauthor:: Davide Leo Parisi <davide.parisi1084@gmail.com>

ROS node for moving a robotic arm and creating the ontology

Service:
    /move_arm_as: server for moving the arm

Publishes:
    /insprob/joint1_position_controller/command
    /insprob/joint2_position_controller/command
    /insprob/joint3_position_controller/command

Subscribes:
    /marker_detector/MarkerList
"""

import rospy
import random
import time
from numpy import *
from std_msgs.msg import Float64
import actionlib
from inspection_robot.msg import MoveArmAction, MoveArmFeedback, MoveArmResult, MarkerList, RoomConnection, RoomCoord
from armor_api.armor_client import ArmorClient
from inspection_robot.srv import *
from os.path import dirname, realpath

link1 = Float64()
link2 = Float64()
link3 = Float64()

pub1 = rospy.Publisher('/insprob/joint1_position_controller/command', Float64, queue_size = 10)
pub2 = rospy.Publisher('/insprob/joint2_position_controller/command', Float64, queue_size = 10)
pub3 = rospy.Publisher('/insprob/joint3_position_controller/command', Float64, queue_size = 10)

global marker_id

class ArmControllerServer():
    """
    This class is used to move the arm, receive the marker_id detected from the camera nd to create the ontolgy.
    """
    def __init__(self):
        # Define the  action server
        self.a_server = actionlib.SimpleActionServer("move_arm_as", MoveArmAction, execute_cb = self.execute_cb, auto_start = False)
        
                                    ######################
                                    ## Set up for Armor ##
        ###########################################################################
        self.armor_client = ArmorClient("assignment", "my_ontology")
        self.path = dirname(realpath(__file__))
        # Put the path of the file.owl
        self.path = self.path + "/../../topological_map/"
        # Initializing with buffered manipulation and reasoning
        self.armor_client.utils.load_ref_from_file(self.path + "topological_map.owl", "http://bnc/exp-rob-lab/2022-23", True, "PELLET", False, False)
        self.armor_client.utils.mount_on_ref()
        self.armor_client.utils.set_log_to_terminal(True)
        # Define the subscriber to the topic in which the marker's id are published
        rospy.Subscriber("marker_detector/MarkerList", MarkerList, self.marker_cb)
        # Client connecting to marker_server to obtain environment's informations
        self.send_id = rospy.ServiceProxy('/room_info', RoomInformation)
        #Start the server
        self.a_server.start()

        # Definition of used variables
        self.marker_list = []
        self.location_list = []
        self.room_coord = []
        self.door_list = []
        self.rooms_info = []
        self.individuals_list = []
        self.room_coord = RoomCoord()
        # Feedback from the arm
        self.feedback = MoveArmFeedback()
        self.result = MoveArmResult()
        self.marker_id = None

                                #####################
                                ## Create Ontology ##
    ###############################################################################
    def marker_cb(self, msg):
        """
            This callback takes the informations aboud the id of the aruco markers. Then it makes the request to the marker server to get informations about the rooms.
            Then it creates the ontology.
        """

        self.marker_id = msg.markers
        print(self.marker_id)
        
        # Making request to server
        rospy.wait_for_service('/room_info')
        try:
            response = self.send_id(self.marker_id)
            room = response.room
            print(room)

            # reset the feedback list
            self.feedback.locations = []

            if room != 'no room associated with this marker id' and room not in self.individuals_list:
                self.marker_list.append(self.marker_id)
                print(self.marker_list)
                # saving room cooridnates in a variable of type RoomCoord
                self.room_coord.room = room
                self.room_coord.x = response.x
                self.room_coord.y = response.y
                # Send room's information as a feedback to the state machine
                self.feedback.locations.append(self.room_coord)
                self.a_server.publish_feedback(self.feedback)
                # update locatin list for the ontology
                self.location_list.append(room)
                # append all rooms in the list of individuals
                self.individuals_list.append(room)
                # add individuals to class to create the ontolgy
                # declare the new creted individuals as visted to make the timer start counting
                if room != 'E':
                    self.armor_client.manipulation.add_ind_to_class(room, "LOCATION")
                    print(" Room ", room, " Added to locations")
                    self.armor_client.manipulation.add_dataprop_to_ind("visitedAt", room, "Long", str(int(time.time())))
                else:
                    self.armor_client.manipulation.add_ind_to_class(room, "LOCATION")
                    self.armor_client.manipulation.add_objectprop_to_ind("isIn", "Robot1", room)
                    print("Robot is in ", room)

            for connection in response.connections:
                has_door = connection.through_door
                self.armor_client.manipulation.add_ind_to_class(has_door, "DOOR")
                # Adding doors to the ontology for the connections
                self.armor_client.manipulation.add_objectprop_to_ind('hasDoor', room, has_door)
                print("room ", room, "has door", has_door)
                self.door_list.append(has_door)
                # put doors in individuals list
                if has_door not in self.individuals_list:
                    self.individuals_list.append(has_door)
                
                print(room, " has door " , has_door)
            print(self.individuals_list)
        except rospy.ServiceException as e:
            print("Service call failed")
    
    
    def pub_position(self,joint1, joint2, joint3):
        """
        function to publish on the topics of the three joints.
        This function makes the arm moving in different positions to detect aruco markers. It goes through poses until all the markers have been detected.
        """

        link1.data = joint1
        link2.data = joint2
        link3.data = joint3
        pub1.publish(link1)
        pub2.publish(link2)
        pub3.publish(link3)

                                    ###############
                                    ### Move Arm ##
    ###############################################################################
    def execute_cb(self, goal):
        print(goal)
        positions = array([['pose0',0.0,0.0,0.0],['pose1',0.2,0.0,-0.7],['pose2',1.5,0.0,0.2],['pose3',1.5,0.0,-0.5],['pose4',2.2,0.0,0.2],['pose5',2.8,0.0,0.4],['pose6',-1.2,0,0.3],['pose7',-1.2,0,-0.5]])
        print(positions)
        success = True
        self.result.rooms_info = self.rooms_info

        if goal is None:
            print('no goal has been received')
            self.a_server.set_aborted()
            return

        i = 0
        while size(self.marker_list) <= 6:
            if self.a_server.is_preempt_requested():
                success = False
                break

            self.pub_position(float(positions[i][1]), float(positions[i][2]), float(positions[i][3]))
            rospy.sleep(1)

            if i == 7:
                i = 0
            else:
                i = i + 1

        if success:
            self.armor_client.manipulation.disjoint_all_ind(self.individuals_list)
            self.armor_client.utils.apply_buffered_changes()
            self.armor_client.utils.sync_buffered_reasoner()
            print(self.individuals_list)
            self.a_server.set_succeeded(self.result)

if __name__== '__main__':

    rospy.init_node('arm_controller')
    s = ArmControllerServer()
    rospy.spin()