#!/usr/bin/env python3

import rospy
import random
import time
from numpy import *
from std_msgs.msg import Float64
import actionlib
from inspection_robot.msg import MoveArmAction, MoveArmFeedback, MoveArmResult, MarkerList, RoomConnection
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
    def __init__(self):
        #self._action_name = name
        self.a_server = actionlib.SimpleActionServer("move_arm_as", MoveArmAction, execute_cb = self.execute_cb, auto_start = False)
        self.client = ArmorClient("assignment", "my_ontology")
        #self.client = actionlib.SimpleActionClient('create_ontology', RoomConnection)
        self.a_server.start()
        # Create the lists for storing markers, locations and room coordinates
        self.marker_list = []
        self.location_list = []
        self.room_coord = []
        self.door_list = []
        # Define the subscriber to the topic in which the marker's id are published
        rospy.Subscriber("marker_detector/MarkerList", MarkerList, self.marker_cb)
        self.send_id = rospy.ServiceProxy('/room_info', RoomInformation)
        self.path = dirname(realpath(__file__))
        # Put the path of the file.owl
        self.path = self.path + "/../../topological_map/"

        # Initializing with buffered manipulation and reasoning
        self.client.utils.load_ref_from_file(self.path + "topological_map.owl", "http://bnc/exp-rob-lab/2022-23", True, "PELLET", False, False)

        self.client.utils.mount_on_ref()
        self.client.utils.set_log_to_terminal(True)

    def marker_cb(self, msg):

        global marker_id 
        
        marker_id = msg.markers
        print(marker_id)
        
        # Making request to server 
        rospy.wait_for_service('/room_info')

        try:

            response = self.send_id(marker_id)
            room = response.room
            print(room)

            if room != 'no room associated with this marker id':
                self.marker_list.append(marker_id)
                print(self.marker_list)
                self.location_list.append(room)
                self.client.manipulation.add_ind_to_class(room, "LOCATION")
                self.client.manipulation.add_dataprop_to_ind("visitedAt", room, "Long", str(int(time.time())))
                #self.client.utils.apply_buffered_changes()
                #self.client.utils.sync_buffered_reasoner()
            
            x_coord = response.x
            y_coord = response.y
            self.room_coord = (x_coord, y_coord)
            print(self.room_coord)
            #array stating the connection and the door
            
            for i in range(len(response.connections)):
                connection = response.connections[i]
                has_door = connection.through_door
                self.client.manipulation.add_objectprop_to_ind('hasDoor', room, has_door)
                self.door_list.append(has_door)
                print(room, " has door " , has_door)

        except rospy.ServiceException as e:
            print("Service call failed")
       
    """
        function to publish on the topics of the three joints
    """
    def pub_position(self,joint1, joint2, joint3):

        link1.data = joint1
        link2.data = joint2
        link3.data = joint3
        pub1.publish(link1)
        pub2.publish(link2)
        pub3.publish(link3)

    def execute_cb(self, goal):
        print(goal)
        positions = array([['pose0',0.0,0.0,0.0],['pose1',0.2,0.0,-0.7],['pose2',1.5,0.0,0.2],['pose3',1.5,0.0,-0.5],['pose4',2.2,0.0,0.2],['pose5',2.8,0.0,0.4],['pose6',-1.2,0,0.3],['pose7',-1.2,0,-0.5]])
        success = True
        feedback = MoveArmFeedback()
        result = MoveArmResult()
        result.location_list = self.location_list
        #feedback.locations = marker_id

        if goal is None:
            print('no goal has been received')
            return

        i = 0
        while size(self.marker_list) <= 6:
            if self.a_server.is_preempt_requested():
                success = False
                break

            epsilon = 0#random.uniform(0, 0.15)
            self.pub_position(float(positions[i][1]), float(positions[i][2]), float(positions[i][3]))
            #self.pub_position(float(positions[i][1]) + epsilon, float(positions[i][2]) + epsilon , float(positions[i][3]) + epsilon)
            #self.a_server.publish_feedback(feedback)
            rospy.sleep(1)

            if i == 7:
                i = 0
            else:
                i = i + 1

        if success:
            self.a_server.set_succeeded(result)

if __name__== '__main__':

    rospy.init_node('arm_controller')
    s = ArmControllerServer()
    rospy.spin()