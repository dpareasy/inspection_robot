#! /usr/bin/env python3

"""
.. module:: robot_actions
   :platform: Unix
   :synopsis: Python module for implementing the Finite State Machine

.. moduleauthor:: Davide Leo Parisi <davide.parisi1084@gmail.com>

ROS node that helps the robot in performing its actions.
"""
# Import ROS libraries.
import random
import time
import simple_colors
from armor_api.armor_client import ArmorClient

client = ArmorClient("assignment", "my_ontology")


class BehaviorHelper:
    """
    A class to define the methods used for implementing the behavior of the robot. 
    """

    def __init__(self):
        
        self.agent = "Robot1"
        self.charging_location = "E"
        self.urgent = 'urgent'
        self.corridor = 'corridor'
        self.robot_position = 'robot_position'
        self.reachable_destinations = 'reachable_destinations'

    def inters_b2_lists(self, list1, list2):
        """
        Intersection between the urgent rooms and the reacheble rooms.

        Args:
            list1(str): first list to intersect

            list2(str): second list to intersect

        Returns:
            intersection(str): list of the intersection

        """
        intersection = [value for value in list1 if value in list2]
        return intersection

    def clean_strings(self, string_type, list_):
        """
        clean the strings obtained by the query.

        Args:
            string_type(int): an integer specifying if type of the string to clear (if locations or timestamps)
            
            list_(str): list of string to be cleaned 

        Returns:
            list_(str): list of cleaned strings

        """
        if string_type == 1:
            if len(list_) == 1:
                list_[0] = list_[0][32:-1]
                
            else:
                for x in range (len(list_)):
                    list_[x] = list_[x][32:-1]
        if string_type == 2:
            list_ = list_[0][1:-11]
        return list_

    def get_locations(self, location):
        """
        function performing the location's queries to aRMOR server.

        Args:
            location(str): a string to select the typr of query.

        Returns:
            location_list(str): list of queried location.

        """

        if location == self.corridor:
            list_of_corridors = client.query.ind_b2_class('CORRIDOR')
            location_list = self.clean_strings(1, list_of_corridors)
            
        if location == self.urgent:
            urgent_rooms = client.query.ind_b2_class('URGENT')
            location_list = self.clean_strings(1, urgent_rooms)

        if location == self.robot_position:
            current_pose = client.query.objectprop_b2_ind('isIn', self.agent)
            location = self.clean_strings(1, current_pose)
            location_list = current_pose[0]

        if location == self.reachable_destinations:
            possible_destinations = client.query.objectprop_b2_ind('canReach', self.agent)
            location_list = self.clean_strings(1, possible_destinations)

        return location_list


    def decide_target(self):
        """
        Function that plans the next robot movement.

        Returns:
            current_pose(str): The current robot position obtained from the ontology

        Returns:
            target(str): The target position chosen from a list of reachable and urgent locations
        
        Returns:
            list_of_corridors(str): List of corridors in the map

        """

        client.utils.sync_buffered_reasoner()
        list_of_corridors = self.get_locations(self.corridor)
        current_pose = self.get_locations(self.robot_position)
        print(simple_colors.green("\n\ncurrent pose is: ["+(current_pose)+"]"))
        possible_destinations = self.get_locations(self.reachable_destinations)
        print(simple_colors.green("Possible destinations: [" + ", ".join(possible_destinations) +"]"))
        urgent_rooms = self.get_locations(self.urgent)
        print(simple_colors.green("URGENT ROOMS: [" + ", " .join(urgent_rooms) + "]"))
        reachable_urgent = self.inters_b2_lists(possible_destinations, urgent_rooms)
        print(simple_colors.green("URGENT AND REACHABLE LOCATIONS: ["+", ".join(reachable_urgent)+"]"))
        # if the list is empty
        if not reachable_urgent:
            # if the list of possible destination 
            # contains only one element
            if len(possible_destinations) == 1:
                target = possible_destinations[0]
            else:
                target = random.choice(self.inters_b2_lists(possible_destinations, list_of_corridors))
        # if chosen_target list is not empty
        else:
            # save the first element of the list as the oldest timestamp
            oldest = client.query.dataprop_b2_ind('visitedAt', reachable_urgent[0])
            # clean the string
            oldest = self.clean_strings(2, oldest)
            for i in range (len(reachable_urgent)):
                choice_last_visit = client.query.dataprop_b2_ind('visitedAt', reachable_urgent[i])
                choice_last_visit = self.clean_strings(2, choice_last_visit)
                if choice_last_visit <= oldest:
                    target = reachable_urgent[i]
        print(simple_colors.green("TARGET " + target + " ACQUIRED\n\n"))
        return current_pose, target, list_of_corridors


    def move_to_target(self, chosen_target, current_pose, list_of_corridors):
        """
        Moving to the target room.

        Args:
            chosen_target(str): The target position chosen from a list of reachable and urgent locations
            current_pose(str): The current robot position obtained from the ontology
            list_of_corridors(str): List of corridors  in the map

        """
        

        if chosen_target not in list_of_corridors:
            last_visit = client.query.dataprop_b2_ind('visitedAt', chosen_target)
            last_visit = self.clean_strings(2, last_visit)

        client.manipulation.replace_objectprop_b2_ind('isIn', self.agent, chosen_target, current_pose)
        client.utils.sync_buffered_reasoner()
        last_change = client.query.dataprop_b2_ind('now', self.agent)
        last_change = self.clean_strings(2, last_change)
        current_time = str(int(time.time()))

        if chosen_target not in list_of_corridors:
            client.manipulation.replace_dataprop_b2_ind('visitedAt', chosen_target, 'Long', current_time, last_visit)

        client.manipulation.replace_dataprop_b2_ind('now', self.agent, 'Long', current_time, last_change)
        client.utils.sync_buffered_reasoner()
        print(simple_colors.magenta("\n\nRobot in " + chosen_target + "\n\n"))


    def go_to_recharge(self, current_location):
        """
        Going to charging location.

        Args:
            current_location(str): The current robot position obtained from the ontology

        """
        client.manipulation.replace_objectprop_b2_ind('isIn', self.agent, self.charging_location, current_location)
        client.utils.sync_buffered_reasoner()
