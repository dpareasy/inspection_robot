#!/usr/bin/env python3

"""
.. module:: my_state_machine
   :platform: Unix
   :synopsis: Python module for implementing the Finite State Machine

.. moduleauthor:: Davide Leo Parisi <davide.parisi1084@gmail.com>

ROS node for implementing the policy for choosing the room to visit.

Service:
    /server_name: introspection server for visualization

Publishes:
    /recharging_status: to simulate robot connection to power
"""

import smach
import rospy
import smach_ros
import actionlib
import actionlib.msg
from inspection_robot.msg import MoveArmGoal, MoveArmAction, MoveArmActionResult, SurveyGoal
from smach import State
from helper import InterfaceHelper
from robot_actions import BehaviorHelper
from inspection_robot.msg import RechStatus
from armor_api.armor_client import ArmorClient

client = ArmorClient("assignment", "my_ontology") 

#list of states in the machine
STATE_INIT = 'INITIALIZE_MAP'
STATE_DECISION = 'DECIDE_LOCATION'
STATE_MOVING = "MOVING_TO_LOCATION"
STATE_NORMAL = 'NORMAL'
STATE_RECHARGING = 'RECHARGING'
STATE_SURVEY = 'SURVEYING'
STATE_NORMAL = 'NORMAL'

# list of transition states
TRANS_INITIALIZED = 'everithing_loaded'
TRANS_DECIDED = 'target_acquired'
TRANS_MOVED = 'robot_moved'
TRANS_BATTERY_LOW = 'battery_low'
TRANS_RECHARGING = 'recharging'
TRANS_RECHARGED = 'recharged'
TRANS_SURVEYED = 'surveyed'
TRANS_NORMAL = 'trans_normal'

# Sleeping time (in seconds) of the waiting thread to allow the computations
# for getting stimulus from the other components of the architecture.
LOOP_SLEEP_TIME = 0.3

class LoadOntology(smach.State):
    """
    A class to implement the loading of the map.
    """
    def __init__(self):

        #self._action_helper = action_helper
        smach.State.__init__(self, outcomes = [TRANS_INITIALIZED], output_keys = ['rooms'], input_keys=['rooms'])
        self.client = actionlib.SimpleActionClient('move_arm_as', MoveArmAction)
        self.armor_client = ArmorClient("assignment", "my_ontology")
        self.client.wait_for_server()
        self.goal = MoveArmGoal()
        self.result = MoveArmActionResult()
        self.markers = []
        self.rooms = dict()
        self.room = None
        self.x_coord = None
        self.y_coord = None

    def feedback_cb(self, feedback):
        room_info = feedback.locations[0]
        self.room = room_info.room
        self.x_coord = room_info.x
        self.y_coord = room_info.y
        self.rooms[self.room] = (self.x_coord, self.y_coord)
        print('Feedback received: ', self.room, self.x_coord, self.y_coord)

    def execute(self, userdata):
        """
        Function responsible of the loading of the
        environment. It makes a request to move_arm_as server which 
        is the one responsible of scannign the aruco markers and creating the ontology. 

        Args:
            userdata: used for output_keys to make the dictionary with rooms' information available to the other states.

        Returns:
            TRANS_INITIALIZED(str): transition to the STATE_DECISION
        """
        # Create the action client that calls the server to move the arm
        self.goal.move = True
        self.client.send_goal(self.goal, feedback_cb = self.feedback_cb)
        self.client.wait_for_result()
        self.result = self.client.get_result()
        print(self.result.rooms_info)
        userdata.rooms = self.rooms
        print(userdata.rooms['E'])

        return TRANS_INITIALIZED

class DecideTarget(smach.State):
    """
    A class to implement the behavior of the reasoning state of the robot.
    """

    def __init__(self, interface_helper, behavior_helper):
        
        # Get a reference to the interfaces with the other nodes of the architecture.
        self._helper = interface_helper
        self._behavior = behavior_helper
        # Get the environment size from ROS parameters.
        self.environment_size = rospy.get_param('config/environment_size')
        smach.State.__init__(self, outcomes = [TRANS_RECHARGING, TRANS_DECIDED], input_keys = ['rooms'], output_keys = ['current_pose', 'choice'])

    def execute(self, userdata):
        """
        Function responsible of the transitions between the 
        STATE_DECISION and the STATE_RECHARGING or STATE_MOVING.
        The function will call several functions responsible of
        the decision state of the robot. It makes a request to the
        planner server to obtain the path to follow.

        Args:
            userdata: used for output_keys to pass data to the other states.

        Returns:
            TRANS_RECHARGING (str): transition to STATE_RECHARGING.
        
        Returns:
            TRANS_DECIDED (str): transition to the STATE_MOVING.

        """
        # current_pose, choice, list_of_corridors = self._behavior.decide_target()
        current_pose, choice = self._behavior.decide_target()
        print(choice)
        
        userdata.current_pose = current_pose
        userdata.choice = choice
        # userdata.list_of_corridors = list_of_corridors

        while not rospy.is_shutdown():
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._helper.mutex.acquire()
            try:
                # If the battery is low, then cancel the control action server and take the `battery_low` transition.
                if self._helper.is_battery_low():  # Higher priority

                    self._behavior.go_to_recharge(current_pose)
                    return TRANS_RECHARGING
                else:
                    return TRANS_DECIDED

            finally:
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._helper.mutex.release()

class MoveToTarget(smach.State):
    """
    A class to implement the behavior of the moving state of the robot.
    """

    def __init__(self, interface_helper, behavior_helper):
        # Get a reference to the interfaces with the other nodes of the architecture.
        self._helper = interface_helper
        self._behavior = behavior_helper

        smach.State.__init__(self, outcomes = [TRANS_RECHARGING, TRANS_MOVED], input_keys = ['rooms','current_pose', 'choice'], output_keys = ['current_pose'])

    def execute(self, userdata):
        """
        Function responsible of the transition between the 
        STATE_MOVING and the STATE_RECHARGING or STATE_DECISION.
        The function will call several functions responsible of 
        the movement of the robot. It makes a request to move_base action server to make the robot move to the target position.

        Args:
            userdata: for input_keys and output_keys to get data and pass data.
        
        Returns:
            TRANS_RECHARGING(str): transition to the STATE_RECHARGING.

        Returns:
            TRANS_MOVED(str): transition to STATE_SURVEY.

        """

        """
        ControlGoal: via points to reach the goal 
        """

        current_pose = userdata.current_pose
        choice = userdata.choice
        #list_of_corridors = userdata.list_of_corridors

        # take the dictionary as an input
        room_coordinates = userdata.rooms[choice]
        
        self._helper.move_base_client.send_goal(room_coordinates)
        print('Coordinates sent')
        while not rospy.is_shutdown():
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._helper.mutex.acquire()
            try:
                # If the battery is low, then cancel the control action server and take the `battery_low` transition.
                if self._helper.is_battery_low():  # Higher priority
                    self._helper.move_base_client.cancel_goals()
                    return TRANS_RECHARGING
                # If the controller finishes its computation, then take the `went_random_pose` transition, which is related to the `repeat` transition.
                if self._helper.move_base_client.is_done():
                    self._behavior.move_to_target(choice, current_pose)
                    userdata.current_pose = choice
                    return TRANS_MOVED
            finally:
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._helper.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)

class Surveying(State):
    """
    A class to implement the surveillance action of the robot, when it is inside a location.
    """

    def __init__(self, interface_helper, behavior_helper):
        # Get a reference to the interfaces with the other nodes of the architecture.
        self._helper = interface_helper
        self._behavior = behavior_helper
        # Initialise this state with possible transitions (i.e., valid outputs of the `execute` function).
        smach.State.__init__(self, outcomes = [TRANS_RECHARGING, TRANS_SURVEYED], input_keys = ['current_pose'])

    # Define the function performed each time a transition is such to enter in this state.
    # Note that the input parameter `userdata` is not used since no data is required from the other states.
    def execute(self, userdata):
        """
        Function responsible of the transition between the 
        STATE_SURVEY to the STATE_DECISION. It sends a request to the surveyor action client which implements the survey behavior. 
        If the battery is low it suddenly goes to the STATE_RECHARGING.

        Args:
            userdata: for input_keys to get data from the other states. 

        Returns:
            TRANS_RECHARGING(str): transition to STATE_RECHARGING.

        Returns:
            TRANS_SURVEYED(str): transition to STATE_DECISION.

        """

        goal = SurveyGoal()
        """
        SurveyGoal(): start survey
        """
        goal.survey = True
        self._helper.surveyor_client.send_request(goal)

        while not rospy.is_shutdown():  # Wait for stimulus from the other nodes of the architecture.            
            self._helper.mutex.acquire()
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            try:
                if self._helper.is_battery_low():  # Higher priority
                    self._helper.surveyor_client.cancel_goals()
                    return TRANS_RECHARGING
                
                if self._helper.surveyor_client.is_done():
                    return TRANS_SURVEYED
            finally:
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._helper.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)

class Recharging(State):
    """
    A class to implement the behavior of the recharging state of the robot.
    """

    def __init__(self, interface_helper, behavior_helper):
        # Get a reference to the interfaces with the other nodes of the architecture.
        self._helper = interface_helper
        self._behavior = behavior_helper
        # Initialise this state with possible transitions (i.e., valid outputs of the `execute` function).
        smach.State.__init__(self, outcomes = [TRANS_RECHARGED], input_keys=['rooms', 'current_pose'])

    # Define the function performed each time a transition is such to enter in this state.
    # Note that the input parameter `userdata` is not used since no data is required from the other states.
    def execute(self, userdata):
        """
        Function responsible of the transition between the 
        STATE_RECHARGING to the STATE_DECISION. It makes the request to move_base action server to move the robot in 
        the recharging location. Once in position it simulates the connection to the power for battery recharge by publishing
        on topic recharging_status to make the battery state starting recharging the battery.
        It waits until the battery is fully charged and then it changes state.
        The battery status is notified by the publisher robot_state. The function called to check the status `is_battery_low()`
        is defined in helper.py.
 
        Args:
            userdata:  input_keys used to take the rechargin room's information from the dictionary

        Returns:
            TRANS_RECHARGED(str): transition to STATE_DECISION

        """
        current_pose = userdata.current_pose
        charging_point = userdata.rooms['E']
        self._helper.move_base_client.send_goal(charging_point)
        pub = rospy.Publisher('recharging_status', RechStatus, queue_size=10)
        msg = RechStatus()
        """
        RechStatus(): enable recharging
        """
        msg.rech_status = True
        while not rospy.is_shutdown():  # Wait for stimulus from the other nodes of the architecture.
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._helper.mutex.acquire()
            try:
                # If the battery is no low anymore take the `charged` transition.
                if self._helper.move_base_client.is_done():                    
                    pub.publish(msg)
                if not self._helper.is_battery_low():
                    self._helper.reset_states()  # Reset the state variable related to the stimulus.
                    self._behavior.go_to_recharge(current_pose)
                    msg.rech_status = False
                    pub.publish(msg)
                    return TRANS_RECHARGED
            finally:
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._helper.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)

def main():
    """
    This function creates the state machine and defines all the transitions. Here a nested state machine is created.
    """
    rospy.init_node('state_machine', log_level = rospy.INFO)
    # Initialise an classes to manage the interfaces with the other nodes in the architecture.
    helper = InterfaceHelper()
    behavior = BehaviorHelper()
    sm_main = smach.StateMachine([])
    
    with sm_main:

        smach.StateMachine.add(STATE_INIT, LoadOntology(),
                         transitions = {TRANS_INITIALIZED: STATE_NORMAL})
        
        sm_normal = smach.StateMachine(outcomes=[TRANS_BATTERY_LOW], input_keys = ['rooms'], output_keys = ['current_pose'])

        with sm_normal:
            smach.StateMachine.add(STATE_DECISION, DecideTarget(helper, behavior),
                            transitions = {TRANS_RECHARGING : TRANS_BATTERY_LOW,
                                            TRANS_DECIDED: STATE_MOVING})
            smach.StateMachine.add(STATE_MOVING, MoveToTarget(helper, behavior),
                            transitions = {TRANS_RECHARGING : TRANS_BATTERY_LOW,
                                            TRANS_MOVED: STATE_SURVEY})
            smach.StateMachine.add(STATE_SURVEY, Surveying(helper, behavior),
                            transitions = {TRANS_RECHARGING : TRANS_BATTERY_LOW,
                                            TRANS_SURVEYED: STATE_DECISION})
        smach.StateMachine.add(STATE_NORMAL, sm_normal,
                         transitions={TRANS_BATTERY_LOW: STATE_RECHARGING})
            
        smach.StateMachine.add(STATE_RECHARGING, Recharging(helper, behavior),
                            transitions = {TRANS_RECHARGED: STATE_NORMAL})
    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm_main, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm_main.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
   
if __name__ == "__main__":
    main()