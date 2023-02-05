#!/usr/bin/env python3
"""
.. module:: my_state_machine
   :platform: Unix
   :synopsis: Python module for implementing the Finite State Machine

.. moduleauthor:: Davide Leo Parisi <davide.parisi1084@gmail.com>

ROS node for implementing the Finite State Machine.

Service:
    /server_name: introspection server for visualization
"""

import smach
import rospy
import random
import smach_ros
import time
import actionlib
from inspection_robot.msg import MoveArmGoal, MoveArmAction, MoveArmActionFeedback, MoveArmActionResult
#from threading import Lock
#from std_msgs.msg import Bool
from smach import State #StateMachine, State
from helper import InterfaceHelper, ActionClientHelper
from robot_actions import BehaviorHelper
from inspection_robot.msg import MarkerList
#from load_ontology import CreateMap
from Assignment1.msg import Point, ControlGoal, PlanGoal
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
    A class to implement the behavior of the decision state of the robot.
    """
    def __init__(self):

        #self._action_helper = action_helper
        State.__init__(self, outcomes = [TRANS_INITIALIZED])
        self.client = actionlib.SimpleActionClient('move_arm_as', MoveArmAction)
        self.armor_client = ArmorClient("assignment", "my_ontology")
        self.client.wait_for_server()
        self.goal = MoveArmGoal()
        self.result = MoveArmActionResult()
        self.markers = []

    def feedback_cb(self, feedback):
        room_info = feedback.locations[0]
        room = room_info.room
        print('Feedback received: ', room)
    
    def markers_callback(self, msg):
        self.markers = msg.markers

    def execute(self, userdata):
        """
        Function responsible of the loading of the
        environment. It calls the LoadMap() function which 
        is the one responsible of the creation of the environment. 
        The input parameter `userdata` is not used since no data is 
        required from the other states.

        Args:
            userdata: not used

        Returns:
            TRANS_INITIALIZED(str): transition to the STATE_DECISION
        """
        # Create the action client that calls the server to move the arm
        self.goal.move = True
        self.client.send_goal(self.goal, feedback_cb = self.feedback_cb)
        self.client.wait_for_result()
        self.result = self.client.get_result()
        print(self.result.rooms_info)
        #print(self.result.individuals_list[2])
        #print("Markers detected: ", self.markers)

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
        State.__init__(self, outcomes = [TRANS_RECHARGING, TRANS_DECIDED], output_keys = ['current_pose', 'choice', 'list_of_corridors', 'random_plan'])

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
        # Define a random point to be reached through some via-points to be planned.
        goal = PlanGoal()
        """
        PlanGoal: goal position
        """
        goal.target = Point(x = random.uniform(0, self.environment_size[0]),
                            y = random.uniform(0, self.environment_size[1]))
        current_pose, choice, list_of_corridors = self._behavior.decide_target()
        userdata.current_pose = current_pose
        userdata.choice = choice
        userdata.list_of_corridors = list_of_corridors
        # Invoke the planner action server.
        self._helper.planner_client.send_goal(goal)
        while not rospy.is_shutdown():
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._helper.mutex.acquire()
            try:
                # If the battery is low, then cancel the control action server and take the `battery_low` transition.
                if self._helper.is_battery_low():  # Higher priority
                    self._helper.planner_client.cancel_goals()
                    self._behavior.go_to_recharge(current_pose)
                    return TRANS_RECHARGING
                # If the controller finishes its computation, then take the `went_random_pose` transition, which is related to the `repeat` transition.
                if self._helper.planner_client.is_done():
                    userdata.random_plan = self._helper.planner_client.get_results().via_points
                    return TRANS_DECIDED

            finally:
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._helper.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)

class MoveToTarget(smach.State):
    """
    A class to implement the behavior of the moving state of the robot.
    """

    def __init__(self, interface_helper, behavior_helper):
        # Get a reference to the interfaces with the other nodes of the architecture.
        self._helper = interface_helper
        self._behavior = behavior_helper

        State.__init__(self, outcomes = [TRANS_RECHARGING, TRANS_MOVED], input_keys = [ "random_plan",'current_pose', 'choice', 'list_of_corridors'], output_keys = ['current_pose'])

    def execute(self, userdata):
        """
        Function responsible of the transition between the 
        STATE_MOVING and the STATE_RECHARGING or STATE_DECISION.
        The function will call several functions responsible of 
        the movement of the robot. It makes a request to the controller
        server which is the one responsible of the movement.

        Args:
            userdata: for input_keys and output_keys to get data and pass data.
        
        Returns:
            TRANS_RECHARGING(str): transition to the STATE_RECHARGING.

        Returns:
            TRANS_MOVED(str): transition to STATE_SURVEY.

        """
        # Get the plan to a random position computed by the `PLAN_TO_RANDOM_POSE` state.
        plan = userdata.random_plan
        # Start the action server for moving the robot through the planned via-points.
        goal = ControlGoal(via_points = plan)
        """
        ControlGoal: via points to reach the goal 
        """
        self._helper.controller_client.send_goal(goal)
        current_pose = userdata.current_pose
        choice = userdata.choice
        list_of_corridors = userdata.list_of_corridors

        while not rospy.is_shutdown():
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._helper.mutex.acquire()
            try:
                # If the battery is low, then cancel the control action server and take the `battery_low` transition.
                if self._helper.is_battery_low():  # Higher priority
                    self._helper.controller_client.cancel_goals()
                    self._behavior.go_to_recharge(current_pose)
                    return TRANS_RECHARGING
                # If the controller finishes its computation, then take the `went_random_pose` transition, which is related to the `repeat` transition.
                if self._helper.controller_client.is_done():
                    self._behavior.move_to_target(choice, current_pose, list_of_corridors)
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
        State.__init__(self, outcomes = [TRANS_RECHARGING, TRANS_SURVEYED], input_keys = ['current_pose'])

    # Define the function performed each time a transition is such to enter in this state.
    # Note that the input parameter `userdata` is not used since no data is required from the other states.
    def execute(self, userdata):
        """
        Function responsible of the transition between the 
        STATE_SURVEY to the STATE_DECISION. It waits a specified amount of time 
        for the survey of the location and then make tha transition. If the
        battery is low it suddenly goes to the STATE_RECHARGING

        Args:
            userdata: for input_keys to get data from the other states. 

        Returns:
            TRANS_RECHARGING(str): transition to STATE_RECHARGING.

        Returns:
            TRANS_SURVEYED(str): transition to STATE_DECISION.

        """
        current_pose = userdata.current_pose
        timer = 0
        while (not rospy.is_shutdown()):  # Wait for stimulus from the other nodes of the architecture.
            
            while(timer != 500) and (not self._helper.is_battery_low()):
                self._helper.mutex.acquire()
                # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
                timer = timer + 1
                time.sleep(0.01)
                try:
                    if self._helper.is_battery_low():  # Higher priority
                        self._behavior.go_to_recharge(current_pose)
                        return TRANS_RECHARGING
                    
                    if timer == 500:
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
        State.__init__(self, outcomes = [TRANS_RECHARGED])

    # Define the function performed each time a transition is such to enter in this state.
    # Note that the input parameter `userdata` is not used since no data is required from the other states.
    def execute(self, userdata):
        """
        Function responsible of the transition between the 
        STATE_RECHARGING to the STATE_DECISION.
        It waits until the battery is fully charged and then it changes state.
        The battery status is notified by the publisher robot_state. The function called to check the status `is_battery_low()`
        is defined in helper.py.
 
        Args:
            userdata: not used

        Returns:
            TRANS_RECHARGED(str): transition to STATE_DECISION

        """
        while not rospy.is_shutdown():  # Wait for stimulus from the other nodes of the architecture.
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._helper.mutex.acquire()
            try:
                # If the battery is no low anymore take the `charged` transition.
                if not self._helper.is_battery_low():
                    self._helper.reset_states()  # Reset the state variable related to the stimulus.
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
    #ontology = CreateMap()
    helper = InterfaceHelper()
    #action_helper = ActionClientHelper()
    behavior = BehaviorHelper()
    sm_main = smach.StateMachine([])
    
    with sm_main:

        smach.StateMachine.add(STATE_INIT, LoadOntology(),
                         transitions = {TRANS_INITIALIZED: STATE_NORMAL})
        
        sm_normal = smach.StateMachine(outcomes=[TRANS_BATTERY_LOW])

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