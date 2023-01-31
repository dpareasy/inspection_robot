#!/usr/bin/env python3

"""
.. module:: interface_helper
   :platform: Unix
   :synopsis: Python module for defining function used by the state machine

.. moduleauthor:: Luca Buoncompagni, Davide Leo Parisi <davide.parisi1084@gmail.com>

ROS node for defining the behavior of the state machine

Subscribes to:
    /state/battery_low: where the robot_state publishes the battery status.

Service:
    /state/set_pose: set the current position.

"""

# Import ROS libraries.
import rospy
from actionlib import SimpleActionClient

# Import mutex to manage synchronization among ROS-based threads (i.e., node loop and subscribers)
from threading import Lock
from armor_api.armor_client import ArmorClient

# Import ROS-based messages.
from std_msgs.msg import Bool
from Assignment1.msg import PlanAction, ControlAction
from Assignment1.srv import SetPose

client = ArmorClient("assignment", "my_ontology")

class ActionClientHelper:
    """
    A class to simplify the implementation of a client for ROS action servers. It is used by the `InterfaceHelper` class.

    """
    def __init__(self, service_name, action_type, done_callback=None, feedback_callback=None, mutex=None):
        # Initialise the state of this client, i.e.,  `_is_running`, `_is_done`, and `_results`.
        self.reset_client_states()
        # Set the name of the server to be invoked.
        self._service_name = service_name
        # Get or create a new mutex.
        if mutex is None:
            self._mutex = Lock()
        else:
            self._mutex = mutex
        # Instantiate a simple ROS-based action client.
        self._client = SimpleActionClient(service_name, action_type)
        # Set the done and feedback callbacks defined by the class using this client.
        self._external_done_cb = done_callback
        self._external_feedback_cb = feedback_callback
        # Wait for the action server to be alive.
        self._client.wait_for_server()

    def send_goal(self, goal):
        """
        Start the action server with a new `goal`. Note this call is not blocking (i.e., asynchronous performed).

        Args:
            goal(Point): Goal point.

        """
        # A new goal can be given to the action server only if it is not running. This simplification implies that
        # within the ROS architecture no more than one client can use the same server at the same time.
        if not self._is_running:
            # Start the action server.
            self._client.send_goal(goal,
                                   done_cb = self._done_callback,
                                   feedback_cb = self._feedback_callback)
            # Set the client's states.
            self._is_running = True
            self._is_done = False
            self._results = None
        else:
            print("Warning send a new goal, cancel the current request first!")

    def cancel_goals(self):
        """
        Stop the computation of the action server.
        """
        # The computation can be stopped only if the server is actually computing.
        if self._is_running:
            # Stop the computation.
            self._client.cancel_all_goals()
            # Reset the client's state.
            self.reset_client_states()
        else:
            print("Warning cannot cancel a not running service!")

    def reset_client_states(self):
        """
        Reset the client state variables stored in this class.
        """
        self._is_running = False
        self._is_done = False
        self._results = None

    def _feedback_callback(self, feedback):
        """
        This function is called when the action server send some `feedback` back to the client.

        Args:
            feedback: feedbacks from the action servers.

        """
        # Acquire the mutex to synchronise the computation concerning the `feedback` message with the other nodes of the architecture.
        self._mutex.acquire()
        try:
            # Eventually, call the method provided by the node that uses this action client to manage a feedback.
            if self._external_feedback_cb is not None:
                self._external_feedback_cb(feedback)
        finally:
            # Realise the mutex to (eventually) unblock ROS-based thread waiting on the same mutex.
            self._mutex.release()

    def _done_callback(self, status, results):
        """
        This function is called when the action server finish its 
        computation, i.e., it provides a `done` message.

        Args:
            status: status.

            results: results from the action servers.

        """
        # Acquire the mutex to synchronise the computation concerning the `done` message with the other nodes of the architecture.
        self._mutex.acquire()
        try:
            # Set the client's state
            self._is_running = False
            self._is_done = True
            self._results = results
            # Eventually, call the method provided by the node that uses this action client to manage a result.
            if self._external_done_cb is not None:
                self._external_done_cb(status, results)
        finally:
            self._mutex.release()

    def is_done(self):  # they should be mutex safe
        """
        Get `True` if the action server finished is computation, or `False` otherwise.
        Note that use this method should do it in a `self._mutex` safe manner.

        Returns:
            bool: `True` if the reasoner has finished its calculations, `False` otherwise.

        """
        return self._is_done

    def is_running(self):
        """
        Get `True` if the action server is running, or `False` otherwise.
        A note that use this method should do it in a `self._mutex` safe manner.

        Returns:
            bool: `True` If the server is still running, `False` otherwise.

        """
        return self._is_running
 
    def get_results(self):
        """
        Get the results of the action server, if any, or `None` and return this value.

        Returns:
            self.results: Some results have arrived, None otherwise.

        """
        if self._is_done:
            return self._results
        else:
            print("Error: cannot result")
            return None

class InterfaceHelper:
    """
    A class to decouple the implementation of the Finite State Machine to the stimulus might that
    lead to state transitions. This class manages the synchronization with subscribers and action
    servers.
    """

    # Class constructor, i.e., class initializer.
    def __init__(self):
        # Create a shared mutex to synchronize action clients and subscribers.
        # Note that, based on different assumptions, further optimization can be done to make the different threads
        # blocking for a less amount of time in the same mutex.
        self.mutex = Lock()
        # Set the initial state involving the `self._battery_low`, `self._start_interaction` and `self._gesture` variables.
        self.reset_states()
        # Define the callback associated with the speech, gesture, and battery low ROS subscribers.
        rospy.Subscriber('state/battery_low', Bool, self._battery_callback)
        # Define the clients for the the plan and control action servers.
        self.planner_client = ActionClientHelper('motion/planner', PlanAction, mutex=self.mutex)
        self.controller_client = ActionClientHelper('motion/controller', ControlAction, mutex=self.mutex)

    def reset_states(self):
        """
        Reset the stimulus, which are stored as states variable fo this class.
        This function assumes that no states of the Finite State Machine run concurrently.

        """
        self._battery_low = False
        self._start_interaction = False
        self._gesture = None

    def _battery_callback(self, msg):
        """
        The subscriber to get messages published from the `robot-state` node into the `/state/battery_low/` topic.
        """
        # Acquire the mutex to assure the synchronization with the other subscribers and action clients (this assure data consistency).
        self.mutex.acquire()
        try:
            # Get the battery level and set the relative state variable encoded in this class.
            self._battery_low = msg.data
        finally:
            # Release the mutex to eventually unblock the other subscribers or action servers that are waiting.
            self.mutex.release()

    def is_battery_low(self):
        """
        Get the state variable encoded in this class that concerns the battery level.
        The returning value will be `True` if the battery is low, `False` otherwise.
        Note that the node using this class might exploit the `reset_state` function to improve robustness.
        Also note that this function should be used when the `mutex` has been acquired. This assures the
        synchronization  with the threads involving the subscribers and action clients.

        Returns:
            bool: `True` if the battery is low, `False` otherwise.

        """
        return self._battery_low

    # Update the current robot pose stored in the `robot-state` node.
    @staticmethod
    def init_robot_pose(point):
        """
        Function that initialise the robot position. It calls the server for initialising the robot position inside the environment.

        Args:
            point(Point): robot initial position.
        """
        # Eventually, wait for the server to be initialised.
        rospy.wait_for_service('state/set_pose')
        try:
            # Call the service and set the current robot position.
            service = rospy.ServiceProxy('state/set_pose', SetPose)
            service(point)  # None that the service `response` is not used.
            print("Setting initial robot position")
        except rospy.ServiceException as e:
            print("Cannot set current robot position")
