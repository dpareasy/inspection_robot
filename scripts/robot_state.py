#!/usr/bin/env python3
"""
.. module:: robot_state
   :platform: Unix
   :synopsis: Python module for implementing the robot state
.. moduleauthor:: Luca Buoncompagni, Davide Leo Parisi davide.parisi1084@gmail.com

ROS node for implementing the robot state. It includes the current position and battery level.

Publishes to: 
    /state/battery_low: the boolean stating if the battery is low or not

Service:
    /state/get_pose: get the robot current pose

    /state/set_pose: set the robot current pose
"""

import threading
import rospy
from std_msgs.msg import Bool
from helper import InterfaceHelper

class RobotState:
    """
    The node manager class.
    This class defines two services to get and set the current 
    robot pose, and a publisher to notify that the battery is low.
    """

    def __init__(self):
        # Initialise this node.
        rospy.init_node('robot-state', log_level = rospy.INFO)
        # Initialise robot position.
        self._pose = None
        # Initialise battery level.
        self._battery_low = False
        # Start publisher on a separate thread.
        th = threading.Thread(target = self.is_battery_low_)
        th.start()

    def is_battery_low_(self):
        """
        Publish changes of battery levels. This method runs on a separate thread.
        """
        # Define a `lathed` publisher to wait for initialisation and publish immediately.
        publisher = rospy.Publisher('state/battery_low', Bool, queue_size = 1, latch = True)
        # Publish battery level changes randomly.
        self.random_battery_notifier_(publisher)

    def random_battery_notifier_(self, publisher):
        """
        Publish when the battery change state (i.e., high/low) based on a random
        delay within the interval [`self._random_battery_time[0]`, `self._random_battery_time[1]`).
        The message is published through the `publisher` input parameter and is a
        boolean value, i.e., `True`: battery low, `False`: battery high.

        Args:
            publisher(publisher): publisher for the battery status.

        """
        delay = 0  # Initialised to 0 just for logging purposes.
        while not rospy.is_shutdown():
            # Publish battery level.
            publisher.publish(Bool(self._battery_low))
            # Log state.
            if self._battery_low:
                print("Robot got low battery after" , delay , "seconds")
                delay = 10
            else:
                print("Robot got fully charged battery after" , delay , "seconds")
                delay = 1800
            # Wait for simulate battery usage.
            rospy.sleep(delay)
            # Change battery state.
            self._battery_low = not self._battery_low

if __name__ == "__main__":
    """
    Initialize the robot position in [0,0]. init_robot_pose() function will make the request to the controller server
    """
    RobotState()
    helper = InterfaceHelper()
    rospy.spin()

