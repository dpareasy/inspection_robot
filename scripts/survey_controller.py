#!/usr/bin/env python3

"""
.. module:: my_state_machine
   :platform: Unix
   :synopsis: Python module for implementing the survey action

.. moduleauthor:: Davide Leo Parisi <davide.parisi1084@gmail.com>

ROS node for implementing the survey action.

Service:
    /surveyor: introspection server for visualization

Publishes:
    /cmd_vel: to make the robot scan the location
"""

import rospy
# Import the ActionServer implementation used.
from actionlib import SimpleActionServer
# Import custom message, actions and services.
from inspection_robot.msg import SurveyAction, SurveyResult
from geometry_msgs.msg import Twist
import inspection_robot  # This is required to pass the `PlanAction` type for instantiating the `SimpleActionServer`.

class SurveyRoom(object):
    """
    A class to implement the survey routine. 
    """
    def __init__(self):

        self._as = SimpleActionServer('surveyor',
                                        inspection_robot.msg.SurveyAction,
                                        execute_cb=self.execute_callback,
                                        auto_start=False)
        self._as.start()
    
    def execute_callback(self, goal):
        """
        Callback for making the robot scan the whole location after the client request.

        Args:
            goal: It contains the client request

        Returns:
            nothing

        """

        rate = rospy.Rate(10)

        goal = goal.survey

        if goal is None:
            print('No goal has been received')
            self._as.set_aborted()
            return
        
        # Publish a twist message to rotate the base link
        twist = Twist()
        """
        Twist(): robot velocity
        """
        twist.angular.z = 2.0
        pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        for i in range(200):
            pub.publish(twist)

            # Check if the action has been cancelled by the client
            if self._as.is_preempt_requested():
                self._as.set_preempted()
                return

        result = SurveyResult()
        result.result = 'SCANNED AREA'
        print(result.result)
        self._as.set_succeeded(result)
        return

if __name__ == '__main__':
    rospy.init_node("survey_controller")
    server = SurveyRoom()
    rospy.spin()